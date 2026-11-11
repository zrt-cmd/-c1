#!/usr/bin/env python3
"""Track a reference route or navigate to a single goal in the Gazebo room."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

TUNE = Path(__file__).resolve().parent
WORLD = 'math_room'
CONTACT_PREFIX = '/reference_trial/contact/'
ROUTE_FILE = TUNE / 'demonstrations/manual_fast_20261110/route.json'


def instrument_world(source, output):
    tree = ET.parse(source)
    walls = {}
    for model in tree.findall('world/model'):
        name = model.attrib['name']
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if size is None or model.findtext('static') != 'true':
                    continue
                x, y, *_ = map(float, model.findtext('pose').split())
                width, height, _ = map(float, size.split())
                walls[name] = (x, y, width, height)
                sensor = ET.SubElement(link, 'sensor', name='wall_contact', type='contact')
                contact = ET.SubElement(sensor, 'contact')
                ET.SubElement(contact, 'collision').text = collision.attrib['name']
                ET.SubElement(contact, 'topic').text = CONTACT_PREFIX + name
    if not walls:
        raise RuntimeError('The simulation world has no wall boxes to monitor')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return walls


def clearance(pose, walls):
    x, y, yaw = pose
    half_x = .18 * abs(math.cos(yaw)) + .16 * abs(math.sin(yaw))
    half_y = .18 * abs(math.sin(yaw)) + .16 * abs(math.cos(yaw))
    distances = []
    for name, (cx, cy, width, height) in walls.items():
        dx = abs(x - cx) - width / 2 - half_x
        dy = abs(y - cy) - height / 2 - half_y
        gap = math.hypot(max(dx, 0), max(dy, 0)) if max(dx, dy) > 0 else max(dx, dy)
        distances.append((gap, name))
    return min(distances)


def wait_until(predicate, timeout, description, spin=lambda: time.sleep(.02)):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise RuntimeError(f'Timed out waiting for {description}')
        spin()


def stop_launch(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def run(args, walls, planning_grid):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Path as PathMessage
    from nav2_msgs.action import ComputePathToPose, FollowPath
    from rclpy.action import ActionClient
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist as GzTwist
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node as GzNode

    rclpy.init()
    node = rclpy.create_node('follow_path_evaluator', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    client = ActionClient(node, FollowPath, 'follow_path')
    controller_state = node.create_client(GetState, '/controller_server/get_state')
    gz = GzNode()
    brake = gz.advertise('/cmd_vel', GzTwist)
    state = {'pose': None, 't': None, 'pose_received': 0., 'command': (0., 0., 0.),
             'probe_present': False, 'probe_seen': False, 'local_goal': (None, None)}
    events = []
    handle = None
    result_future = None
    result = {'status': 'setup_failed', 'success': False,
              'route_source': 'theta_star' if args.goal else 'reference_route',
              'completed_actions': 0}

    def spin():
        rclpy.spin_once(node, timeout_sec=.02)

    def stats(message):
        state['t'] = message.sim_time.sec + message.sim_time.nsec * 1e-9

    def poses(message):
        names = {pose.name for pose in message.pose}
        state['probe_present'] = 'contact_check_box' in names
        state['probe_seen'] |= state['probe_present']
        for pose in message.pose:
            if pose.name == 'car1':
                q = pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                state['pose'] = (pose.position.x, pose.position.y, yaw)
                state['pose_received'] = time.monotonic()

    def contacts(message, wall):
        if message.contact:
            contact = message.contact[0]
            events.append({'wall': wall, 'sim_time': state['t'],
                           'collision_1': contact.collision1.name or contact.collision1.id,
                           'collision_2': contact.collision2.name or contact.collision2.id})

    def command(message):
        state['command'] = (message.linear.x, message.linear.y, message.angular.z)

    def local_plan(message):
        if message.poses:
            position = message.poses[-1].pose.position
            state['local_goal'] = (position.x, position.y)

    def send_path(path):
        goal = FollowPath.Goal()
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        goal.progress_checker_id = 'progress_checker'
        goal.path.header.frame_id = 'map'
        goal.path.header.stamp = node.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.
            goal.path.poses.append(pose)
        future = client.send_goal_async(goal)
        wait_until(future.done, 10, 'FollowPath goal acceptance', spin)
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError('Nav2 rejected the tracking path')
        return goal_handle

    def plan_from_localization():
        from goal_planning import split_at_turns

        planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        wait_until(planner.server_is_ready, 15, 'ComputePathToPose action', spin)
        request = ComputePathToPose.Goal()
        request.planner_id = 'FootprintThetaStar'
        request.use_start = False
        request.goal.header.frame_id = 'map'
        request.goal.header.stamp = node.get_clock().now().to_msg()
        request.goal.pose.position.x, request.goal.pose.position.y = args.goal
        request.goal.pose.orientation.w = 1.
        future = planner.send_goal_async(request)
        wait_until(future.done, 10, 'planner goal acceptance', spin)
        plan_handle = future.result()
        if not plan_handle.accepted:
            raise RuntimeError('ThetaStar rejected the target')
        future = plan_handle.get_result_async()
        wait_until(future.done, 30, 'ThetaStar route', spin)
        response = future.result()
        result.update(planner_action_status=response.status,
                      planner_error_code=response.result.error_code,
                      planner_error_message=response.result.error_msg)
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'ThetaStar failed with code {response.result.error_code}')
        route = [[pose.pose.position.x, pose.pose.position.y] for pose in response.result.path.poses]
        (args.out / 'planned_path.json').write_text(json.dumps(route))
        if not route or math.dist(route[-1], args.goal) > .02:
            raise RuntimeError('ThetaStar did not return a path to the requested target')
        if math.dist(route[-1], args.goal) > 1e-5:
            route.append(args.goal)
        waypoints = split_at_turns(route, planning_grid)
        (args.out / 'waypoints.json').write_text(json.dumps(waypoints, indent=2) + '\n')
        result.update(requested_goal=args.goal, planned_start=route[0],
                      planner_used_explicit_start=False, planned_waypoints=len(waypoints))
        print(f'ThetaStar planned from AMCL to {args.goal}: {len(waypoints) - 1} segments',
              flush=True)
        return waypoints

    gz.subscribe(WorldStatistics, f'/world/{WORLD}/stats', stats)
    gz.subscribe(Pose_V, f'/world/{WORLD}/pose/info', poses)
    for wall in walls:
        gz.subscribe(Contacts, CONTACT_PREFIX + wall, lambda msg, wall=wall: contacts(msg, wall))
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(PathMessage, '/transformed_global_plan', local_plan, 1)
    try:
        wait_until(lambda: state['pose'] and state['t'] is not None, 60, 'the full robot', spin)
        expected_topics = {CONTACT_PREFIX + wall for wall in walls}
        wait_until(lambda: expected_topics <= set(gz.topic_list()), 10, 'all wall contact topics', spin)

        # Verify that physical contact reaches the evaluator before driving. The
        # temporary box is placed at a distant outer wall, then removed.
        wall = 'wall_south_right'
        x, y, _, height = walls[wall]
        box = EntityFactory(sdf=f"""<sdf version="1.9"><model name="contact_check_box">
          <pose>{x} {y + height / 2 + .04} 0.25 0 0 0</pose><link name="body">
          <collision name="box"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision>
          </link></model></sdf>""")
        for _ in range(3):
            if state['probe_seen']:
                break
            ok, reply = gz.request(f'/world/{WORLD}/create', box, EntityFactory, Boolean, 4000)
            if ok and reply.data:
                break
        else:
            raise RuntimeError('Could not spawn the wall-contact check box')
        wait_until(lambda: any(event['wall'] == wall for event in events)
                   and state['probe_seen'], 10, 'a physical wall contact from the check box', spin)
        result['contact_sensor_check'] = next(event for event in events if event['wall'] == wall)
        entity = Entity(name='contact_check_box', type=Entity.MODEL)
        ok, reply = gz.request(f'/world/{WORLD}/remove', entity, Entity, Boolean, 4000)
        if not ok or not reply.data:
            raise RuntimeError('Could not remove the wall-contact check box')
        wait_until(lambda: not state['probe_present'], 10, 'check box removal', spin)
        settle_until = state['t'] + .3
        wait_until(lambda: state['t'] >= settle_until, 10, 'contact check settling', spin)
        events.clear()
        print(f'Contact check passed, {len(walls)} walls monitored', flush=True)

        subprocess.run([sys.executable, str(TUNE / 'initialize_localization.py'), '--out',
                        str(args.out / 'initialization.json')], timeout=100, check=True,
                       stdout=subprocess.DEVNULL)
        wait_until(controller_state.service_is_ready, 20, 'controller lifecycle service', spin)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            response = controller_state.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, response, timeout_sec=2)
            if response.done() and response.result().current_state.id == 3:
                break
        else:
            raise RuntimeError('Controller server did not become active')
        if args.goal:
            result['status'] = 'planning_failed'
            waypoints = plan_from_localization()
        else:
            waypoints = json.loads(ROUTE_FILE.read_text())
            if math.dist(state['pose'][:2], waypoints[0]) > .03:
                raise RuntimeError('The simulated car did not start at G')
        destination = args.goal if args.goal else waypoints[-1]
        legs = []
        for origin, end in zip(waypoints, waypoints[1:]):
            samples = math.ceil(math.dist(origin, end) / .015)
            legs.append([[origin[0] + (end[0] - origin[0]) * i / samples,
                          origin[1] + (end[1] - origin[1]) * i / samples]
                         for i in range(samples + 1)])
        points = legs[0] + [point for leg in legs[1:] for point in leg[1:]]
        paths = [points] if args.continuous else legs
        result['path_actions'] = len(paths)
        if args.goal:
            # Independent Gazebo geometry check before allowing motion.
            gap = min(clearance((x, y, 0.), walls)[0] for x, y in points)
            result['planned_min_envelope_clearance_m'] = gap
            if gap < .015:
                raise RuntimeError(f'Planned path has only {gap:.3f} m of geometric clearance')
        wait_until(client.server_is_ready, 10, 'FollowPath action', spin)

        (args.out / 'path.json').write_text(json.dumps(points))
        start = state['t']
        last_pose = state['pose']
        last_log = start - 1
        next_status = start
        min_gap, nearest_wall = clearance(last_pose, walls)
        distance = 0.
        lateral_peak = 0.
        wall_deadline = time.monotonic() + 720
        print(f'Following {len(points)} poses in {len(paths)} action(s)', flush=True)
        result['status'] = 'tracking'
        with (args.out / 'telemetry.csv').open('w', newline='') as file:
            log = csv.writer(file)
            log.writerow(['sim_seconds', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'clearance_m', 'wall',
                          'local_goal_x', 'local_goal_y', 'action'])
            for index, path in enumerate(paths, 1):
                handle = send_path(path)
                result_future = handle.get_result_async()
                while not result_future.done():
                    spin()
                    t, pose = state['t'], state['pose']
                    gap, wall = clearance(pose, walls)
                    if gap < min_gap:
                        min_gap, nearest_wall = gap, wall
                    distance += math.dist(pose[:2], last_pose[:2])
                    last_pose = pose
                    lateral_peak = max(lateral_peak, abs(state['command'][1]))
                    if t - last_log >= .1:
                        log.writerow([t - start, *pose, *state['command'], gap, wall,
                                      *state['local_goal'], index])
                        file.flush()
                        last_log = t
                    if t >= next_status:
                        print(f'{t - start:.1f} sim s, action {index}: '
                              f'({pose[0]:.3f}, {pose[1]:.3f}), clearance {gap:.3f} m', flush=True)
                        next_status = t + 12.
                    if events:
                        result['status'] = 'wall_contact'
                    elif gap < .008:
                        result['status'] = 'clearance_guard'
                    elif time.monotonic() - state['pose_received'] > 2:
                        result['status'] = 'pose_stream_stale'
                    elif t - start >= args.timeout or time.monotonic() >= wall_deadline:
                        result['status'] = 'timeout'
                    if result['status'] != 'tracking':
                        break
                if result['status'] != 'tracking':
                    break
                response = result_future.result()
                result.update(action_status=response.status, error_code=response.result.error_code,
                              error_message=response.result.error_msg)
                if response.status != GoalStatus.STATUS_SUCCEEDED:
                    result['status'] = 'controller_failed'
                    break
                result['completed_actions'] = index
                print(f'Action {index}/{len(paths)} reached at {state["t"] - start:.1f} sim s',
                      flush=True)
            else:
                result['status'] = 'reached_goal' if args.goal else 'reached_C'
        if not result_future.done():
            handle.cancel_goal_async()
        for _ in range(10):
            brake.publish(GzTwist())
            spin()
        endpoint_error = math.dist(state['pose'][:2], destination)
        recorded_contacts = list(events)
        result.update(pose=state['pose'], endpoint_error_m=endpoint_error,
                      sim_seconds=state['t'] - start, path_length_m=distance,
                      min_envelope_clearance_m=min_gap, nearest_wall=nearest_wall,
                      peak_lateral_command_mps=lateral_peak, wall_contacts=recorded_contacts[:10],
                      wall_contact_messages=len(recorded_contacts), instrumented_walls=len(walls))
        reached = result['status'] in ('reached_C', 'reached_goal')
        result['success'] = (reached and endpoint_error <= .05
                             and not recorded_contacts)
        if recorded_contacts:
            result['status'] = 'wall_contact'
        elif reached and not result['success']:
            result['status'] = 'goal_outside_truth_tolerance'
    except Exception as exc:
        result['error'] = str(exc)
        raise
    finally:
        if handle is not None and result_future is not None and not result_future.done():
            handle.cancel_goal_async()
        brake.publish(GzTwist())
        (args.out / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return result['success']


def main():
    from ament_index_python.packages import get_package_share_directory

    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--params', type=Path, default=TUNE / 'config/follow_path.yaml')
    parser.add_argument('--timeout', type=float, default=240.)
    parser.add_argument('--visual', action='store_true')
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'),
                        help='Plan from the current AMCL pose to a map-frame target')
    parser.add_argument('--continuous', action='store_true',
                        help='Send the entire route as one action instead of stopping at corners')
    args = parser.parse_args()
    if args.goal and not all(math.isfinite(value) for value in args.goal):
        parser.error('Goal coordinates must be finite')
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.params, args.out / 'controller_params.yaml')
    world = Path(get_package_share_directory('car1_gazebo')) / 'worlds/math_room.sdf'
    instrumented = args.out / 'world_with_contacts.sdf'
    walls = instrument_world(world, instrumented)
    planning_grid = None
    planning_map = ''
    if args.goal:
        from goal_planning import make_planning_map

        planning_map = args.out / 'planning_map.yaml'
        planning_grid = make_planning_map(TUNE / 'maps/math_room_1cm.yaml', planning_map,
                                          args.params)
        shutil.copyfile(TUNE / 'config/goal_planner.yaml', args.out / 'planner_params.yaml')
    os.environ.update(GZ_PARTITION=f'car1_follow_path_{os.getpid()}',
                      ROS_DOMAIN_ID=str(80 + os.getpid() % 80),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      DISPLAY=':0', GALLIUM_DRIVER='d3d12', QT_QPA_PLATFORM='xcb')
    os.environ.pop('WAYLAND_DISPLAY', None)
    os.environ.pop('ROS_STATIC_PEERS', None)
    command = ['ros2', 'launch', str(TUNE / 'follow_path_probe.launch.py'),
               f'world_file:={instrumented}', f'controller_params:={args.params.resolve()}',
               f'headless:={str(not args.visual).lower()}', f'planning_map:={planning_map}']
    with (args.out / 'launch.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            return 0 if run(args, walls, planning_grid) else 1
        finally:
            stop_launch(process)


if __name__ == '__main__':
    sys.exit(main())
