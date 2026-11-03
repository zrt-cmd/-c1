#!/usr/bin/env python3
"""Measure physical motion in isolated copies of the competition world."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--body-frame", action="store_true")
    parser.add_argument("--visual", action="store_true")
    parser.add_argument("--capture-dir", type=Path)
    parser.add_argument("--workspace", type=Path,
                        default=Path(os.environ.get("CAR1_WS", str(Path.home() / "ros2_ws"))))
    args = parser.parse_args()
    if args.visual:
        os.environ["DISPLAY"] = ":0"
        os.environ["GALLIUM_DRIVER"] = "d3d12"
        os.environ["QT_QPA_PLATFORM"] = "xcb"
        os.environ.pop("WAYLAND_DISPLAY", None)
        if args.capture_dir is None:
            parser.error("--visual requires --capture-dir for frame checkpoints")
        args.capture_dir.mkdir(parents=True, exist_ok=True)
    os.environ["GZ_PARTITION"] = "car1_lateral_check_" + str(os.getpid())
    os.environ["ROS_DOMAIN_ID"] = "218"
    from gz.transport13 import Node
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist
    from gz.msgs10.world_stats_pb2 import WorldStatistics

    workspace = args.workspace.expanduser().resolve()
    script_directory = Path(__file__).resolve().parent
    output_directory = script_directory / "results"
    output_directory.mkdir(exist_ok=True)
    source = workspace / "src/xflz_eai_car1/simulations"
    robot = ET.parse(script_directory / "fixtures/car1_lateral_current.sdf").getroot().find("model")
    if robot.find("pose") is None:
        ET.SubElement(robot, "pose")
    robot.find("pose").text = "-1.8 -1.0 0.1 0 0 0"
    # This plugin only controls the arm/gripper and crashed during startup.
    # MecanumDrive controls the wheels independently through Gazebo Transport.
    for plugin in list(robot.findall("plugin")):
        if plugin.get("name") == "gz_ros2_control::GazeboSimROS2ControlPlugin":
            robot.remove(plugin)
    # Resolve model:// paths exactly as the normal launch resolves its meshes.
    for uri in robot.iter("uri"):
        if uri.text.startswith("model://car1_description/"):
            uri.text = "file://" + str(workspace / "install/car1_description/share/car1_description" / uri.text.removeprefix("model://car1_description/"))
    if args.body_frame:
        ET.register_namespace("gz", "http://gazebosim.org/schema")
        for link in robot.findall("link"):
            if link.get("name", "").startswith("wheel_"):
                for direction in link.findall("collision/surface/friction/ode/fdir1"):
                    direction.set("{http://gazebosim.org/schema}expressed_in", "base_footprint")

    world_tree = ET.parse(source / "car1_gazebo/worlds/math_room.sdf")
    world = world_tree.getroot().find("world")
    # Physics-only diagnostic. Walls and robot collision geometry are preserved.
    for plugin in list(world.findall("plugin")):
        if plugin.get("name") == "gz::sim::systems::Sensors":
            world.remove(plugin)
    world.append(copy.deepcopy(robot))
    node = Node()
    state = {"time": None, "pose": None, "odom": None}

    def stats(message):
        state["time"] = message.sim_time.sec + message.sim_time.nsec * 1e-9

    def poses(message):
        for pose in message.pose:
            if pose.name == "car1":
                state["pose"] = copy.deepcopy(pose)

    def odometry(message):
        state["odom"] = copy.deepcopy(message.pose)

    node.subscribe(WorldStatistics, "/world/math_room/stats", stats)
    node.subscribe(Pose_V, "/world/math_room/pose/info", poses)
    node.subscribe(Odometry, "/odom", odometry)
    publisher = node.advertise("/cmd_vel", Twist)
    stop = Twist()
    results = []

    def wait_until(predicate, seconds=30):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out waiting for simulation state")
            time.sleep(0.02)

    def yaw(pose):
        q = pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def drive(x, y, angular, duration):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = x, y, angular
        initial_time = state["time"]
        deadline = time.monotonic() + 35
        while state["time"] - initial_time < duration:
            if time.monotonic() > deadline:
                raise RuntimeError("Simulation stopped advancing")
            publisher.publish(message)
            time.sleep(0.02)
        publisher.publish(stop)

    def checkpoint(name):
        if not args.visual:
            return
        marker = args.capture_dir / (name + ".ready")
        marker.write_text(json.dumps({"stage": name, "sim_time": state["time"]}))
        print("CAPTURE " + name, flush=True)
        wait_until(lambda: (args.capture_dir / (name + ".continue")).exists(), 180)

    with tempfile.TemporaryDirectory(prefix="car1_lateral_") as directory:
        world_file = Path(directory) / "world.sdf"
        world_tree.write(world_file, encoding="utf-8", xml_declaration=True)
        command = ["gz", "sim", "-s", "-r", str(world_file)]
        if args.visual:
            gui_file = Path(directory) / "gui.config"
            gui_text = Path("/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config").read_text()
            gui_text = gui_text.replace('show_again="true"', 'show_again="false"')
            gui_text = gui_text.replace("<engine>ogre2</engine>", "<engine>ogre</engine>")
            gui_text = gui_text.replace("<camera_pose>-6 0 6 0 0.5 0</camera_pose>", "<camera_pose>-1.7 -0.9 1.8 0 1.5707963 0</camera_pose>")
            gui_file.write_text(gui_text)
            command = ["gz", "sim", "-r", str(world_file), "--render-engine-gui", "ogre", "--gui-config", str(gui_file)]
        log_path = output_directory / ("lateral_body_frame.log" if args.body_frame else "lateral_current.log")
        with log_path.open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                wait_until(lambda: state["pose"] is not None and state["odom"] is not None and state["time"] is not None)
                drive(0, 0, 0, 2)
                checkpoint("start")
                for name, x, y, angular, duration in [
                    ("forward", 0.05, 0, 0, 4),
                    ("left", 0, 0.05, 0, 4),
                    ("right", 0, -0.05, 0, 4),
                    ("rotate", 0, 0, 0.2, 2),
                ]:
                    start = copy.deepcopy(state["pose"])
                    odom_start = copy.deepcopy(state["odom"])
                    drive(x, y, angular, duration)
                    drive(0, 0, 0, 0.3)
                    end = state["pose"]
                    heading = yaw(start)
                    dx, dy = end.position.x - start.position.x, end.position.y - start.position.y
                    angle = math.atan2(math.sin(yaw(end) - heading), math.cos(yaw(end) - heading))
                    result = {
                        "motion": name,
                        "body_x_m": round(math.cos(heading) * dx + math.sin(heading) * dy, 4),
                        "body_y_m": round(-math.sin(heading) * dx + math.cos(heading) * dy, 4),
                        "yaw_deg": round(math.degrees(angle), 2),
                        "odom_distance_m": round(math.hypot(state["odom"].position.x - odom_start.position.x, state["odom"].position.y - odom_start.position.y), 4),
                        "actual_distance_m": round(math.hypot(dx, dy), 4),
                    }
                    results.append(result)
                    print(json.dumps(result), flush=True)
                    checkpoint(name)
            finally:
                publisher.publish(stop)
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    output = output_directory / ("lateral_body_frame.json" if args.body_frame else "lateral_current.json")
    output.write_text(json.dumps(results, indent=2) + "\n")
    if args.capture_dir is not None:
        (args.capture_dir / "measurements.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
