#!/usr/bin/env python3
# encoding: utf-8
"""Controller manager node: orchestrates servo control for car1 robot arm and gripper.

Design (rewritten):
  - joint_states cache defaults to the LAST COMMANDED position (updated on every
    command). This keeps TF / RViz / visual_grasp consistent with what we asked
    the arm to do, without hammering the serial bus with round-robin reads.
  - Actual servo readback is ON-DEMAND via the /servo_controller/refresh_positions
    service. Callers (e.g. visual_grasp before detection) call it at critical
    moments; the service returns success=False on timeout so callers can decide
    whether to proceed with the commanded value or abort.
  - The old 5Hz round-robin readback is removed. It caused mismatched responses
    (no ID validation), stale caches, and unreliable joint_states.
"""
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from car1_servo_controller_msgs.msg import ServosPosition, ServoPosition, ServoState, ServoStateList
from car1_base_driver_msgs.srv import GetBusServoState
from car1_base_driver_msgs.msg import GetBusServoCmd
from car1_servo_controller.servo_manager import ServoManager
from car1_servo_controller.joint_position_controller import JointPositionController
from car1_servo_controller.joint_trajectory_controller import JointTrajectoryActionController


class ControllerManagerNode(Node):
    def __init__(self):
        super().__init__(
            'servo_controller',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # Parse joint configurations from parameters
        self.joint_controllers = {}
        self.connected_ids = {}
        joints_param = self.get_parameters_by_prefix('joints')

        # Group parameters by joint name
        joint_configs = {}
        for key, param in joints_param.items():
            parts = key.split('.')
            joint_name = parts[0]
            field = parts[1] if len(parts) > 1 else key
            if joint_name not in joint_configs:
                joint_configs[joint_name] = {}
            joint_configs[joint_name][field] = param.value

        # Create joint position controllers
        for joint_name, config in joint_configs.items():
            servo_id = int(config['id'])
            init_pos = int(config['init'])
            min_pos = int(config['min'])
            max_pos = int(config['max'])
            orientation = int(config['orientation'])

            self.get_logger().info(f'Loading joint {joint_name}: id={servo_id}, orientation={orientation}')

            controller = JointPositionController(
                joint_name, servo_id, init_pos, min_pos, max_pos, orientation
            )
            self.joint_controllers[joint_name] = controller
            self.connected_ids[str(servo_id)] = joint_name

        # Reverse lookup: servo_id -> joint_name (for updating cache on command)
        self.id_to_joint = {ctrl.servo_id: jn for jn, ctrl in self.joint_controllers.items()}

        # Create servo manager
        self.servo_manager = ServoManager(self, self.connected_ids)

        # Create trajectory action controllers from config
        controllers_param = self.get_parameters_by_prefix('controllers')
        controller_configs = {}
        for key, param in controllers_param.items():
            parts = key.split('.')
            ctrl_name = parts[0]
            field = parts[1] if len(parts) > 1 else key
            if ctrl_name not in controller_configs:
                controller_configs[ctrl_name] = {}
            controller_configs[ctrl_name][field] = param.value

        self.trajectory_controllers = {}
        for ctrl_name, config in controller_configs.items():
            joint_names = config.get('joints', [])
            action_name = config.get('action_name', f'{ctrl_name}/follow_joint_trajectory')
            controllers = [self.joint_controllers[j] for j in joint_names]
            traj_ctrl = JointTrajectoryActionController(
                self, self.servo_manager, ctrl_name, action_name, controllers
            )
            self.trajectory_controllers[ctrl_name] = traj_ctrl

        # Publishers
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.servo_states_pub = self.create_publisher(ServoStateList, '~/servo_states', 10)

        # Subscribers
        self.create_subscription(
            ServosPosition, 'servo_controller', self.servo_controller_callback, 1
        )
        self.create_subscription(
            JointState, 'joint_controller', self.joint_controller_callback, 1
        )

        # Wait for base driver to be ready
        namespace = self.get_namespace()
        if namespace == '/':
            namespace = ''
        self.init_client = self.create_client(
            Trigger, namespace + '/car1_base_driver/init_finish'
        )
        self.get_logger().info('Waiting for car1_base_driver/init_finish...')
        self.init_client.wait_for_service()
        self.get_logger().info('car1_base_driver ready.')

        # Client for reading actual servo positions (used on-demand only).
        # Uses a ReentrantCallbackGroup so its response can be dispatched while
        # a service callback is waiting on the future. Without this, the single
        # executor thread would sit inside _refresh_positions_callback and never
        # deliver the get_state response — every read times out.
        self._readback_cbg = ReentrantCallbackGroup()
        self.get_state_client = self.create_client(
            GetBusServoState,
            namespace + '/car1_base_driver/bus_servo/get_state',
            callback_group=self._readback_cbg,
        )
        self.get_logger().info('Waiting for car1_base_driver/bus_servo/get_state...')
        self.get_state_client.wait_for_service()
        self.get_logger().info('get_state service ready.')

        # Cache: joint_name -> last known pulse position.
        # Initialised to configured init pose; updated by:
        #   1) command callbacks (immediately, source of truth by default)
        #   2) refresh_positions service (on-demand real readback)
        self.last_actual_positions = {}
        self._cache_lock = threading.Lock()
        for joint_name, controller in self.joint_controllers.items():
            self.last_actual_positions[joint_name] = controller.initial_position_raw

        # Configurable read timeout (per-servo, seconds)
        self.read_timeout = self.get_parameter_or(
            'read_timeout', rclpy.parameter.Parameter('read_timeout', value=1.5)
        ).value

        # Publish joint states via timer (from cache; no serial I/O in this path)
        publish_rate = self.get_parameter_or(
            'publish_rate', rclpy.parameter.Parameter('publish_rate', value=5.0)
        ).value
        self.publish_period = 1.0 / publish_rate
        self.publish_timer = self.create_timer(self.publish_period, self._publish_joint_states_tick)

        # Services
        self.create_service(Trigger, '~/init_finish', self._init_finish_callback)
        # On-demand actual readback: refreshes cache with real servo positions.
        # Returns success=True only if ALL servos were read within read_timeout.
        # On timeout the cache is partially updated (the servos that did respond)
        # and success=False so callers can decide their fallback policy.
        # Same ReentrantCallbackGroup as the client — the callback waits on the
        # client's future, so both must be reentrant.
        self.create_service(
            Trigger,
            '~/refresh_positions',
            self._refresh_positions_callback,
            callback_group=self._readback_cbg,
        )
        self.get_logger().info('\033[1;32mservo_controller started (on-demand readback)\033[0m')

    def _init_finish_callback(self, request, response):
        response.success = True
        return response

    # --- Cache update helpers ------------------------------------------------

    def _update_cache_from_command(self, servo_positions):
        """After sending a command, write the target pulses into the cache.

        This makes joint_states / TF reflect what we asked for, which is
        accurate enough for visual grasp (FK verified <1.5mm error). Real
        readback is only needed for safety-critical verification.
        """
        with self._cache_lock:
            for p in servo_positions:
                jn = self.id_to_joint.get(int(p.id))
                if jn is not None:
                    self.last_actual_positions[jn] = int(p.position)

    # --- Command callbacks ---------------------------------------------------

    def servo_controller_callback(self, msg):
        """Handle servo position commands (pulse, rad, or deg units)."""
        positions = self.servo_manager.get_position()
        valid_positions = []

        if msg.position_unit == 'pulse':
            for p in msg.position:
                if str(p.id) in positions:
                    valid_positions.append(p)
            self.servo_manager.set_position(msg.duration, valid_positions)

        elif msg.position_unit == 'rad':
            for p in msg.position:
                if str(p.id) in positions:
                    joint_name = positions[str(p.id)].name
                    pulse = self.joint_controllers[joint_name].pos_rad_to_pulse(p.position)
                    servo_pos = ServoPosition()
                    servo_pos.id = p.id
                    servo_pos.position = float(pulse)
                    valid_positions.append(servo_pos)
            self.servo_manager.set_position(msg.duration, valid_positions)

        elif msg.position_unit == 'deg':
            for p in msg.position:
                if str(p.id) in positions:
                    joint_name = positions[str(p.id)].name
                    pulse = self.joint_controllers[joint_name].pos_rad_to_pulse(
                        math.radians(p.position)
                    )
                    servo_pos = ServoPosition()
                    servo_pos.id = p.id
                    servo_pos.position = float(pulse)
                    valid_positions.append(servo_pos)
            self.servo_manager.set_position(msg.duration, valid_positions)

        # Sync cache with the values we just commanded.
        self._update_cache_from_command(valid_positions)

    def joint_controller_callback(self, msg):
        """Handle JointState commands (radians)."""
        commanded = []
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_controllers:
                controller = self.joint_controllers[name]
                pulse = controller.pos_rad_to_pulse(position)
                servo_pos = ServoPosition()
                servo_pos.id = controller.servo_id
                servo_pos.position = float(pulse)
                self.servo_manager.set_position(0.02, [servo_pos])
                commanded.append(servo_pos)
                time.sleep(0.005)
        self._update_cache_from_command(commanded)

        # 指令后读回：机械臂动过之后，等舵机到位（短暂延迟）后刷新真实位姿。
        # 让 joint_states 反映"实际到位值"而非"指令值"，供孪生/验证精确同步。
        # 用独立线程延迟，避免阻塞主回调；读回走 get_state 服务（复用 refresh 逻辑）。
        self._schedule_readback(delay=0.3)

    # --- 指令后真实位姿读回 ------------------------------------------------

    def _schedule_readback(self, delay=0.3):
        """延迟调度一次真实位姿读回（独立线程，不阻塞主回调）。"""
        try:
            t = threading.Timer(delay, self._readback_worker)
            t.daemon = True
            t.start()
        except Exception as e:
            self.get_logger().warn(f'调度读回失败: {e}')

    def _readback_worker(self):
        """在独立线程读回所有舵机真实位姿并更新缓存（复用 refresh 的读回逻辑）。"""
        try:
            request = Trigger.Request()
            response = self._refresh_positions_callback(request, Trigger.Response())
            if response.success:
                self.get_logger().debug(f'指令后读回 OK: {response.message}')
            else:
                self.get_logger().warn(f'指令后读回 PARTIAL: {response.message}')
        except Exception as e:
            self.get_logger().warn(f'指令后读回异常: {e}')

    # --- joint_states publisher ---------------------------------------------

    def _publish_joint_states_tick(self):
        """Publish cached positions. No serial I/O here — pure cache read."""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'

        servo_list_msg = ServoStateList()
        servo_list_msg.header = joint_msg.header

        with self._cache_lock:
            snapshot = dict(self.last_actual_positions)

        for joint_name, controller in self.joint_controllers.items():
            pulse = snapshot.get(joint_name, controller.initial_position_raw)

            joint_msg.name.append(joint_name)
            joint_msg.position.append(controller.pos_pulse_to_rad(pulse))

            servo_state = ServoState()
            servo_state.id = controller.servo_id
            servo_state.position = pulse
            servo_list_msg.servo_state.append(servo_state)

        self.joint_states_pub.publish(joint_msg)
        self.servo_states_pub.publish(servo_list_msg)

    # --- On-demand refresh service ------------------------------------------

    def _refresh_positions_callback(self, request, response):
        """Blocking read of all joints. Updates cache with real readback.

        Result:
          success=True  → every joint refreshed within read_timeout.
          success=False → at least one joint timed out or gave a bad reply;
                          message lists the failures. Callers can still use
                          the (partially updated) cache from joint_states if
                          they accept the risk.
        """
        deadline = time.time() + max(0.1, float(self.read_timeout) * len(self.joint_controllers))
        failed = []
        refreshed = []

        for joint_name, controller in self.joint_controllers.items():
            remaining = deadline - time.time()
            if remaining <= 0:
                failed.append(f'{joint_name}(overall timeout)')
                continue

            per_call_timeout = min(float(self.read_timeout), remaining)
            pulse = self._read_one_joint_blocking(controller.servo_id, per_call_timeout)

            if pulse is None:
                failed.append(f'{joint_name}(timeout)')
                continue
            if not (500 <= pulse <= 2500):
                failed.append(f'{joint_name}(bad pulse {pulse})')
                continue

            with self._cache_lock:
                self.last_actual_positions[joint_name] = pulse
            refreshed.append(f'{joint_name}={pulse}')

        response.success = (len(failed) == 0)
        if response.success:
            response.message = 'refreshed: ' + ', '.join(refreshed)
            self.get_logger().info(f'refresh_positions OK: {response.message}')
        else:
            response.message = 'failed: [' + ', '.join(failed) + '] refreshed: [' + ', '.join(refreshed) + ']'
            self.get_logger().warn(f'refresh_positions PARTIAL: {response.message}')
        return response

    def _read_one_joint_blocking(self, servo_id, timeout_sec):
        """Send a get_state request and wait up to timeout_sec for the reply.

        Verifies the reply ID matches the request (defense against out-of-order
        responses that plagued the old async round-robin path).

        Returns pulse (int) on success, None on timeout / bad reply.
        """
        if not self.get_state_client.service_is_ready():
            return None

        req = GetBusServoState.Request()
        cmd = GetBusServoCmd()
        cmd.id = servo_id
        cmd.get_position = 1
        req.cmd = [cmd]

        future = self.get_state_client.call_async(req)

        # Poll the future in a bounded loop. rclpy.spin_until_future_complete
        # would deadlock a single-threaded executor when this handler is itself
        # running on the executor, so we hand-roll a short poll.
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if future.done():
                break
            time.sleep(0.01)

        if not future.done():
            future.cancel()
            return None

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'read servo {servo_id} raised: {e}')
            return None

        if not (resp and resp.success and resp.state):
            return None

        state = resp.state[0]
        # Validate the response is for the servo we asked about. This is the
        # key check the old code was missing — without it, mismatched replies
        # got silently written into the wrong joint's cache.
        actual_id = state.present_id[0] if state.present_id else None
        if actual_id is not None and actual_id != servo_id:
            self.get_logger().warn(
                f'get_state ID mismatch: asked {servo_id}, got {actual_id}, dropping')
            return None

        if not state.position:
            return None
        return int(state.position[0])


def main(args=None):
    rclpy.init(args=args)
    node = ControllerManagerNode()
    # MultiThreadedExecutor lets the get_state response arrive on a different
    # thread while the refresh_positions callback is still waiting on its
    # future. Otherwise the single spin thread deadlocks on itself.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
