#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bus servo zero-position calibration tool.

Safety-first design:
- Does NOT auto-move on startup
- User must explicitly call move_to_center service
- Reads actual servo positions from STM32 when saving
- Falls back to tracked positions if STM32 read fails

Usage:
  ros2 launch calibration servo_calib.launch.py
  ros2 service call /calibrate_servo/move_to_center std_srvs/srv/Trigger
  # Fine-tune via topic pub
  ros2 service call /calibrate_servo/save_offsets std_srvs/srv/Trigger
"""
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from car1_base_driver_msgs.msg import ServosPosition, ServoPosition
from car1_base_driver_msgs.srv import GetBusServoState
from car1_base_driver_msgs.msg import GetBusServoCmd


SERVO_IDS = [0, 1, 2, 3, 4, 5]
SERVO_CENTER = 1500
MOVE_DURATION = 3.0


class CalibrateServo(Node):
    def __init__(self):
        super().__init__('calibrate_servo')

        self.declare_parameter('servo_ids', SERVO_IDS)
        self.declare_parameter('center_position', SERVO_CENTER)
        self.declare_parameter('move_duration', MOVE_DURATION)
        self.servo_ids = self.get_parameter('servo_ids').value
        self.center_position = self.get_parameter('center_position').value
        self.move_duration = self.get_parameter('move_duration').value

        # Track positions from user commands (fallback)
        self.servo_positions = {sid: self.center_position for sid in self.servo_ids}

        self.position_pub = self.create_publisher(
            ServosPosition, '/car1_base_driver/bus_servo/set_position', 10)

        # Subscribe to track user adjustments
        self.create_subscription(
            ServosPosition, '/car1_base_driver/bus_servo/set_position',
            self._position_cb, 10)

        # Service client for reading servo state from STM32
        self.get_state_client = self.create_client(
            GetBusServoState, '/car1_base_driver/bus_servo/get_state')

        self.create_service(Trigger, '~/move_to_center', self._move_to_center_cb)
        self.create_service(Trigger, '~/save_offsets', self._save_offsets_cb)

        self.get_logger().info(
            f'\033[1;32m=== Servo Calibration Tool ===\033[0m')
        self.get_logger().info(
            f'Servo IDs: {self.servo_ids}, Center: {self.center_position}, '
            f'Duration: {self.move_duration}s')
        self.get_logger().info('')
        self.get_logger().info('\033[1;33mWill NOT auto-move. Steps:\033[0m')
        self.get_logger().info('')
        self.get_logger().info('Step 1 - Move to center (one by one, slowly):')
        self.get_logger().info('  ros2 service call /calibrate_servo/move_to_center std_srvs/srv/Trigger')
        self.get_logger().info('')
        self.get_logger().info('Step 2 - Fine-tune individual servos:')
        self.get_logger().info('  ros2 topic pub /car1_base_driver/bus_servo/set_position \\')
        self.get_logger().info('    car1_base_driver_msgs/msg/ServosPosition \\')
        self.get_logger().info('    "{duration: 2.0, position: [{id: 1, position: 1450}]}" --once')
        self.get_logger().info('')
        self.get_logger().info('Step 3 - Save offsets:')
        self.get_logger().info('  ros2 service call /calibrate_servo/save_offsets std_srvs/srv/Trigger')

    def _position_cb(self, msg):
        """Track position commands to know where each servo was sent."""
        for pos in msg.position:
            if pos.id in self.servo_positions:
                self.servo_positions[pos.id] = pos.position

    def _move_to_center_cb(self, request, response):
        """Move servos to center one at a time, slowly."""
        if self.position_pub.get_subscription_count() == 0:
            response.success = False
            response.message = 'No subscriber on bus_servo/set_position. Is base_driver running?'
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(
            f'\033[1;33mMoving servos to center ({self.center_position}) '
            f'one at a time, {self.move_duration}s each...\033[0m')

        for sid in self.servo_ids:
            self.get_logger().info(f'  Moving servo {sid} to {self.center_position}...')
            msg = ServosPosition()
            msg.duration = self.move_duration
            pos = ServoPosition()
            pos.id = sid
            pos.position = self.center_position
            msg.position.append(pos)
            self.position_pub.publish(msg)
            time.sleep(self.move_duration + 0.5)

        response.success = True
        response.message = f'All servos moved to center ({self.center_position})'
        self.get_logger().info(f'\033[1;32m{response.message}\033[0m')
        return response

    def _read_positions_from_stm32(self):
        """Try to read actual servo positions from STM32."""
        if not self.get_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('GetBusServoState service not available, using tracked positions')
            return None

        positions = {}
        for sid in self.servo_ids:
            req = GetBusServoState.Request()
            cmd = GetBusServoCmd()
            cmd.id = sid
            cmd.get_position = 1
            req.cmd.append(cmd)

            future = self.get_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is None:
                self.get_logger().warn(f'Failed to read servo {sid} position')
                return None

            result = future.result()
            if result.state and result.state[0].position:
                positions[sid] = result.state[0].position[0]
            else:
                self.get_logger().warn(f'No position data for servo {sid}')
                return None

        return positions

    def _save_offsets_cb(self, request, response):
        """Read servo positions and save offsets to YAML config."""
        # Try reading from STM32 first
        positions = self._read_positions_from_stm32()

        if positions is None:
            # Fallback: use tracked positions from topic subscription
            self.get_logger().warn('Using tracked positions (from topic commands)')
            positions = dict(self.servo_positions)

        offsets = {}
        info_lines = []
        for sid in self.servo_ids:
            current_pos = positions.get(sid, self.center_position)
            offset = current_pos - self.center_position
            offsets[sid] = offset
            info_lines.append(f'  Servo {sid}: position={current_pos}, offset={offset}')

        # Save to YAML
        src_config = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'servo_offsets.yaml')
        try:
            install_config = os.path.join(
                get_package_share_directory('calibration'), 'config', 'servo_offsets.yaml')
        except Exception:
            install_config = None

        config_data = {'servo_offsets': {f'servo_{sid}': offsets[sid] for sid in self.servo_ids}}

        saved_paths = []
        for path in [src_config, install_config]:
            if path is None:
                continue
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'w') as f:
                    yaml.dump(config_data, f, default_flow_style=False)
                saved_paths.append(path)
                self.get_logger().info(f'Saved to: {path}')
            except Exception as e:
                self.get_logger().warn(f'Could not save to {path}: {e}')

        msg = 'Offsets saved:\n' + '\n'.join(info_lines)
        self.get_logger().info(f'\033[1;32m{msg}\033[0m')

        response.success = True
        response.message = f'Offsets saved: {offsets}'
        return response


def main():
    rclpy.init()
    node = CalibrateServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
