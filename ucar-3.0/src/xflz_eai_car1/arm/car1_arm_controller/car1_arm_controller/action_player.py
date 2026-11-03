#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YAML action player for car1 arm.

Loads a YAML action file and plays back the servo positions sequentially.
Supports stop/pause via ROS2 service.

Usage:
  ros2 run car1_arm_controller action_player --ros-args -p action_file:=home.yaml
  ros2 service call /action_player/stop std_srvs/srv/Trigger
"""
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from car1_base_driver_msgs.msg import ServosPosition, ServoPosition


class ActionPlayer(Node):
    def __init__(self):
        super().__init__('action_player')

        self.declare_parameter('action_file', '')
        self.declare_parameter('actions_dir', '')
        self.declare_parameter('auto_play', True)
        self.declare_parameter('loop', False)
        self.declare_parameter('step_mode', False)

        self.stop_flag = False
        self.playing = False
        self.step_event = False  # Used in step mode to advance

        self.position_pub = self.create_publisher(
            ServosPosition, '/car1_base_driver/bus_servo/set_position', 10)

        self.create_service(Trigger, '~/play', self._play_cb)
        self.create_service(Trigger, '~/stop', self._stop_cb)
        self.create_service(Trigger, '~/next', self._next_cb)

        # Resolve actions directory
        actions_dir = self.get_parameter('actions_dir').value
        if not actions_dir:
            try:
                pkg_share = get_package_share_directory('car1_arm_controller')
                actions_dir = os.path.join(pkg_share, 'config', 'actions')
            except Exception:
                actions_dir = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'config', 'actions')
        self.actions_dir = actions_dir

        # Load action file
        action_file = self.get_parameter('action_file').value
        self.action_data = None
        if action_file:
            self.action_data = self._load_action(action_file)

        if self.action_data:
            self.get_logger().info(
                f'\033[1;32mLoaded action: {self.action_data["name"]} '
                f'({len(self.action_data["actions"])} steps)\033[0m')
            self.get_logger().info(f'  Description: {self.action_data.get("description", "")}')
            for i, step in enumerate(self.action_data['actions']):
                self.get_logger().info(
                    f'  Step {i+1}: {step["name"]} (duration={step["duration"]}s, '
                    f'servos={list(step["servos"].keys())})')
        else:
            self.get_logger().warn('No action file loaded. Use ~/play service or set action_file param.')

        self.get_logger().info('')
        self.get_logger().info('Services:')
        self.get_logger().info('  ~/play  - Start playback')
        self.get_logger().info('  ~/stop  - Stop playback')
        self.get_logger().info('  ~/next  - Advance to next step (step_mode only)')

        # Auto play on startup if configured
        if self.get_parameter('auto_play').value and self.action_data:
            self.create_timer(2.0, self._auto_play)

    def _auto_play(self):
        # Cancel timer after first call
        for timer in self.timers:
            timer.cancel()
        self._play_action()

    def _load_action(self, action_file):
        """Load a YAML action file."""
        # Try absolute path first
        if os.path.isabs(action_file) and os.path.exists(action_file):
            path = action_file
        else:
            # Try in actions directory
            if not action_file.endswith('.yaml'):
                action_file += '.yaml'
            path = os.path.join(self.actions_dir, action_file)

        if not os.path.exists(path):
            self.get_logger().error(f'Action file not found: {path}')
            return None

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            if 'actions' not in data:
                self.get_logger().error(f'Invalid action file: missing "actions" key')
                return None

            # Normalize servo keys to int
            for step in data['actions']:
                step['servos'] = {int(k): int(v) for k, v in step['servos'].items()}

            return data
        except Exception as e:
            self.get_logger().error(f'Failed to load action file: {e}')
            return None

    def _play_action(self):
        """Play the loaded action sequence."""
        if not self.action_data:
            self.get_logger().error('No action loaded')
            return False

        if self.playing:
            self.get_logger().warn('Already playing')
            return False

        self.playing = True
        self.stop_flag = False
        self.step_event = False

        loop = self.get_parameter('loop').value
        step_mode = self.get_parameter('step_mode').value
        action_name = self.action_data['name']

        try:
            while True:
                self.get_logger().info(f'\033[1;33mPlaying: {action_name}\033[0m')
                if step_mode:
                    self.get_logger().info('\033[1;36m[STEP MODE] Call ~/next to advance\033[0m')

                for i, step in enumerate(self.action_data['actions']):
                    if self.stop_flag:
                        self.get_logger().info('\033[1;31mStopped by user\033[0m')
                        return False

                    step_name = step['name']
                    duration = float(step['duration'])
                    servos = step['servos']

                    self.get_logger().info(
                        f'  [{i+1}/{len(self.action_data["actions"])}] {step_name} '
                        f'(duration={duration}s)')

                    # Build and publish message
                    msg = ServosPosition()
                    msg.duration = duration
                    for servo_id, pwm in servos.items():
                        pos = ServoPosition()
                        pos.id = int(servo_id)
                        pos.position = int(pwm)
                        msg.position.append(pos)
                    self.position_pub.publish(msg)

                    # Wait for movement to complete
                    time.sleep(duration + 0.3)

                    # In step mode, wait for ~/next before continuing
                    if step_mode and i < len(self.action_data['actions']) - 1:
                        self.get_logger().info(
                            f'\033[1;36m  Waiting... call ~/next to continue '
                            f'(next: {self.action_data["actions"][i+1]["name"]})\033[0m')
                        self.step_event = False
                        while not self.step_event and not self.stop_flag:
                            time.sleep(0.1)
                        if self.stop_flag:
                            self.get_logger().info('\033[1;31mStopped by user\033[0m')
                            return False

                self.get_logger().info(f'\033[1;32mDone: {action_name}\033[0m')

                if not loop:
                    break
                self.get_logger().info('Looping...')

        finally:
            self.playing = False

        return True

    def _play_cb(self, request, response):
        """Service callback: start playback."""
        if self._play_action():
            response.success = True
            response.message = 'Playback completed'
        else:
            response.success = False
            response.message = 'Playback failed or stopped'
        return response

    def _stop_cb(self, request, response):
        """Service callback: stop playback."""
        self.stop_flag = True
        response.success = True
        response.message = 'Stop requested'
        self.get_logger().info('\033[1;31mStop requested\033[0m')
        return response

    def _next_cb(self, request, response):
        """Service callback: advance to next step (step_mode only)."""
        if not self.playing:
            response.success = False
            response.message = 'Not playing'
        elif not self.get_parameter('step_mode').value:
            response.success = False
            response.message = 'Not in step mode'
        else:
            self.step_event = True
            response.success = True
            response.message = 'Advancing to next step'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ActionPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
