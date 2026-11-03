#!/usr/bin/env python3
"""Map save utility node."""
import os
import subprocess
import rclpy
from rclpy.node import Node


class MapSave(Node):
    def __init__(self):
        super().__init__('map_save')
        self.declare_parameter('map_name', 'map_01')
        self.declare_parameter('map_dir', '')

        map_name = self.get_parameter('map_name').value
        map_dir = self.get_parameter('map_dir').value

        if not map_dir:
            from ament_index_python.packages import get_package_share_directory
            map_dir = os.path.join(get_package_share_directory('car1_slam'), 'maps')

        os.makedirs(map_dir, exist_ok=True)
        map_path = os.path.join(map_dir, map_name)

        self.get_logger().info(f'Saving map to: {map_path}')
        try:
            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path,
                '--ros-args', '-p', 'map_subscribe_transient_local:=true'
            ], check=True)
            self.get_logger().info('Map saved successfully!')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to save map: {e}')

        raise SystemExit(0)


def main():
    rclpy.init()
    node = MapSave()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
