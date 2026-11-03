#!/usr/bin/env python3
"""Launch COIN-D6 lidar (m1ct_d2 driver)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_node = Node(
        package='m1ct_d2',
        executable='m1ct_d2',
        name='m1ct_d2_node',
        parameters=[{
            'port': '/dev/car1_lidar',
            'frame_id': 'laser_link',
            'ignore_array': '',
            'baudrate': 230400,
            'samp_rate': 9,
            'resolution_fixed': True,
            'singleChannel': False,
            'auto_reconnect': True,
            'reversion': True,
            'isToFLidar': False,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'max_range': 12.0,
            'min_range': 0.05,
            'frequency': 10.0,
            'version': 4,
        }],
        output='screen'
    )

    return LaunchDescription([lidar_node])
