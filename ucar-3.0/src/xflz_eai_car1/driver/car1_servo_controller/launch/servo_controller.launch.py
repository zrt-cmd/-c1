#!/usr/bin/env python3
"""Launch file for car1_servo_controller."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('car1_servo_controller'),
        'config',
        'servo_controller.yaml'
    )

    servo_controller_node = Node(
        package='car1_servo_controller',
        executable='servo_controller',
        name='servo_controller',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        servo_controller_node,
    ])
