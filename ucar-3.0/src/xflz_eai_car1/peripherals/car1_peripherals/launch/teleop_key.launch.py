#!/usr/bin/env python3
"""Launch keyboard teleop for car1 (mecanum)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='car1_peripherals',
        executable='teleop_key_control',
        name='teleop_key_control',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([teleop_node])
