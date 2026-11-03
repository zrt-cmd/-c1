#!/usr/bin/env python3
"""Minimal launch: robot_state_publisher only (for TF verification)."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('car1_description'),
        'urdf', 'car1.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ])
