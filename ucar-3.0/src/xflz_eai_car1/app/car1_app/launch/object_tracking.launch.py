#!/usr/bin/env python3
"""Launch object tracking application."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('car1_bringup')
    peripherals_pkg = get_package_share_directory('car1_peripherals')
    app_pkg = get_package_share_directory('car1_app')
    app_params = os.path.join(app_pkg, 'config', 'app_params.yaml')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'bringup.launch.py'))
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'depth_camera.launch.py'))
    )

    object_tracking_node = Node(
        package='car1_app',
        executable='object_tracking',
        name='object_tracking',
        parameters=[app_params],
        output='screen'
    )

    return LaunchDescription([
        bringup_launch,
        camera_launch,
        object_tracking_node,
    ])
