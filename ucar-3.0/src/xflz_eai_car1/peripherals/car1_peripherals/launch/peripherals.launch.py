#!/usr/bin/env python3
"""Launch all peripherals: lidar + depth camera."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('car1_peripherals')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'lidar.launch.py'))
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'depth_camera.launch.py'))
    )

    return LaunchDescription([
        lidar_launch,
        depth_camera_launch,
    ])
