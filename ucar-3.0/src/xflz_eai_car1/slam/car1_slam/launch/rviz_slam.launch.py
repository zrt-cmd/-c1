#!/usr/bin/env python3
"""Launch RViz2 for SLAM visualization."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam_pkg = get_package_share_directory('car1_slam')
    rviz_config = os.path.join(slam_pkg, 'rviz', 'slam.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([rviz_node])
