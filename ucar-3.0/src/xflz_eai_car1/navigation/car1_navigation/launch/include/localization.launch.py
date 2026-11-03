#!/usr/bin/env python3
"""Localization launch: map_server + AMCL."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_pkg = get_package_share_directory('car1_navigation')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_arg = DeclareLaunchArgument('map', description='Full path to map yaml file')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    configured_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        },
        convert_types=True
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[configured_params],
        output='screen'
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[configured_params],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[configured_params],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        sim_time_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager,
    ])
