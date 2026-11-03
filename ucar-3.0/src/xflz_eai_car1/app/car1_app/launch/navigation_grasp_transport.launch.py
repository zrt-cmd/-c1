#!/usr/bin/env python3
"""All-in-one launch for Level 5: navigation + vision + grasp + transport.

Composes:
  1. car1_navigation navigation.launch.py  (bringup + lidar + AMCL + Nav2)
  2. car1_arm_controller arm_controller.launch.py (arm services)
  3. car1_peripherals depth_camera.launch.py (HP60C RGB-D)
  4. car1_app rsp_only.launch.py (URDF TF - camera in eye-in-hand pose needs
     live TF from arm joints; arm_controller.launch.py starts its own RSP so
     we do NOT start rsp_only here to avoid a duplicate).
  5. navigation_grasp_transport node (this state machine)

Timing:
  bringup + navigation start at t=0 (nav takes longest to lifecycle-activate)
  arm_controller at  t+8s   (bringup must have serial + servo_controller up)
  camera at          t+10s
  main node at       t+15s  (Nav2 lifecycle usually ready by then; the node
                             also waits internally for its own service clients)

Trigger the task from a second terminal:
    ros2 service call /navigation_grasp_transport/start std_srvs/srv/Trigger
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    app_pkg = get_package_share_directory('car1_app')
    nav_pkg = get_package_share_directory('car1_navigation')
    arm_pkg = get_package_share_directory('car1_arm_controller')
    peripherals_pkg = get_package_share_directory('car1_peripherals')

    default_params = os.path.join(app_pkg, 'config',
                                  'navigation_grasp_transport.yaml')

    args = [
        DeclareLaunchArgument('map', default_value='slam_map',
                              description='Map name (without .yaml)'),
        DeclareLaunchArgument('start_navigation', default_value='true',
                              description='Start navigation stack (bringup+lidar+AMCL+Nav2)'),
        DeclareLaunchArgument('start_arm', default_value='true',
                              description='Start arm_controller'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='Start depth camera driver'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz (navigation view)'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='navigation_grasp_transport params yaml'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='Skip arm pick (detect + plane project only)'),
    ]

    # Navigation stack (also includes bringup + lidar + robot_state_publisher).
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_navigation')),
    )

    arm_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(arm_pkg, 'launch', 'arm_controller.launch.py')),
                # Nav launch already brought bringup up; do NOT let arm launch
                # bring up a second base_driver on /dev/car1_stm32.
                launch_arguments={'start_bringup': 'false'}.items(),
                condition=IfCondition(LaunchConfiguration('start_arm')),
            )
        ],
    )

    camera_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(peripherals_pkg, 'launch', 'depth_camera.launch.py')),
                condition=IfCondition(LaunchConfiguration('start_camera')),
            )
        ],
    )

    main_node = Node(
        package='car1_app',
        executable='navigation_grasp_transport',
        name='navigation_grasp_transport',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'dry_run': LaunchConfiguration('dry_run')},
        ],
    )

    main_delayed = TimerAction(period=15.0, actions=[main_node])

    return LaunchDescription(args + [
        navigation_launch,
        arm_launch,
        camera_launch,
        main_delayed,
    ])
