#!/usr/bin/env python3
"""SLAM launch: bringup + lidar + slam_toolbox.

Single entry point. Layered include of bringup (driver + chassis controller +
robot_state_publisher + joint_state_publisher), then lidar, and finally
slam_toolbox delayed until odom/TF/scan have stabilized.

Note: robot_state_publisher and joint_state_publisher now live in bringup
(EKF needs base_link -> imu_link TF), so they are NOT started here to avoid
duplicate publishers fighting over /robot_description and /tf_static.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('car1_bringup')
    peripherals_pkg = get_package_share_directory('car1_peripherals')
    slam_pkg = get_package_share_directory('car1_slam')

    slam_config = os.path.join(slam_pkg, 'config', 'slam.yaml')
    rviz_config = os.path.join(slam_pkg, 'rviz', 'slam.rviz')

    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    # 1. Bringup (driver + robot_state_publisher + joint_state_publisher + chassis controller).
    #    robot_state_publisher (URDF static TF) and joint_state_publisher are provided
    #    by bringup so EKF gets base_link -> imu_link TF.
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'bringup.launch.py'))
    )

    # 2. Lidar (publishes /scan in frame laser_link)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'lidar.launch.py'))
    )

    # 3. slam_toolbox, delayed 10s to let odom / TF / scan stabilize.
    slam_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_config],
                output='screen'
            )
        ]
    )

    # 4. RViz2 (optional, toggle with use_rviz:=false)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        use_rviz_arg,
        bringup_launch,
        lidar_launch,
        slam_node,
        rviz_node,
    ])
