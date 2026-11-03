#!/usr/bin/env python3
"""Main chassis controller launch: IMU filter + odom + EKF + servo controller."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    chassis_pkg = get_package_share_directory('controller')
    servo_pkg = get_package_share_directory('car1_servo_controller')

    calibrate_config = os.path.join(chassis_pkg, 'config', 'calibrate_params.yaml')
    ekf_config = os.path.join(chassis_pkg, 'config', 'ekf.yaml')
    init_pose_config = os.path.join(chassis_pkg, 'config', 'init_pose.yaml')

    # IMU Complementary filter (参考 lander_pi，支持零偏估计)
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            # bias 在线估计启用，但使用极小的 alpha（0.005）让零偏估计时间常数变长（~200s）。
            # 配合 rf2o 激光里程计后，静止时 rf2o 提供零漂移参考，动态权重自动调整。
            'do_bias_estimation': True,
            'bias_alpha': 0.005,           # 极小 alpha，零偏估计非常慢
            'do_adaptive_gain': True,      # 启用自适应增益
        }],
        remappings=[
            ('imu/data_raw', 'car1_base_driver/imu_raw'),
            ('imu/data', 'imu'),
        ],
        output='screen'
    )

    # Odometry publisher
    odom_publisher_node = Node(
        package='controller',
        executable='odom_publisher',
        name='odom_publisher',
        parameters=[calibrate_config],
        output='screen'
    )

    # EKF node (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen'
    )

    # Servo controller
    servo_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(servo_pkg, 'launch', 'servo_controller.launch.py')
        )
    )

    # Init pose（默认 true 保持原有归位行为；调用方传 run_init_pose:=false 可跳过，机械臂不动）
    init_pose_node = Node(
        package='controller',
        executable='init_pose',
        name='init_pose',
        parameters=[init_pose_config],
        condition=IfCondition(LaunchConfiguration('run_init_pose')),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('run_init_pose', default_value='true',
                              description='是否执行 init_pose 归位（默认 true 保持原行为）'),
        imu_filter_node,
        odom_publisher_node,
        ekf_node,
        servo_controller_launch,
        init_pose_node,
    ])
