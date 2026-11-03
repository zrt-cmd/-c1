#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据采集 Launch 文件

启动顺序：
  t=0s   apriltag_detector  — AprilTag 检测节点
  t=2s   data_collector     — 数据采集节点（等待 /visual_grasp/start 触发）
  t=4s   rqt_image_view     — 可视化（显示 AprilTag 检测调试图）

前置条件（需要已经在运行）：
  - bringup（base_driver + servo_controller + robot_state_publisher）
  - arm_controller
  - depth_camera（HP60C 驱动）

使用方式：
  ros2 launch eye_hand_calibration collect_data.launch.py
  # 然后触发采集：
  ros2 service call /eye_hand_calib/collect/start std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('eye_hand_calibration')

    # ---- 参数 ----
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='日志级别 (debug/info/warn/error)')
    start_rqt_arg = DeclareLaunchArgument(
        'start_rqt', default_value='true',
        description='是否启动 rqt 可视化')
    save_images_arg = DeclareLaunchArgument(
        'save_images', default_value='true',
        description='是否保存标定图像')
    data_dir_arg = DeclareLaunchArgument(
        'data_dir', default_value='',
        description='数据保存目录（留空使用默认 ~/.ros/eye_hand_calib/）')

    log_level = LaunchConfiguration('log_level')
    start_rqt = LaunchConfiguration('start_rqt')
    save_images = LaunchConfiguration('save_images')
    data_dir = LaunchConfiguration('data_dir')

    config_file = PathJoinSubstitution([pkg, 'config', 'apriltag_settings.yaml'])
    poses_file = PathJoinSubstitution([pkg, 'config', 'calibration_poses.yaml'])

    # ---- 节点 ----

    # AprilTag 检测节点（独立进程，避免 executor 冲突）
    apriltag_node = ExecuteProcess(
        cmd=['ros2', 'run', 'eye_hand_calibration', 'apriltag_detector',
             '--ros-args', '--log-level', log_level,
             '--params-file', config_file],
        output='screen',
    )

    # 数据采集节点（延迟 2s 等待检测器就绪，独立进程）
    # data_dir 为空时不传参数（让节点使用默认值）
    from launch.conditions import IfCondition, UnlessCondition
    from launch.substitutions import PythonExpression
    
    # 基础 cmd
    base_cmd = [
        'ros2', 'run', 'eye_hand_calibration', 'data_collector',
        '--ros-args', '--log-level', log_level,
        '-p', ['save_images:=', save_images],
    ]
    
    # data_dir 非空时追加
    # 注意：ExecuteProcess 不支持条件参数，统一传空字符串让节点处理
    data_collector_node = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=base_cmd,
                output='screen',
            )
        ]
    )

    # rqt 可视化（延迟 4s）
    rqt_node = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                     '/eye_hand_calib/apriltag/debug_image'],
                output='screen',
            )
        ]
    )

    # 启动提示
    start_msg = LogInfo(msg=(
        '\n'
        '============================================================\n'
        '  手眼标定数据采集已启动\n'
        '  \n'
        '  触发采集：\n'
        '    ros2 service call /eye_hand_calib/collect/start std_srvs/srv/Trigger\n'
        '  \n'
        '  或运行交互式工具：\n'
        '    ros2 run eye_hand_calibration collect_calibration_data\n'
        '============================================================\n'
    ))

    actions = [
        log_level_arg,
        start_rqt_arg,
        save_images_arg,
        data_dir_arg,
        start_msg,
        apriltag_node,
        data_collector_node,
        rqt_node,
    ]

    return LaunchDescription(actions)
