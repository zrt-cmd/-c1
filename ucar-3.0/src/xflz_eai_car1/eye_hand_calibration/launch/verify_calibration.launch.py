#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证标定结果 Launch 文件

启动验证工具和 AprilTag 检测，在新位置放置标定板后
自动计算并报告 3D 位置误差。

使用方式：
  ros2 launch eye_hand_calibration verify_calibration.launch.py
  # 在新位置放置 AprilTag，终端会实时输出误差
  # 查看报告: ros2 topic echo /eye_hand_calib/verify/report
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('eye_hand_calibration')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='日志级别')
    threshold_arg = DeclareLaunchArgument(
        'pass_threshold_mm', default_value='5.0',
        description='合格阈值（毫米），误差低于此值为合格')

    log_level = LaunchConfiguration('log_level')
    threshold = LaunchConfiguration('pass_threshold_mm')

    config_file = PathJoinSubstitution([pkg, 'config', 'apriltag_settings.yaml'])

    # AprilTag 检测
    apriltag_node = Node(
        package='eye_hand_calibration',
        executable='apriltag_detector',
        name='apriltag_detector',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[config_file],
    )

    # 验证节点（延迟 2s 等待检测器就绪）
    verifier_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='eye_hand_calibration',
                executable='calibration_verifier',
                name='calibration_verifier',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'pass_threshold_mm': threshold,
                    'publish_debug_image': True,
                }],
            )
        ]
    )

    info_msg = LogInfo(msg=(
        '\n'
        '============================================================\n'
        '  标定验证已启动\n'
        '  \n'
        '  请将 AprilTag 放在工作空间内任意位置\n'
        '  系统会自动计算 3D 位置误差并报告\n'
        '  \n'
        '  查看实时报告:\n'
        '    ros2 topic echo /eye_hand_calib/verify/report\n'
        '  \n'
        '  查看调试图像:\n'
        '    ros2 run rqt_image_view rqt_image_view /eye_hand_calib/verify/debug_image\n'
        '============================================================\n'
    ))

    return LaunchDescription([
        log_level_arg,
        threshold_arg,
        info_msg,
        apriltag_node,
        verifier_node,
    ])
