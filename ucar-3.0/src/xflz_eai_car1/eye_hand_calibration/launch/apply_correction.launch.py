#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
深度校正节点 Launch 文件

启动实时深度校正节点，将原始深度图校正后重新发布。
校正参数从标定结果自动加载。

如果 visual_grasp 等节点需要使用校正后的深度图，
将其深度话题改为 /eye_hand_calib/depth/image_corrected 即可。

使用方式：
  ros2 launch eye_hand_calibration apply_correction.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('eye_hand_calibration')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='日志级别')
    config_path_arg = DeclareLaunchArgument(
        'config_path', default_value='',
        description='depth_correction.yaml 路径（留空自动查找）')

    log_level = LaunchConfiguration('log_level')
    config_path = LaunchConfiguration('config_path')

    depth_correction_config = PathJoinSubstitution(
        [pkg, 'config', 'depth_correction.yaml'])

    corrector_node = Node(
        package='eye_hand_calibration',
        executable='depth_corrector',
        name='depth_corrector',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            depth_correction_config,
            {'config_path': config_path},
        ],
    )

    info_msg = LogInfo(msg=(
        '\n'
        '============================================================\n'
        '  深度校正节点已启动\n'
        '  \n'
        '  原始深度: /ascamera_hp60c/camera_publisher/depth0/image_raw\n'
        '  校正深度: /eye_hand_calib/depth/image_corrected\n'
        '  \n'
        '  校正状态: ros2 topic echo /eye_hand_calib/depth/correction_active\n'
        '============================================================\n'
    ))

    return LaunchDescription([
        log_level_arg,
        config_path_arg,
        info_msg,
        corrector_node,
    ])
