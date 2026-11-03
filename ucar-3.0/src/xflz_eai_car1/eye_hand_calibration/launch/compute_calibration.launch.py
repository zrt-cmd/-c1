#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定计算 Launch 文件

运行离线标定计算（不需要硬件在线）：
  - 读取最新会话数据
  - 求解手眼变换矩阵（AX=XB）
  - 拟合深度校正模型
  - 生成结果文件和评估报告

使用方式：
  ros2 launch eye_hand_calibration compute_calibration.launch.py
  ros2 launch eye_hand_calibration compute_calibration.launch.py session:=/path/to/session
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # 支持两个同义参数：session（旧）和 data_dir（新）
    # 推荐使用 data_dir（与采集脚本的输出目录一致）
    session_arg = DeclareLaunchArgument(
        'session', default_value='',
        description='标定会话目录（与 data_dir 等价，留空自动查找最新）')
    data_dir_arg = DeclareLaunchArgument(
        'data_dir', default_value='',
        description='标定会话目录（与 session 等价，留空自动查找最新）')
    algorithm_arg = DeclareLaunchArgument(
        'algorithm', default_value='horaud',
        description='求解算法: tsai 或 horaud')
    pair_mode_arg = DeclareLaunchArgument(
        'pair_mode', default_value='first-ref',
        description='相对变换模式: consecutive, all, first-ref(默认)')
    min_rotation_arg = DeclareLaunchArgument(
        'min_rotation', default_value='8',
        description='最小旋转角阈值(度), 默认8')
    session = LaunchConfiguration('session')
    data_dir = LaunchConfiguration('data_dir')
    algorithm = LaunchConfiguration('algorithm')
    pair_mode = LaunchConfiguration('pair_mode')
    min_rotation = LaunchConfiguration('min_rotation')

    # 使用 ros2 run 调用注册的 console_script（避免模块路径问题）
    # 优先使用 data_dir，为空则用 session，两者都为空则自动查找最新会话
    from launch.substitutions import PythonExpression
    session_val = PythonExpression([
        '"', data_dir, '" if "', data_dir, '" else "', session, '"'
    ])

    compute_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'eye_hand_calibration', 'compute_hand_eye_transform',
            '--algorithm', algorithm,
            '--pair-mode', pair_mode,
            '--min-rotation', min_rotation,
            '--auto-outlier',
            '--session', session_val,
        ],
        output='screen',
    )

    info_msg = LogInfo(msg=(
        '\n'
        '============================================================\n'
        '  手眼标定计算已启动\n'
        '  完成后结果保存在会话目录的 results/ 子目录\n'
        '  \n'
        '  下一步：更新 URDF\n'
        '    ros2 run eye_hand_calibration generate_corrected_urdf\n'
        '============================================================\n'
    ))

    return LaunchDescription([
        session_arg,
        data_dir_arg,
        algorithm_arg,
        pair_mode_arg,
        min_rotation_arg,
        info_msg,
        compute_process,
    ])
