#!/usr/bin/env python3
"""
启动基于手动参考角度的抓取演示
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('car1_gazebo').find('car1_gazebo')

    # 复用 grasp_demo.launch.py 的环境（Gazebo + 机器人 + 方块 + 控制器）
    # 但不启动 grasp_demo 节点
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'grasp_demo.launch.py')
        )
    )

    # 手动参考版本的抓取节点（延迟 5 秒启动，等待环境完全就绪）
    grasp_node = Node(
        package='car1_gazebo',
        executable='grasp_demo_manual_ref.py',
        name='grasp_demo_manual_ref',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        base_launch,
        # 延迟启动抓取节点
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=base_launch,
                on_exit=[grasp_node]
            )
        ),
    ])
