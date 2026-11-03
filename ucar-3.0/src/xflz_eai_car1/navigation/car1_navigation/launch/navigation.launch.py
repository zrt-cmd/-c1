#!/usr/bin/env python3
"""Main navigation launch: bringup + lidar + localization + navigation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bringup_pkg = get_package_share_directory('car1_bringup')
    peripherals_pkg = get_package_share_directory('car1_peripherals')
    nav_pkg = get_package_search_directory('car1_navigation') if False else get_package_share_directory('car1_navigation')

    # 直接使用源码目录下的地图路径（不依赖 install 目录）
    maps_dir = os.path.expanduser('~/ros2_ws/src/xflz_eai_car1/slam/car1_slam/maps')

    # 在运行时求值 LaunchConfiguration
    # 优先使用 map_file(绝对路径),否则用 map(相对路径)
    map_file_param = LaunchConfiguration('map_file').perform(context)
    if map_file_param:
        # 如果没有 .yaml 后缀，自动补上
        map_file = map_file_param if map_file_param.endswith('.yaml') else map_file_param + '.yaml'
    else:
        map_name = LaunchConfiguration('map').perform(context)
        map_file = os.path.join(maps_dir, map_name + '.yaml')

    # 使用源码目录的 RViz 配置
    rviz_config = os.path.expanduser('~/ros2_ws/src/xflz_eai_car1/navigation/car1_navigation/rviz/navigation.rviz')

    # 1. Bringup
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'bringup.launch.py'))
    )

    # 2. Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'lidar.launch.py'))
    )

    # 3. Localization（延迟到 base_driver + 雷达就绪后再启动）
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'include', 'localization.launch.py')),
        launch_arguments={'map': map_file}.items()
    )

    # 4. Navigation（延迟到 amcl 就绪后再启动）
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'include', 'navigation_base.launch.py'))
    )

    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    # --- 分层启动：条件就绪检测（非固定延时） ---
    # localization: 等 base_driver 的 /odom_raw 话题出现（底盘驱动就绪）
    # navigation:   等 /map 话题出现（map_server 已激活并发布地图）
    # 超时兜底：即使条件未满足也继续启动（exit 0），避免卡死

    def _wait_ready(check_cmd, timeout, ready_msg, next_actions):
        """ExecuteProcess 检测就绪条件 → OnProcessExit 触发下一层启动。"""
        full_cmd = (
            f'echo "[nav_chain] 等待: {ready_msg}"; '
            f'timeout {timeout} bash -c "{check_cmd}" > /dev/null 2>&1; '
            f'exit 0'
        )
        wait_proc = ExecuteProcess(
            cmd=['bash', '-c', full_cmd],
            output='screen',
        )
        handler = RegisterEventHandler(OnProcessExit(
            target_action=wait_proc,
            on_exit=lambda event, context: [
                LogInfo(msg=f'[nav_chain] {ready_msg}'),
            ] + next_actions,
        ))
        return [wait_proc, handler]

    localization_chain = _wait_ready(
        check_cmd='while ! ros2 topic echo --once /odom_raw --field header 2>/dev/null | grep -q stamp; do sleep 0.5; done',
        timeout=20,
        ready_msg='base_driver 就绪（/odom_raw 有数据）→ 启动 localization',
        next_actions=[localization_launch],
    )

    navigation_chain = _wait_ready(
        check_cmd='while ! ros2 topic echo --once /map --field info 2>/dev/null | grep -q resolution; do sleep 0.5; done',
        timeout=30,
        ready_msg='map_server 就绪（/map 有数据）→ 启动 navigation',
        next_actions=[navigation_launch],
    )

    return [
        bringup_launch,
        lidar_launch,
        *localization_chain,
        *navigation_chain,
        rviz_node,
    ]


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map', default_value='slam_map',
        description='Map name (without extension) under car1_slam/maps/')

    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='',
        description='Absolute path to map YAML file (overrides map parameter)')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz')

    return LaunchDescription([
        map_arg,
        map_file_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])
