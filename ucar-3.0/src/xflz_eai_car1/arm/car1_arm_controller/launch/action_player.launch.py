import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    action_arg = DeclareLaunchArgument('action', default_value='home')
    auto_play_arg = DeclareLaunchArgument('auto_play', default_value='true')
    loop_arg = DeclareLaunchArgument('loop', default_value='false')
    step_mode_arg = DeclareLaunchArgument('step_mode', default_value='false')

    base_driver_pkg = get_package_share_directory('car1_base_driver')

    base_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_driver_pkg, 'launch/base_driver.launch.py'))
    )

    action_player_node = Node(
        package='car1_arm_controller',
        executable='action_player',
        name='action_player',
        output='screen',
        parameters=[{
            'action_file': LaunchConfiguration('action'),
            'auto_play': LaunchConfiguration('auto_play'),
            'loop': LaunchConfiguration('loop'),
            'step_mode': LaunchConfiguration('step_mode'),
        }]
    )

    return LaunchDescription([
        action_arg,
        auto_play_arg,
        loop_arg,
        step_mode_arg,
        base_driver_launch,
        action_player_node,
    ])
