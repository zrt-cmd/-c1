import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('car1_arm_controller')
    bringup_pkg = get_package_share_directory('car1_bringup')

    arm_params = os.path.join(pkg_dir, 'config', 'arm_params.yaml')
    preset_poses = os.path.join(pkg_dir, 'config', 'preset_poses.yaml')

    # Launch argument: start_bringup (default: false, user can override with start_bringup:=true)
    start_bringup_arg = DeclareLaunchArgument(
        'start_bringup',
        default_value='false',
        description='Whether to start bringup.launch.py (set to true if not already running)'
    )

    # Conditionally include bringup only if start_bringup:=true
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch/bringup.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_bringup'))
    )

    arm_controller_node = Node(
        package='car1_arm_controller',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[arm_params, preset_poses],
    )

    return LaunchDescription([
        start_bringup_arg,
        bringup_launch,
        arm_controller_node,
    ])
