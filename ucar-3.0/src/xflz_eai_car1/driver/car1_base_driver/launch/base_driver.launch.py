import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('car1_base_driver'),
        'config', 'base_driver.yaml')

    return LaunchDescription([
        Node(
            package='car1_base_driver',
            executable='base_driver',
            name='car1_base_driver',
            output='screen',
            parameters=[config_file],
        ),
    ])
