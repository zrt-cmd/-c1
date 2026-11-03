from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car1_kinematics',
            executable='fk_demo',
            name='car1_fk_demo',
            output='screen',
        ),
    ])
