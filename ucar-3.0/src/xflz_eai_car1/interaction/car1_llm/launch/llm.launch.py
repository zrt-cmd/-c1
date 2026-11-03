"""Launch LLM chat and skill executor nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config paths
    config_dir = PathJoinSubstitution([
        FindPackageShare('car1_llm'), 'config'
    ])

    llm_config = PathJoinSubstitution([config_dir, 'llm.yaml'])
    skills_config = PathJoinSubstitution([config_dir, 'skills.yaml'])

    # Chat node
    chat_node = Node(
        package='car1_llm',
        executable='chat_node',
        name='chat_node',
        output='screen',
        parameters=[llm_config],
    )

    # Skill executor node
    skill_executor = Node(
        package='car1_llm',
        executable='skill_executor',
        name='skill_executor',
        output='screen',
        parameters=[{'skills_config': skills_config}],
    )

    return LaunchDescription([
        chat_node,
        skill_executor,
    ])
