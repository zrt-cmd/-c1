"""Full interaction system launch (speech + LLM)."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include speech launch
    speech_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('car1_speech'),
                'launch',
                'speech.launch.py'
            ])
        )
    )

    # Include LLM launch
    llm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('car1_llm'),
                'launch',
                'llm.launch.py'
            ])
        )
    )

    return LaunchDescription([
        speech_launch,
        llm_launch,
    ])
