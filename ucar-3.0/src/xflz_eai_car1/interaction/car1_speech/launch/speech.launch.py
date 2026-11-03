"""Launch all speech nodes (wake word, ASR, TTS)."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_dir = PathJoinSubstitution([
        FindPackageShare('car1_speech'), 'config'
    ])

    wake_word_config = PathJoinSubstitution([config_dir, 'wake_word.yaml'])
    tts_config = PathJoinSubstitution([config_dir, 'tts.yaml'])

    # Nodes
    # 注意：真实 ASR（唤醒后识别命令）在 wake_word_node 内部实现；
    # 旧的独立 asr_node 是 stub 假识别，已从 launch 移除，避免污染 /interaction/asr/text。
    wake_word_node = Node(
        package='car1_speech',
        executable='wake_word_node',
        name='wake_word_node',
        output='screen',
        parameters=[wake_word_config],
    )

    tts_node = Node(
        package='car1_speech',
        executable='tts_node',
        name='tts_node',
        output='screen',
        parameters=[tts_config],
    )

    return LaunchDescription([
        wake_word_node,
        tts_node,
    ])
