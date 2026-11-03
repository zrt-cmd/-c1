"""人脸关键点启动文件。

用法（板上）：
  ros2 launch car1_face_landmark face_landmark.launch.py
参数：
  config_path   配置文件（默认 share 内 face_config.yaml；display.show 控制弹窗）
前置：
  - 相机已起：ros2 launch ascamera hp60c.launch.py（话题严禁 remap）
  - rknnlite 已装（rknn-toolkit-lite2 2.3.2，wheel 见 course_examples/runtime/board/）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("car1_face_landmark"), "config", "face_config.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=default_config),

        Node(
            package="car1_face_landmark",
            executable="face_landmark",
            name="face_landmark",
            output="screen",
            arguments=[
                "--config", LaunchConfiguration("config_path"),
            ],
        ),
    ])
