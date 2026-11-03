"""姿态启动文件。

用法（板上）：
  ros2 launch car1_pose_landmark pose_landmark.launch.py
参数：
  config_path   配置文件（默认 share 内 pose_config.yaml；display.show 控制弹窗）
前置：
  - 相机已起：ros2 launch ascamera hp60c.launch.py（话题严禁 remap）
  - rknnlite 已装（rknn-toolkit-lite2 2.3.2）+ pose_landmark_full.rknn 已入库
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("car1_pose_landmark"), "config", "pose_config.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=default_config),

        Node(
            package="car1_pose_landmark",
            executable="pose_landmark",
            name="pose_landmark",
            output="screen",
            arguments=[
                "--config", LaunchConfiguration("config_path"),
            ],
        ),
    ])
