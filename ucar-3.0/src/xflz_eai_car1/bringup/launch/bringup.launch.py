import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_driver_pkg = get_package_share_directory('car1_base_driver')
    chassis_pkg = get_package_share_directory('controller')
    description_pkg = get_package_share_directory('car1_description')

    # URDF (single source of static TF: base_footprint -> base_link -> imu_link / laser_link / arm joints).
    # EKF needs base_link -> imu_link to transform IMU angular velocity into base frame,
    # so robot_state_publisher MUST run as part of bringup (not only in slam.launch).
    urdf_file = os.path.join(description_pkg, 'urdf', 'car1.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    bringup_description = LaunchDescription([
        # 1. Board-level driver (serial communication with STM32)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(base_driver_pkg, 'launch/base_driver.launch.py')),
        ),
        # 2. Robot state publisher — URDF static TF (base_link -> imu_link etc.).
        #    Started before/with the chassis controller so EKF can look up the
        #    base_link -> imu_link transform. robot_localization retries TF lookups,
        #    so a small startup-order gap is tolerated.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),
        # 3. Joint state publisher — fills wheel joints (arm joints come from servo_controller).
        #    用位置参数传 URDF 文件：joint_state_publisher 只认 description_file 参数，
        #    不读 ros__parameters 的 robot_description（那是 robot_state_publisher 的参数），
        #    否则它会一直等 /robot_description 话题而卡住（不发 wheel 关节，RViz 模型缺失）。
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf_file],
            parameters=[
                {'source_list': ['joint_states']},
                {'rate': 20},
            ],
            output='screen',
        ),
        # 3b. 发布 /robot_description 话题：RViz RobotModel 从该话题读 URDF（Description
        #    Source=Topic）。robot_state_publisher/joint_state_publisher 都只发 TF 不发布
        #    该话题，缺了它 RViz 模型不显示。
        Node(
            package='car1_bringup',
            executable='robot_description_pub',
            name='robot_description_pub',
            output='screen',
        ),
        # 4. Chassis controller (IMU filter + odom + EKF + servo controller + init pose)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(chassis_pkg, 'launch/chassis_controller.launch.py')),
        ),
    ])

    return bringup_description
