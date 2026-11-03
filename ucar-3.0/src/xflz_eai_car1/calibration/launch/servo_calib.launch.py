import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_driver_pkg = get_package_share_directory('car1_base_driver')

    base_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_driver_pkg, 'launch/base_driver.launch.py'))
    )

    calibrate_servo_node = Node(
        package='calibration',
        executable='servo_calib_gui',  # GUI：5 关节独立控制面板（joint1~joint5）
        output='screen',
        parameters=[{
            'servo_ids': [0, 1, 2, 3, 4],  # joint1~5 → servo ID 0~4
            'center_position': 1500,  # PWM center (range 500-2500)
        }]
    )

    return LaunchDescription([
        base_driver_launch,
        calibrate_servo_node,
    ])
