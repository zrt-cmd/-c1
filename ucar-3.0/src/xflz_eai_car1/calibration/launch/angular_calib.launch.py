from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('controller'),
            '/launch/chassis_controller.launch.py'
        ])
    )

    calibrate_angular_node = Node(
        package='calibration',
        executable='calibrate_angular',
        output='screen'
    )

    calibrate_rqt_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='calibrate_rqt_reconfigure'
    )

    return LaunchDescription([
        controller_launch,
        calibrate_angular_node,
        calibrate_rqt_node,
    ])
