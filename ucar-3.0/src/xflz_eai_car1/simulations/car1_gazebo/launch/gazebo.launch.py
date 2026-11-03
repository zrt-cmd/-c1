from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def generate_launch_description():
    pkg_share = get_package_share_directory('car1_description')
    urdf_original = os.path.join(pkg_share, 'urdf', 'car1.urdf')
    meshes_path = os.path.join(pkg_share, 'meshes')

    with open(urdf_original, 'r') as f:
        urdf_content = f.read()

    urdf_content = urdf_content.replace('package://car1_description/meshes/', f'file://{meshes_path}/')

    fd, urdf_temp = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(urdf_content)

    gazebo_ros_share = get_package_share_directory('ros_gz_sim')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', meshes_path),
        DeclareLaunchArgument('model', default_value='car1', description='Model name'),
        gazebo_launch,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_model',
            arguments=['-name', 'car1', '-file', urdf_temp, '-urdf'],
            output='screen'
        )
    ])
