"""Full robot, AMCL and global planner; no velocity-producing navigation nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('car1_gazebo'), 'launch', 'gazebo_math_room.launch.py')))
    nodes = [Node(package=package, executable=executable, name=name,
                  parameters=[params], output='screen') for package, executable, name in [
        ('nav2_map_server', 'map_server', 'map_server'),
        ('nav2_amcl', 'amcl', 'amcl'),
        ('nav2_planner', 'planner_server', 'planner_server')]]
    manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                   name='lifecycle_manager_probe', output='screen', parameters=[{
                       'use_sim_time': True, 'autostart': True,
                       'node_names': ['map_server', 'amcl', 'planner_server']}])
    return LaunchDescription([DeclareLaunchArgument('params_file'), simulation,
                              TimerAction(period=15., actions=nodes + [manager])])
