"""Run the full simulated robot, beam AMCL and Nav2's controller server."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    tune = Path(__file__).resolve().parent
    gazebo_launch = Path(get_package_share_directory('car1_gazebo')) / 'launch/gazebo_math_room.launch.py'
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )
    beam_config = str(tune / 'config/localization_beam.yaml')
    map_server = Node(
        package='nav2_map_server', executable='map_server', output='screen',
        parameters=[beam_config, {'yaml_filename': str(tune / 'maps/math_room_1cm.yaml')}],
    )
    amcl = Node(package='nav2_amcl', executable='amcl', output='screen', parameters=[beam_config])
    controller = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=[LaunchConfiguration('controller_params')],
    )
    nodes = [simulation, map_server, amcl]
    managed_nodes = ['map_server', 'amcl']
    planning_map = LaunchConfiguration('planning_map').perform(context)
    if planning_map:
        nodes.extend([
            Node(
                package='nav2_map_server', executable='map_server',
                name='planning_map_server', output='screen',
                parameters=[{'use_sim_time': True, 'yaml_filename': planning_map,
                             'topic_name': '/planning_map'}],
            ),
            Node(
                package='nav2_planner', executable='planner_server', output='screen',
                parameters=[str(tune / 'config/goal_planner.yaml')],
            ),
        ])
        managed_nodes.extend(['planning_map_server', 'planner_server'])
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': managed_nodes + ['controller_server']}],
    )
    return nodes + [controller, manager]


def generate_launch_description():
    tune = Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('world_file'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('planning_map', default_value=''),
        DeclareLaunchArgument('controller_params', default_value=str(tune / 'config/follow_path.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
