import os
import tempfile
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def process_urdf(context, *args, **kwargs):
    car1_gazebo_path = get_package_share_directory('car1_gazebo')
    car1_description_path = get_package_share_directory('car1_description')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    xacro_file = os.path.join(car1_gazebo_path, 'urdf', 'car1.gazebo.xacro')
    import subprocess
    urdf_content = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')
    meshes_path = os.path.join(car1_description_path, 'meshes')
    urdf_content = urdf_content.replace(
        'package://car1_description/meshes/',
        f'file://{meshes_path}/'
    )
    fd, urdf_temp = tempfile.mkstemp(suffix='.urdf', text=True)
    with os.fdopen(fd, 'w') as f:
        f.write(urdf_content)
    sys.path.insert(0, os.path.join(car1_gazebo_path, 'launch'))
    from mecanum_sdf import build_mecanum_sdf
    sdf_file = build_mecanum_sdf(urdf_temp)

    world_file = LaunchConfiguration('world_file').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    gz_args = f'-s -r "{world_file}"' if headless else f'-r "{world_file}" --render-engine-gui ogre'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': gz_args
        }.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': use_sim_time == 'true'
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['/controller_manager/joint_states'],
            'rate': 20.0,
            'use_sim_time': use_sim_time == 'true'
        }]
    )

    # Spawn at region G (top-middle pocket), facing +X.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', sdf_file,
            '-name', 'car1',
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.1',
            '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': True}]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return [
        gazebo,
        bridge,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[gripper_controller_spawner]
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='0.15'),
        DeclareLaunchArgument('spawn_y', default_value='1.7'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(
            get_package_share_directory('car1_gazebo'), 'worlds', 'math_room.sdf')),
        OpaqueFunction(function=process_urdf)
    ])
