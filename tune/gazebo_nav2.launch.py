import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    car1_gazebo_path = get_package_share_directory('car1_gazebo')
    car1_description_path = get_package_share_directory('car1_description')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    # --- robot description ---
    import subprocess
    xacro_file = os.path.join(car1_gazebo_path, 'urdf', 'car1.gazebo.xacro')
    urdf_content = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')
    meshes_path = os.path.join(car1_description_path, 'meshes')
    urdf_content = urdf_content.replace(
        'package://car1_description/meshes/', f'file://{meshes_path}/')
    fd, urdf_temp = tempfile.mkstemp(suffix='.urdf', text=True)
    with os.fdopen(fd, 'w') as f:
        f.write(urdf_content)

    # --- Gazebo headless server ---
    world_file = os.path.join(car1_gazebo_path, 'worlds', 'math_room.sdf')
    headless = LaunchConfiguration('headless').perform(context)
    if headless == 'true':
        gz_args = f'-s -r --headless-rendering {world_file}'
    else:
        gz_args = f'-r {world_file} --render-engine-gui ogre'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # --- Gazebo <-> ROS bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
    )

    # Nav2 publishes /controller/cmd_vel; relay it to /cmd_vel for the bridge.
    cmd_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/controller/cmd_vel', '/cmd_vel'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content,
                     'use_sim_time': use_sim_time == 'true'}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['/controller_manager/joint_states'],
                     'rate': 20.0, 'use_sim_time': True}],
    )

    # proxy: spawn the kinematic box model (holonomic, no wheel physics)
    if LaunchConfiguration('proxy').perform(context) == 'true':
        spawn_args = ['-file', '/home/zrtzrt/tune/proxy_robot.sdf',
                      '-name', 'car1', '-allow_renaming', 'true',
                      '-x', '0.15', '-y', '1.70', '-z', '0.0', '-Y', '0.0']
    else:
        spawn_args = ['-topic', 'robot_description', '-name', 'car1',
                      '-allow_renaming', 'true', '-x', '0.15', '-y', '1.70',
                      '-z', '0.1', '-Y', '0.0']

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=spawn_args,
        parameters=[{'use_sim_time': True}],
    )

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
               output='screen', parameters=[{'use_sim_time': True}],
               condition=UnlessCondition(LaunchConfiguration('proxy')))
    arm = Node(package='controller_manager', executable='spawner',
               arguments=['arm_controller', '--controller-manager', '/controller_manager'],
               output='screen', parameters=[{'use_sim_time': True}],
               condition=UnlessCondition(LaunchConfiguration('proxy')))
    grip = Node(package='controller_manager', executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                output='screen', parameters=[{'use_sim_time': True}],
                condition=UnlessCondition(LaunchConfiguration('proxy')))

    # --- Nav2 nodes (sim params file) ---
    map_server = Node(package='nav2_map_server', executable='map_server',
                      name='map_server', parameters=[params_file], output='screen')
    amcl = Node(package='nav2_amcl', executable='amcl',
                name='amcl', parameters=[params_file], output='screen')
    life_loc = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name='lifecycle_manager_localization', parameters=[params_file],
                    output='screen')

    controller = Node(package='nav2_controller', executable='controller_server',
                      name='controller_server', parameters=[params_file],
                      remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen')
    planner = Node(package='nav2_planner', executable='planner_server',
                   name='planner_server', parameters=[params_file], output='screen')
    path_smoother = Node(package='nav2_smoother', executable='smoother_server',
                         name='smoother_server', parameters=[params_file],
                         output='screen')
    behavior = Node(package='nav2_behaviors', executable='behavior_server',
                    name='behavior_server', parameters=[params_file], output='screen')
    bt_nav = Node(package='nav2_bt_navigator', executable='bt_navigator',
                  name='bt_navigator', parameters=[params_file], output='screen')
    smoother = Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                    name='velocity_smoother', parameters=[params_file],
                    remappings=[('cmd_vel', 'cmd_vel_nav'),
                                ('cmd_vel_smoothed', 'controller/cmd_vel')],
                    output='screen')
    life_nav = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name='lifecycle_manager_navigation', parameters=[params_file],
                    output='screen')

    nav_timer = TimerAction(period=18.0,
                     condition=IfCondition(LaunchConfiguration('start_nav2')),
                     actions=[
        map_server, amcl, life_loc,
        controller, planner, path_smoother, behavior, bt_nav, smoother, life_nav,
    ])

    return [
        gazebo, bridge, cmd_relay, robot_state_publisher, joint_state_publisher, spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[grip])),
        nav_timer,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument('proxy', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.expanduser('~/tune/nav2_sim_template.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
