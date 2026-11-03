import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler,
    IncludeLaunchDescription, OpaqueFunction, TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def setup_simulation(context, *args, **kwargs):
    """启动 Gazebo 仿真 + 机器人 + 控制器 + 物体 + 抓取节点"""
    car1_gazebo_path = get_package_share_directory('car1_gazebo')
    car1_description_path = get_package_share_directory('car1_description')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    mode = LaunchConfiguration('mode').perform(context)

    # 处理 URDF
    xacro_file = os.path.join(car1_gazebo_path, 'urdf', 'car1.gazebo.xacro')
    import subprocess
    urdf_content = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')

    meshes_path = os.path.join(car1_description_path, 'meshes')
    urdf_content = urdf_content.replace(
        'package://car1_description/meshes/',
        f'file://{meshes_path}/'
    )

    # 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r empty.sdf']}.items()
    )

    # Gazebo-ROS 时钟桥接 (将 Gazebo 的 /clock 发布到 ROS2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': use_sim_time == 'true'
        }]
    )

    # Joint State Publisher
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

    # 生成机器人
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'car1',
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # 生成立方体 (在机器人正前方 0.25m 处)
    cube_sdf_path = os.path.join(car1_gazebo_path, 'models', 'cube', 'model.sdf')
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', cube_sdf_path,
            '-name', 'red_cube',
            '-x', '0.25', '-y', '0.0', '-z', '0.02'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # 控制器
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

    # 抓取演示节点（根据 mode 参数选择）
    # mode='ik': 几何IK版本 (默认)
    # mode='manual': 手动参考版本
    # mode='none': 不启动抓取节点（仅环境，用于手动控制）

    grasp_demo_ik = Node(
        package='car1_gazebo',
        executable='grasp_demo.py',
        name='grasp_demo',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'ik'"]))
    )

    grasp_demo_manual = Node(
        package='car1_gazebo',
        executable='grasp_demo_manual_ref.py',
        name='grasp_demo_manual_ref',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'manual'"]))
    )

    return [
        gazebo,
        clock_bridge,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,

        # 生成物体 (与机器人同时)
        spawn_cube,

        # 等待机器人生成后启动控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
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
        # 等待控制器就绪后启动抓取节点（如果 mode 不是 'none'）
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gripper_controller_spawner,
                on_exit=[grasp_demo_ik, grasp_demo_manual]
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'mode',
            default_value='ik',
            description='抓取模式: ik=几何IK(默认), manual=手动参考, none=仅环境(手动控制)'
        ),
        OpaqueFunction(function=setup_simulation)
    ])
