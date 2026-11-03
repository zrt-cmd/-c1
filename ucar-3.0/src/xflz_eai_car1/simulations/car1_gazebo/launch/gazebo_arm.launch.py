import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def process_urdf(context, *args, **kwargs):
    """处理 URDF，将 package:// 路径转换为 file:// 路径"""
    # 获取包路径
    car1_gazebo_path = get_package_share_directory('car1_gazebo')
    car1_description_path = get_package_share_directory('car1_description')

    # Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # URDF 文件
    xacro_file = os.path.join(car1_gazebo_path, 'urdf', 'car1.gazebo.xacro')

    # 处理 xacro 生成 URDF
    import subprocess
    urdf_content = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')

    # 替换 package:// 路径为 file:// 路径
    meshes_path = os.path.join(car1_description_path, 'meshes')
    urdf_content = urdf_content.replace(
        'package://car1_description/meshes/',
        f'file://{meshes_path}/'
    )

    # 写入临时文件
    fd, urdf_temp = tempfile.mkstemp(suffix='.urdf', text=True)
    with os.fdopen(fd, 'w') as f:
        f.write(urdf_content)

    # 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r empty.sdf']}.items()
    )

    # Gazebo-ROS 时钟桥接
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

    # 生成机器人模型
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'car1',
            '-allow_renaming', 'true',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # 控制器启动器
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

    # 返回节点列表
    return [
        gazebo,
        clock_bridge,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,

        # 等待模型生成后启动控制器
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
        DeclareLaunchArgument('world', default_value='empty'),
        OpaqueFunction(function=process_urdf)
    ])
