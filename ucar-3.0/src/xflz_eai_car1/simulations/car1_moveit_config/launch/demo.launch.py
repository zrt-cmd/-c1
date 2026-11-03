import os
import re
import tempfile
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Command-line arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gazebo = LaunchConfiguration('use_gazebo', default='false')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo", default_value="false", description="Use Gazebo for simulation"
    )

    # --- Build merged URDF: car1.urdf + ros2_control ---
    car1_moveit_share = get_package_share_directory("car1_moveit_config")
    car1_desc_share = get_package_share_directory("car1_description")

    # Read the base URDF
    base_urdf_path = os.path.join(car1_desc_share, "urdf", "car1.urdf")
    with open(base_urdf_path, 'r') as f:
        base_urdf = f.read()

    # Process the ros2_control xacro to get the ros2_control XML block
    ros2_control_xacro_path = os.path.join(car1_moveit_share, "config", "car1.ros2_control.xacro")

    # Create a minimal wrapper xacro that includes the ros2_control macro
    wrapper_xacro = (
        '<?xml version="1.0"?>\n'
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="car1">\n'
        f'  <xacro:include filename="{ros2_control_xacro_path}" />\n'
        '  <xacro:car1_ros2_control name="car1_ros2_control" use_mock_hardware="true" />\n'
        '</robot>\n'
    )
    fd, wrapper_path = tempfile.mkstemp(suffix='.xacro')
    with os.fdopen(fd, 'w') as f:
        f.write(wrapper_xacro)

    ros2_control_doc = xacro.process_file(wrapper_path)
    ros2_control_xml = ros2_control_doc.toxml()
    os.unlink(wrapper_path)

    # Extract the ros2_control block and merge into base URDF
    match = re.search(r'(<ros2_control.*?</ros2_control>)', ros2_control_xml, re.DOTALL)
    if match:
        ros2_control_block = match.group(1)
        merged_urdf = base_urdf.replace('</robot>', ros2_control_block + '\n</robot>')
    else:
        merged_urdf = base_urdf

    # Write merged URDF to a temp file for MoveItConfigsBuilder
    fd, merged_urdf_path = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(merged_urdf)

    # --- Build MoveIt config using the merged URDF ---
    moveit_config = (
        MoveItConfigsBuilder("car1", package_name="car1_moveit_config")
        .robot_description(file_path=merged_urdf_path)
        .robot_description_semantic(file_path="config/car1.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    robot_description = {"robot_description": merged_urdf}

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            robot_description,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    tutorial_mode = LaunchConfiguration("rviz_tutorial")

    current_dir = os.path.dirname(os.path.realpath(__file__))
    rviz_base = os.path.abspath(os.path.join(current_dir, '..', 'rviz'))

    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_empty_config = os.path.join(rviz_base, "moveit_empty.rviz")

    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(tutorial_mode),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=UnlessCondition(tutorial_mode),
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_footprint"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ros2_control using mock hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("car1_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=UnlessCondition(use_gazebo),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_gazebo),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(use_gazebo),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(use_gazebo),
    )

    return LaunchDescription(
        [
            declare_use_sim_time_argument,
            tutorial_arg,
            db_arg,
            ros2_control_hardware_type,
            use_gazebo_arg,
            rviz_node,
            rviz_node_tutorial,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
