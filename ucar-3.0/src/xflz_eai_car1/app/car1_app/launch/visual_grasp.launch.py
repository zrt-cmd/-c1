#!/usr/bin/env python3
"""All-in-one launch for visual grasp.

Starts everything needed for visual grasping in one command:
  1. bringup (base_driver + servo_controller + robot_state_publisher + IMU/EKF)
  2. arm_controller (IK + pick/place services)
  3. depth_camera (HP60C driver)
  4. visual_grasp node (state machine)
  5. rqt_image_view showing /visual_grasp/visualization (optional, on by default)

The visual_grasp node starts in IDLE and waits for `/visual_grasp/start` to
begin. After each grasp (success or failure) it returns to IDLE, so you can
run repeated grasps by calling the start service again.

Usage:
    ros2 launch car1_app visual_grasp.launch.py

Optional overrides:
    launch ... start_bringup:=false          # if bringup already running elsewhere
    launch ... start_arm:=false              # if arm_controller already running
    launch ... start_camera:=false           # if depth camera already running
    launch ... start_rqt:=false              # disable rqt_image_view window
    launch ... wait_for_confirmation:=false  # auto-grasp after detection (no pause)

Trigger a grasp (from another terminal):
    ros2 service call /visual_grasp/start std_srvs/srv/Trigger

Confirm during pause (if wait_for_confirmation=true):
    ros2 service call /visual_grasp/confirm std_srvs/srv/Trigger
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---------- Launch arguments ----------
    args = [
        DeclareLaunchArgument('start_bringup', default_value='true',
                              description='Start bringup (base_driver + controllers + RSP)'),
        DeclareLaunchArgument('start_arm', default_value='true',
                              description='Start arm_controller'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='Start HP60C depth camera driver'),
        DeclareLaunchArgument('start_rqt', default_value='true',
                              description='Open rqt_image_view showing the visualization'),
        DeclareLaunchArgument('wait_for_confirmation', default_value='true',
                              description='Pause after detection for user confirm'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='Detect only, do not actually grasp'),
    ]

    # ---------- Package paths ----------
    bringup_pkg = get_package_share_directory('car1_bringup')
    arm_pkg = get_package_share_directory('car1_arm_controller')
    peripherals_pkg = get_package_share_directory('car1_peripherals')

    # ---------- Dependency launches (staggered start) ----------
    # bringup FIRST — provides serial driver, servo_controller (with
    # refresh_positions service), and robot_state_publisher (URDF TF tree).
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch/bringup.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_bringup')),
    )

    # arm_controller at t+5s — needs base_driver's servo services and RSP's TF.
    # Force start_bringup=false when including it, otherwise our outer
    # `start_bringup` argument (which controls whether WE start bringup) leaks
    # into arm_controller.launch.py's same-named argument and starts bringup
    # a second time → two processes fighting for /dev/car1_stm32.
    arm_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(arm_pkg, 'launch/arm_controller.launch.py')),
                launch_arguments={'start_bringup': 'false'}.items(),
                condition=IfCondition(LaunchConfiguration('start_arm')),
            )
        ],
    )

    # depth camera at t+8s — independent of arm, but staggered to avoid
    # simultaneous init spikes on the small board.
    camera_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(peripherals_pkg, 'launch/depth_camera.launch.py')),
                condition=IfCondition(LaunchConfiguration('start_camera')),
            )
        ],
    )

    # ---------- Visual grasp node at t+12s ----------
    # By the time we start visual_grasp: bringup is warm (~7s in), arm_controller
    # has had 7s to come up, camera has had 4s. Any residual /joint_states or
    # camera_info that arrives late is handled by the node itself (it waits
    # in INIT until camera_info arrives).
    visual_grasp_node = Node(
        package='car1_app',
        executable='visual_grasp',
        output='screen',
        parameters=[{
            # ---- observation pose (base_link frame) ----
            'observe_pose': [0.20, 0.0, 0.22, 3.14159],  # x, y, z, pitch(rad). pi=gripper down

            # ---- HSV thresholds for red cube (two ranges OR'd, red wraps H) ----
            'target_color_hsv_lower': [0, 100, 80],
            'target_color_hsv_upper': [15, 255, 255],
            'target_color_hsv_lower2': [160, 100, 80],
            'target_color_hsv_upper2': [180, 255, 255],

            # ---- runtime knobs ----
            'max_detection_attempts': 10,
            'dry_run': LaunchConfiguration('dry_run'),
            'enable_viz': True,
            'wait_for_confirmation': LaunchConfiguration('wait_for_confirmation'),
        }],
    )

    visual_grasp_delayed = TimerAction(period=12.0, actions=[visual_grasp_node])

    # ---------- rqt_image_view showing visualization ----------
    # Uses ExecuteProcess so we can pass the topic as a CLI arg to rqt.
    # This opens a Qt window that works fine over SSH -X (unlike cv2.imshow).
    rqt_process = TimerAction(
        period=14.0,  # slightly after visual_grasp so the topic exists to auto-select
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                     '/visual_grasp/visualization'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_rqt')),
            )
        ],
    )

    return LaunchDescription(args + [
        bringup_launch,
        arm_launch,
        camera_launch,
        visual_grasp_delayed,
        rqt_process,
    ])
