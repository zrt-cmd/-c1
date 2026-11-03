# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Platform

- **Hardware**: STM32F407VET6 control board, 4-wheel mecanum chassis, 5-DOF arm + gripper (bus servos)
- **Sensors**: COIN-D6 lidar (serial 230400), HP60C depth camera (USB 640x480@25fps), BMI088 IMU
- **OS**: Ubuntu 22.04 (dev board) / 24.04 (host for simulation)
- **ROS2**: Humble (dev board) / Jazzy (host simulation)
- **Gazebo**: Harmonic (gz-sim)
- **Build system**: colcon (ament_cmake for msg packages, ament_python for the rest)

## Build Commands

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build all
colcon build --symlink-install

# Build driver layer (respecting dependency order)
colcon build --packages-select \
  car1_base_driver_msgs car1_servo_controller_msgs car1_sdk \
  car1_base_driver car1_servo_controller controller car1_arm_controller car1_bringup

# Build a single package
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash

# Run tests
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Launch Commands

```bash
# Full robot bringup (base driver + chassis controller + servo controller)
ros2 launch car1_bringup bringup.launch.py

# Arm controller (requires bringup first)
ros2 launch car1_arm_controller arm_controller.launch.py

# Sensors (lidar + depth camera)
ros2 launch car1_peripherals peripherals.launch.py

# SLAM (includes bringup + lidar + slam_toolbox)
ros2 launch car1_slam slam.launch.py

# Navigation (requires saved map)
ros2 launch car1_navigation navigation.launch.py map:=map_01

# Applications
ros2 launch car1_app line_following.launch.py
ros2 launch car1_app object_tracking.launch.py
ros2 launch car1_app hand_gesture.launch.py
ros2 launch car1_app lidar_controller.launch.py
ros2 launch car1_app track_and_grab.launch.py
ros2 launch car1_app visual_grasp.launch.py

# Interaction (voice + LLM)
ros2 launch car1_interaction full_interaction.launch.py
ros2 launch car1_speech speech.launch.py
ros2 launch car1_llm llm.launch.py

# Gazebo simulation
ros2 launch car1_gazebo gazebo_arm.launch.py
ros2 launch car1_gazebo grasp_demo.launch.py
ros2 launch car1_gazebo grasp_demo_vision.launch.py

# URDF visualization
ros2 launch car1_description display.launch.py

# MoveIt2
ros2 launch car1_moveit_config demo.launch.py
```

## Architecture

### Data Flow

```
cmd_vel (Twist) → controller → car1_base_driver/set_motor (MotorsState) → STM32 (serial 1Mbps)
                      ↓
                 odom_raw (Odometry) → EKF (robot_localization) → /odometry/filtered

car1_base_driver/imu_raw → imu_filter_madgwick → /imu
```

### Package Dependency Graph

```
car1_base_driver_msgs          car1_servo_controller_msgs
         │                              │
         ▼                              ▼
car1_base_driver ──────────▶ car1_servo_controller
         │                              │
         └──────────┬───────────────────┘
                    ▼
              controller
                    │
                    ▼
              car1_bringup
```

### Velocity Command Routing

- `/cmd_vel` — External commands (teleop, app). `odom_publisher` applies a ±0.2 m/s safety clamp on this channel.
- `/controller/cmd_vel` — Internal high-speed commands from Nav2 (no clamp).
- Nav2 path: `controller_server` → `/cmd_vel_nav` → `velocity_smoother` → `/controller/cmd_vel` → `odom_publisher`. The two `cmd_vel` remaps in `car1_navigation/launch/include/navigation_base.launch.py` must stay in place; otherwise Nav2 output falls back to `/cmd_vel` and gets clamped to ±0.2 m/s.

### Arm Control Interfaces

- `/arm_controller/target_pose` (geometry_msgs/PoseStamped) — IK-based positioning
- `/arm_controller/joint_command` (sensor_msgs/JointState) — Direct joint control
- `/arm_controller/set_gripper` (std_srvs/SetBool) — Gripper open/close
- `/arm_controller/go_home` (std_srvs/Trigger) — Return to zero position

### Robot Geometry

| Parameter | Value |
|-----------|-------|
| Wheelbase | 0.1946 m |
| Track width | 0.216 m |
| Wheel diameter | 0.0935 m |
| Serial port | `/dev/car1_stm32` @ 1000000 baud |
| Protocol | 0xAA 0x55 | Func | Len | Data | CRC8 |
| Arm DOF | 5 (bus servo ID 0-4, arm_joint1-5) + gripper (ID 5, r_joint) |
| Arm links | d1=0.0435m, a2=0.12614m, a3=0.12614m, a4=0.074395m |

### Key Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/car1_base_driver/imu_raw` | sensor_msgs/Imu | Raw IMU from STM32 |
| `/car1_base_driver/set_motor` | MotorsState | Motor speed commands |
| `/odom_raw` | nav_msgs/Odometry | Wheel odometry (pre-EKF) |
| `/odometry/filtered` | nav_msgs/Odometry | Fused odometry (post-EKF) |
| `/scan` | sensor_msgs/LaserScan | Lidar (QoS: Best Effort) |
| `/depth_cam/rgb/image_raw` | sensor_msgs/Image | Depth camera RGB |
| `/depth_cam/depth/image_raw` | sensor_msgs/Image | Depth map |
| `/joint_states` | sensor_msgs/JointState | Arm joint positions |

### Serial Port Configuration

Two CH340 USB serial ports, bound by udev rule `/etc/udev/rules.d/99-car1-serial.rules` (physical USB port) to stable aliases:
- STM32 board: `/dev/car1_stm32`, 1000000 baud, protocol header `0xAA 0x55` — config in `driver/car1_base_driver/config/base_driver.yaml`
- COIN-D6 lidar: `/dev/car1_lidar`, 230400 baud — config in `peripherals/car1_peripherals/launch/lidar.launch.py`

### Environment Variables

| Variable | Purpose |
|----------|---------|
| `need_compile` | `True` = use installed packages, `False` = use source paths |
| `ROS_DOMAIN_ID` | Multi-robot separation |

### Gazebo Simulation

Controllers in simulation (via ros2_control):
- `joint_state_broadcaster` — publishes joint states
- `arm_controller` — JointTrajectoryController for arm_joint1-5
- `gripper_controller` — JointTrajectoryController for r_joint

Vision grasp pipeline: RGB-D camera → HSV color detection → depth-based 3D localization → TF transform (camera_depth_link → base_link) → IK → grasp execution.

### Navigation Stack

- Global planner: ThetaStar
- Local planner: MPPI (nav2_mppi_controller, DiffDrive motion model — Y-axis disabled via vy_std=0, vy_max=0; mecanum lateral motion not exploited)
- Localization: AMCL
- SLAM: slam_toolbox (2D laser)
- Note: LaserScan topics use SensorDataQoS — set RViz Reliability to "Best Effort"
