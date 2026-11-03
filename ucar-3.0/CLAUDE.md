# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Platform

- **Hardware**: RK3576 ARM board (aarch64), STM32F407VET6 control board, 4-wheel mecanum chassis, 5-DOF arm + gripper (bus servos)
- **Sensors**: COIN-D6 lidar (serial 230400), HP60C depth camera (USB 640x480@25fps), BMI088 IMU
- **OS / ROS**: Ubuntu 22.04 + ROS 2 Humble (`/opt/ros/humble`) on the board; Ubuntu 24.04 + ROS 2 Jazzy on the simulation host
- **Gazebo**: Harmonic (gz-sim) on the sim host
- **Build system**: colcon (ament_cmake for msg packages, ament_python for the rest)

## Workspace Layout

Two source trees live under `src/`:
- `src/xflz_eai_car1/` — the active robot stack (all packages below live here).
- `src/lander_pi/` — a reference/vendor stack carrying its own `CLAUDE.md`; it has a `COLCON_IGNORE` so colcon does not build it. Do not assume its packages are available at runtime.

There is also a nested `src/xflz_eai_car1/CLAUDE.md` with deeper per-subsystem detail. Keep it and this root file consistent when either changes.

## Build Commands

```bash
source /opt/ros/humble/setup.bash
cd /home/iflytek/ros2_ws

# Build everything (use symlink-install so Python edits don't require rebuilds)
colcon build --symlink-install

# Build the driver layer in dependency order
colcon build --packages-select \
  car1_base_driver_msgs car1_servo_controller_msgs car1_sdk \
  car1_base_driver car1_servo_controller controller car1_arm_controller car1_bringup

# Single package / package + deps
colcon build --packages-select <package_name>
colcon build --packages-up-to <package_name>

# Source the overlay after building
source install/setup.bash

# Tests
colcon test --packages-select <package_name>
colcon test-result --verbose
```

**Never edit files under `install/` directly.** All changes go in `src/` and are validated through `colcon build`. The deliverable is source others compile themselves.

## Code Change Protocol

**CRITICAL: Before making any code changes, you MUST:**

1. **Diagnose with evidence** — collect logs, error messages, actual output values. Never diagnose based solely on reading code.
2. **State your plan clearly**:
   - What file/lines will be changed
   - Why this change is needed (root cause)
   - What the expected result is (specific values/behavior)
   - How to verify it worked (test commands + expected output)
3. **Wait for user confirmation** before executing the change.
4. **After the change** — run the verification steps and report actual results vs. expected.

**Example of correct workflow**:
```
❌ WRONG: "I see the bug in line 42, let me fix it" [starts editing]
✓ RIGHT: "Based on the log showing X=0.267→-0.012, I believe line 42 has 
         reversed parameters. I'll swap A and B, expecting X≈0.26. 
         Verify by running `ros2 launch ...` and checking the [TF] log line.
         Proceed?"
```

This protocol applies to ALL changes: bug fixes, refactoring, features, config updates.

## TF Transform Reference

**`lookup_transform(target_frame, source_frame, time)`** returns transform FROM source TO target.

To transform a point from frame A to frame B:
```python
# Correct usage
tf_B_from_A = buffer.lookup_transform(
    'frame_B',        # target (where you want the point)
    'frame_A',        # source (where the point is now)
    time
)
point_in_B = do_transform_point(point_in_A, tf_B_from_A)
```

**Common mistake**: swapping the parameters → you get the inverse transform, coordinates will be completely wrong.

**Verification**: After any TF change, compare transformed coordinates against known ground truth or previous working values.

## Launch Commands

```bash
# Full base bringup: base_driver + controller (IMU filter + odom + EKF + servo controller + init pose)
ros2 launch car1_bringup bringup.launch.py

# Arm controller (run after bringup)
ros2 launch car1_arm_controller arm_controller.launch.py

# Sensors (lidar + depth camera)
ros2 launch car1_peripherals peripherals.launch.py

# SLAM / Navigation
ros2 launch car1_slam slam.launch.py
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

# Test interaction
ros2 topic pub /interaction/tts/speak std_msgs/msg/String "{data: '你好'}" --once
ros2 topic pub /interaction/llm/user_input std_msgs/msg/String "{data: '导航到厨房'}" --once

# Calibration helpers
ros2 run calibration calibrate_linear
ros2 run calibration calibrate_angular
ros2 run calibration calibrate_servo

# Simulation (sim host: Jazzy + Gazebo Harmonic)
ros2 launch car1_gazebo gazebo_arm.launch.py
ros2 launch car1_gazebo grasp_demo.launch.py
ros2 launch car1_gazebo grasp_demo_vision.launch.py
ros2 launch car1_description display.launch.py
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

The `controller` package is the chassis controller (mecanum kinematics, odometry integration, EKF, IMU filter, init pose). `car1_bringup` includes `base_driver.launch.py` followed by `controller/launch/chassis_controller.launch.py`.

### Package Dependency Graph

```
car1_base_driver_msgs          car1_servo_controller_msgs
         │                              │
         ▼                              ▼
car1_base_driver ──────────▶ car1_servo_controller
         │                              │
         └──────────┬───────────────────┘
                    ▼
              controller          (chassis controller)
                    │
                    ▼
              car1_bringup
```

### Package Map (`src/xflz_eai_car1/`)

- **driver/** — `car1_base_driver` (STM32 serial: IMU, battery, motors, servos, LEDs, buzzer, OLED, RGB), `car1_base_driver_msgs`, `car1_servo_controller` (+ `_msgs`), `car1_sdk` (board protocol lib), `controller` (chassis controller: `odom_publisher`, `init_pose`).
- **arm/** — `car1_arm_controller` (high-level IK positioning), `car1_kinematics` (FK/IK).
- **app/** — `car1_app`: `line_following`, `object_tracking`, `hand_gesture`, `lidar_controller`, `track_and_grab`, `visual_grasp`.
- **calibration/** — `calibrate_linear`, `calibrate_angular`, `calibrate_servo`.
- **interaction/** — `car1_speech` (`wake_word_node`, `asr_node`, `tts_node`), `car1_llm` (`chat_node`, `skill_executor`), `car1_interaction` (full stack launch). Voice pipeline: mic → wake → `/interaction/asr/text` → LLM → `/interaction/tts/speak`. Skill calls encoded inline as `[SKILL:name|k:v|...]` and dispatched to ROS services/actions per `car1_llm/config/skills.yaml`. API keys (iFlytek/OpenAI) live in `car1_speech/config/*.yaml` and `car1_llm/config/llm.yaml`.
- **peripherals/** — `car1_peripherals` launch files for COIN-D6 lidar and HP60C depth camera.
- **navigation/** — `car1_navigation` (Nav2: AMCL DifferentialMotionModel, ThetaStar global, MPPI local with DiffDrive motion model (Y disabled), custom BT without backup recovery).
- **slam/** — `car1_slam` (slam_toolbox config + saved `maps/`).
- **simulations/** — `car1_description` (URDF), `car1_gazebo` (worlds + grasp demos), `car1_moveit_config`.
- **bringup/** — `car1_bringup` (system launch).
- **examples/** — vendor sensor examples (COIN-D6 lidar, HP60C camera, yahboomcar_*, ORB-SLAM2).
- **test/** — `car1_web_server` (`server_cors.py` + urdf/meshes/viewer for browser URDF viewing); `camera_web_viewer` (Flask+SocketIO server streaming HP60C RGB/Depth/pointcloud to a LAN browser on `0.0.0.0:8000`, with 2D dual view, 3D point cloud, and click-to-measure depth — launch via `ros2 launch camera_web_viewer camera_viewer.launch.py`).

### Velocity Command Routing

- `/cmd_vel` — external commands (teleop, app). `odom_publisher` applies a ±0.2 m/s safety clamp on this channel.
- `/controller/cmd_vel` — internal high-speed commands from Nav2 (no clamp).
- Nav2 path: `controller_server` → `/cmd_vel_nav` → `velocity_smoother` → `/controller/cmd_vel` → `odom_publisher`. The two `cmd_vel` remaps in `car1_navigation/launch/include/navigation_base.launch.py` must stay in place; otherwise Nav2 output falls back to `/cmd_vel` and gets clamped to ±0.2 m/s.

### Arm Control Interfaces

- `/arm_controller/target_pose` (geometry_msgs/PoseStamped) — IK positioning
- `/arm_controller/joint_command` (sensor_msgs/JointState) — direct joint control
- `/arm_controller/set_gripper` (std_srvs/SetBool) — gripper open/close
- `/arm_controller/go_home` (std_srvs/Trigger) — return to zero

### Robot Geometry

| Parameter | Value |
|-----------|-------|
| Wheelbase | 0.1946 m |
| Track width | 0.2117 m |
| Wheel diameter | 0.097 m |
| Serial port | `/dev/car1_stm32` @ 1000000 baud |
| Protocol | `0xAA 0x55` \| Func \| Len \| Data \| CRC8 |
| Arm DOF | 5 (bus servo ID 0-4, arm_joint1-5) + gripper (ID 5, r_joint) |
| Arm links | d1=0.0435 m, a2=0.12614 m, a3=0.12614 m, a4=0.074395 m |

### Key Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/car1_base_driver/imu_raw` | sensor_msgs/Imu | Raw IMU from STM32 |
| `/imu` | sensor_msgs/Imu | Filtered IMU (Madgwick) |
| `/car1_base_driver/set_motor` | MotorsState | Motor speed commands |
| `/odom_raw` | nav_msgs/Odometry | Wheel odometry (pre-EKF) |
| `/odometry/filtered` | nav_msgs/Odometry | Fused odometry (post-EKF) |
| `/scan` | sensor_msgs/LaserScan | Lidar (SensorDataQoS / Best Effort) |
| `/depth_cam/rgb/image_raw` | sensor_msgs/Image | Depth camera RGB (used by app nodes) |
| `/depth_cam/depth/image_raw` | sensor_msgs/Image | Depth map |
| `/joint_states` | sensor_msgs/JointState | Arm joint positions |
| `car1_base_driver/battery` | std_msgs/UInt16 | Battery voltage |
| `/interaction/wake_word/detected` | std_msgs/Bool | Voice wake event |
| `/interaction/asr/text` | std_msgs/String | Speech recognition output |
| `/interaction/llm/assistant_reply` | std_msgs/String | LLM response |
| `/interaction/tts/speak` | std_msgs/String | Text-to-speech input |

### Serial Ports

Two CH340 USB serials, bound by udev rule `/etc/udev/rules.d/99-car1-serial.rules` (physical USB port) to stable aliases:
- STM32 board: `/dev/car1_stm32`, 1000000 baud, header `0xAA 0x55` — config in `driver/car1_base_driver/config/base_driver.yaml`.
- COIN-D6 lidar: `/dev/car1_lidar`, 230400 baud — config in `peripherals/car1_peripherals/launch/lidar.launch.py`.

### Environment Variables

| Variable | Purpose |
|----------|---------|
| `need_compile` | `True` = use installed packages, `False` = use source paths |
| `ROS_DOMAIN_ID` | Multi-robot separation |

### Vision Grasp Pipeline

RGB-D camera → HSV color detection → depth-based 3D localization → TF transform (`camera_depth_link` → `base_link`) → IK → grasp execution. Gazebo uses ros2_control: `joint_state_broadcaster`, `arm_controller` (JointTrajectoryController for arm_joint1-5), `gripper_controller` (r_joint).

### Protocol / Tutorial Docs

`src/xflz_eai_car1/` carries `servo_protocol.md`, `ros_controller_protocol.md`, and `tutorial.md` for board/servo protocol and walkthroughs. Note: RViz LaserScan display requires Reliability = "Best Effort".
