# Repository Guidelines

## Project Overview

XFLZ Car1 — a 4-mecanum-wheel ROS 2 robot with a 5-DOF bus-servo arm + gripper.
Control stack splits in two:

- **Host (dev board)**: Ubuntu 22.04 + ROS 2 Humble, Python ament_python packages, colcon
  build. Talks to the control board over serial.
- **Firmware (STM32)**: `STM32F407VET6` control board (base.hex), handles motors/servos,
  driven via a framed serial protocol.
- **Simulation (host PC)**: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (`gz-sim`),
  mirrors the same launch structure.

Active code lives only under `src/xflz_eai_car1/`. `src/lander_pi/` is a vendor/reference
stack with a `COLCON_IGNORE` — **never build or assume packages from it at runtime.**

Detailed per-subsystem specs live in `src/xflz_eai_car1/CLAUDE.md`. **Code is the source
of truth; that file (and root CLAUDE.md) drift.** Known drift: actual cmd_vel clamp is
±0.3 m/s lin / ±1.0 rad/s ang (not 0.2), IMU filter is `imu_complementary_filter` (not
`imu_filter_madgwick`), Nav2 uses DWB + ThetaStar (not MPPI), track_width=0.216,
wheel_diameter=0.0935 (both real). Annotation when writing code.

## 2. Architecture & Data Flow

Layered colcon workspace (all under `src/xflz_eai_car1/`):

```
nav / slam / app ─► /cmd_vel (clamped) | /controller/cmd_vel (unclamped)
                        │
                        ▼
     controller  odom_publisher_node.py (50 Hz, 0.5 s cmd timeout)
                        │
             car1_base_driver/set_motor (MotorsState) ─► STM32 (serial 1 Mbps)
                        ▼
                   /odom_raw ─► EKF (robot_localization) ─► /odometry/filtered
              /imu (complementary filter) ──► imu0=vyaw
```

Key chains (exact topic names):

- **cmd_vel routing**: external `/cmd_vel` → `odom_publisher` (clamped
  ±0.3 m/s / ±1.0 rad/s). Internal channel `/controller/cmd_vel` (no clamp).
  Nav2 path: `controller_server` → `/cmd_vel_nav` → `velocity_smoother` →
  `/controller/cmd_vel`. **The two remaps in
  `navigation/car1_navigation/launch/include/navigation_base.launch.py` are
  load-bearing — removing them redirects Nav2 to `/cmd_vel` and the tank gets
  speed-limited.**
- **IMU**: `/car1_base_driver/imu_raw` → `imu_filter` (complementary_filter_node) → `/imu`
- **Odometry**: `odom_publisher_node` integrates mecanum kinematics → `/odom_raw`
  (vx, vy) + `/imu` (vyaw) → `ekf_node` → `/odometry/filtered`
- **Sensors**: `/scan` (SensorDataQoS / Best Effort — RViz must set Reliability=Best
  Effort), `/depth_cam/rgb/image_raw`, `/depth_cam/depth/image_raw`
- **Arm**: `arm_controller_node` — `/arm_controller/target_pose` (PoseStamped, IK),
  `/arm_controller/joint_command` (JointState direct), services `/set_gripper`,
  `/go_home`, `/go_work`, `/emergency_stop`, `/pick`, `/place`
- **Vision grasp**: RGB-D → HSV color detect → depth back-proj → TF
  (`camera_depth_link` → `base_link`) → IK → `set_pick_target` + `pick`

Geometry (from code, `driver/controller/controller/mecanum.py` /
`odom_publisher_node.py`): wheelbase 0.1946 m, track_width 0.216 m, wheel_diameter
0.0935 m, serial `/dev/car1_stm32` @ 1000000 baud, protocol `0xAA 0x55 |Func|Len|Data|CRC8`,
5 bus servos (ID 0–4 → arm_joint1–5) + gripper (ID 5 → r_joint). IK geometry in
`arm/kinematics/kinematics/config.py` (L1≈0.12614, L2≈0.12614, L_TCP≈0.1506).

## 3. Key Directories

```
src/xflz_eai_car1/
  driver/        car1_base_driver (STM32 serial: IMU/battery/motors/servos/LEDs) + _msgs
                 car1_servo_controller (+ _msgs)   bus-servo joint mapping from /joint_states
                 car1_sdk (board protocol lib)     controller (odom_publisher, EKF, IMU, init_pose)
  arm/           car1_kinematics (FK/IK: analytical), car1_arm_controller (IK positioning),
                 car1_gripper_controller
  app/           car1_app: line_following, object_tracking, hand_gesture, lidar_controller,
                 track_and_grab, visual_grasp
  calibration/   calibrate_linear / calibrate_angular / calibrate_servo (console scripts)
  peripherals/   car1_peripherals (lidar + depth_camera launch)
  slam/          car1_slam (slam_toolbox config + saved maps/)
  navigation/    car1_navigation (Nav2: AMCL, ThetaStar, DWB, custom BT without backup)
  simulations/   car1_description (URDF), car1_gazebo (worlds/grasp demos), car1_moveit_config
  bringup/       car1_bringup (system launch — composition root)
  examples/      vendor SDKs: COIN-D6 lidar (m1ct_d2), HP60C depth cam (ascamera)
  test/          car1_web_server (Flask 0.0.0.0:8000), camera_web_viewer (Flask+SocketIO)
  interaction/   car1_llm (chat node, skill_request parsing), car1_speech (ASR/KWS/TTS)
  eye_hand_calibration/  apriltag eye-in-hand calibration (detector/data_collector/…)
```

Root: `fw/` (firmware: base.hex + flash/log tools), `check_ros.sh`, `cleanup_ros.sh`, `issues/`
(untracked scratch — not in .gitignore), `log/` (build logs, ignored).

## 4. Development Commands

```bash
source /opt/ros/humble/setup.bash
cd /home/iflytek/ros2_ws

# Build (symlink-install: Python edits need NO rebuild; C++/msg/CMake changes do)
colcon build --symlink-install
colcon build --packages-select <pkg>            # single pkg
colcon build --packages-up-to <pkg>             # pkg + deps
source install/setup.bash                      # after building

# Tests (lint autosuite is the only pytest): 
colcon test --packages-select <pkg>
colcon test-result --verbose

# Launch — bringup first
ros2 launch car1_bringup bringup.launch.py
ros2 launch car1_arm_controller arm_controller.launch.py
ros2 launch car1_peripherals peripherals.launch.py
ros2 launch car1_slam slam.launch.py
ros2 launch car1_navigation navigation.launch.py map:=map_01
ros2 launch car1_app line_following.launch.py  # or object_tracking / visual_grasp …

# Calibration
ros2 run calibration calibrate_linear
ros2 run calibration calibrate_servo

# Ops / firmware tooling
./check_ros.sh        # ROS2 health: nodes/topics/services → suggests cleanup
./cleanup_ros.sh      # interactive kill of stale ROS processes + FastDDS /dev/shm
cd fw && python3 flash_stm32.py [firmware.hex|.bin|.elf]   # STM32 via pyocd+WCH-Link (auto-loads Keil.STM32F4xx_DFP.3.1.1.pack)
cd fw && python3 log_server.py [--port 8090 --serial /dev/ttyACM0 --baud 115200]   # serial log web viewer
```

**Never edit anything under `install/`** — source lives in `src/`, validated through
`colcon build`.

## 5. Code Conventions & Common Patterns

- **Predom package type**: `ament_python` — `setup.py` + `setup.cfg` + `console_scripts`
  entry_points (each node = one entry point). `ament_cmake` only for msg pkgs
  (`rosidl_generate_interfaces`, DEPENDENCIES std_msgs) and vendor/C++ pkgs
  (simulations, kinematics, eye_hand_calibration).
- **Parameters**: YAML per package under `config/*.yaml`, loaded via
  `Node(parameters=[<get_package_share_directory ...>/config/foo.yaml])`.
- **Launch**: Python launch files, layered composition root = car1_bringup
  (`include`s base_driver.launch.py + chassis_controller.launch.py +
  robot_state_publisher from car1_description/urdf/car1.urdf).
- **Serial protocol**: all board traffic via header `0xAA 0x55 | Func | Len | Data |
  CRC8` (`car1_sdk`); CRC table-based (`checksum_crc8`). Do not add a second framing
  convention.
- **Servo angle ↔ PWM**: `ServosPosition.position` MUST be int (PWM pulse); doubles
  silently corrupt the move. Existing conversion: `angle2pulse`/`pulse2angle` in
  `arm/kinematics`.
- **Err handling**: Python nodes are careful with serial timeouts and `TimerCallback`;
  use explicit checks, don't swallow serial errors (silent fails = stuck robot).

## 6. Important Files

| File | Why |
|---|---|
| `driver/car1_base_driver/config/base_driver.yaml` | serial port/baud, motor_type, imu_frame |
| `driver/controller/config/ekf.yaml` | EKF freq/frames/inputs (odom_raw + imu/vyaw) |
| `driver/controller/config/calibrate_params.yaml` | linear_correction_factor 1.04, angular 0.964 |
| `driver/controller/controller/odom_publisher_node.py` | cmd_vel clamp + mecanum odometry + set_motor |
| `driver/controller/controller/mecanum.py` | mecanum IK/odometry math (geometry here) |
| `driver/car1_servo_controller/config/servo_controller.yaml` | servo id/min/max/orientation map |
| `arm/kinematics/kinematics/config.py`, `fk.py`, `ik_analytical.py` | 5-DOF+TCP geometry, FK, analytical IK |
| `arm/car1_arm_controller/car1_arm_controller/arm_controller.py` | arm services + IK placement |
| `navigation/car1_navigation/launch/include/navigation_base.launch.py` | **the two cmd_vel remaps** |
| `navigation/car1_navigation/config/nav2_params.yaml` | AMCL/ThetaStar/DWB/velocity_smoother |
| `slam/car1_slam/config/slam.yaml` | slam_toolbox settings |
| `bringup/launch/bringup.launch.py` | composition root |

## 7. Runtime / Tooling Preferences

- **Runtime**: ROS 2 Humble on board (Py 3.10), Jazzy on sim host. Shell: `setup.bash`
  FIRST (not ROS; Humble/Jazzy incompatible in one shell).
- **Serial ports (CH340)**: STM32 `/dev/car1_stm32` @ 1000000; lidar `/dev/car1_lidar`
  @ 230400 (active launch) — vendor fallback `/dev/sc_mini`; fw debug `/dev/ttyACM0`
  @ 115200 (`log_server.py`). Fixed aliases come from
  `fw/tools/99-car1-serial.rules` (udev, binds physical USB ports `1-1.3`/`1-1.4`);
  serial-permission: user must be in `dialout`.
- **Env vars**: `need_compile`, `ROS_DOMAIN_ID` are **doc-only** — never read by
  launch/runtime code; don't rely on them.
- **QoS**: lidar/scan = SensorDataQoS Best-Effort; RViz viewers need reliability
  Best Effort for /scan.
- **Bundled vendor SDKs** in `examples/` are read-only references; the runtime stack
  does not use them for launch.
- `check_ros.sh` needs ROS env sourced; `cleanup_ros.sh` removes FastDDS shm.

## 8. Testing & QA

- **Registered pytest surface is minimal** — only `navigation/car1_navigation/test/`
  (ament copyright/flake8/pep257 autos). `colcon test --packages-select` runs those.
- Real algorithm tests live ad hoc in package dirs and are **hand-run, not colcon**:
  - `arm/kinematics/kinematics/test_fk_ik.py` — FK/IK round-trip + PWM mapping (`python3`)
  - `arm/kinematics/kinematics/test_transform.py`, `custom_ik_test.py` (1.5 mm threshold)
  - `app/car1_app/car1_app/test_hsv_detection.py`, `test_3d_localization.py` — live-camera ROS nodes
  - `slam/car1_slam/test/imu_canine_test.py` — direct serial IMU/mecanum check
  - `calibration/scripts/calib…serial_test.py` — bypasses ROS to test protocol+math
- **Coverage gap**: driver layer (CRC/framing, motor packing, servo controller),
  controller odom math, eye_hand_calibration, Nav2/SLAM — no automated tests.
- **Expected behavior**: verify hardware changes against actual simulation or real
  launch output (e.g., `ros2 topic echo /odometry/filtered`); don't rely on the
  sparse test suite to catch regressions.