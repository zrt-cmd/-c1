# xflz_robot_car1 上手教程

> 本教程面向学习者，从易到难介绍如何使用 xflz_robot_car1 机器人平台。

---

## 目录

- [项目概述](#项目概述)
- [环境准备](#环境准备)
- [学习路径](#学习路径)
  - [Level 0: 仿真验证（无需硬件）](#level-0-仿真验证无需硬件)
  - [Level 1: 硬件驱动层](#level-1-硬件驱动层)
  - [Level 2: 传感器集成](#level-2-传感器集成)
  - [Level 3: SLAM 建图](#level-3-slam-建图)
  - [Level 4: 自主导航](#level-4-自主导航)
  - [Level 5: 智能应用](#level-5-智能应用)
  - [Level 6: 语音交互](#level-6-语音交互)
    - [6.1 音频硬件测试](#61-音频硬件测试audio_testpy)
    - [6.2 语音识别测试](#62-语音识别测试asr_testpy)
    - [6.3 语音唤醒测试](#63-语音唤醒测试kws_testpy)
    - [6.4 本地大模型服务](#64-本地大模型服务llm_serverpy--llm_testpy)
    - [6.5 Function Calling 工具调用](#65-function-calling-工具调用)
    - [6.6 语音合成测试](#66-语音合成测试tts_testpy)
- [常见问题](#常见问题)

---

## 项目概述

### 项目简介

xflz_robot_car1 是一个基于 ROS2 的移动机械臂平台，具备以下功能：

- **移动底盘**：麦克纳姆轮全向移动
- **机械臂**：5 自由度机械臂 + 夹爪
- **感知**：激光雷达 + 深度相机
- **导航**：SLAM 建图 + 自主导航
- **语音**：语音识别（ASR）+ 语音合成（TTS）
- **应用**：巡线、追踪、手势控制、视觉抓取

### 硬件配置

| 硬件 | 型号 | 说明 |
|------|------|------|
| 主控 | STM32F407VET6 | 串口 1Mbps |
| 底盘 | 麦克纳姆轮 × 4 | 全向移动 |
| 机械臂 | 5-DOF + 夹爪 | 总线舵机 |
| 雷达 | 国科光芯 COIN-D6 | 串口 230400，0.01-16m |
| 相机 | 安思疆 HP60C | USB，640×480@25fps |
| 音频 | ES8388 编解码器 | I2S，板载麦克风+功放 |
| IMU | BMI088 | 六轴姿态传感器 |

### 项目结构

```
xflz_robot_car1/
├── driver/                    # Phase 1: 硬件驱动层
│   ├── car1_base_driver/      # STM32 板级驱动
│   ├── car1_chassis_controller/ # 底盘控制器
│   └── car1_servo_controller/ # 舵机控制器
├── arm/                       # 机械臂
│   ├── kinematics/            # 正/逆运动学
│   └── car1_arm_controller/   # 机械臂控制器
├── peripherals/               # Phase 2: 传感器集成
│   └── car1_peripherals/      # 雷达、相机、遥控
├── slam/                      # Phase 3: SLAM 建图
│   └── car1_slam/             # slam_toolbox
├── navigation/                # Phase 4: 自主导航
│   └── car1_navigation/       # Nav2 导航
├── app/                       # Phase 5: 智能应用
│   └── car1_app/              # 巡线、追踪、抓取等
├── interaction/               # Phase 6: 语音交互
│   └── car1_speech/           # ASR 语音识别 + TTS 语音合成
├── simulations/               # 仿真
│   ├── car1_description/      # URDF 模型
│   ├── car1_gazebo/           # Gazebo 仿真
│   └── car1_moveit_config/    # MoveIt2 配置
└── bringup/                   # 启动配置
```

---

## 环境准备

### 系统要求

- **操作系统**：Ubuntu 22.04(开发板) / 24.04(宿主机验证版本)
- **ROS2 版本**：Humble(开发板) 或 Jazzy(宿主机验证版本)
- **Gazebo 版本**：Harmonic (gz-sim)

### 基础依赖安装

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装 ROS2 Jazzy（如已安装可跳过）
# 参考：https://docs.ros.org/en/jazzy/Installation.html

# 安装基础工具
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git
```

### 项目依赖安装

```bash
# ROS2 核心包
sudo apt install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools

# 仿真相关（Level 0 需要）
sudo apt install -y \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-trajectory-controller

# 导航相关（Level 3-4 需要）
sudo apt install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-teb-local-planner \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

# 视觉相关（Level 5 需要）
sudo apt install -y \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    python3-opencv

# Python 依赖
pip3 install numpy opencv-python
```

### 编译项目

```bash
# 克隆项目（假设已克隆）
cd ~/xflz_robot_car1_ros2_ws/src

# 编译所有包
cd ~/xflz_robot_car1_ros2_ws
colcon build --symlink-install

# 加载环境变量
source install/setup.bash

# 建议添加到 ~/.bashrc
echo "source ~/xflz_robot_car1_ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 学习路径

### 学习建议

1. **按顺序学习**：从 Level 0 开始，逐步进阶
2. **先仿真后实物**：Level 0 无需硬件，可先熟悉操作
3. **遇到问题先查看"常见问题"章节**
4. **每个 Level 完成后再进入下一个**

---

## Level 0: 仿真验证（无需硬件）

> **目标**：在仿真环境中熟悉机器人模型和控制方法  
> **所需时间**：1-2 小时  
> **硬件要求**：无

### 0.1 URDF 模型可视化

**目标**：在 RViz 中查看机器人 3D 模型，理解机器人结构。

```bash
# 启动 URDF 可视化
ros2 launch car1_description display.launch.py
```

**操作说明**：
1. 启动后会打开两个窗口：
   - **Joint State Publisher GUI**：滑块控制关节
   - **RViz2**：显示机器人 3D 模型

2. 尝试拖动滑块：
   - `arm_joint1` ~ `arm_joint5`：机械臂关节
   - `r_joint`：夹爪开合
   - `wheel_*`：车轮旋转

**验证要点**：
- [ ] 机器人模型完整显示
- [ ] 拖动滑块，模型相应运动
- [ ] TF 坐标系正确显示

**可能的问题**：
- **模型不显示**：检查 RViz 左侧 "RobotModel" 是否勾选
- **滑块无响应**：重启 Joint State Publisher GUI

---

### 0.2 MoveIt2 运动规划

**目标**：使用 MoveIt2 进行机械臂运动规划。

```bash
# 启动 MoveIt2
ros2 launch car1_moveit_config demo.launch.py
```

**操作说明**：
1. RViz 中会出现 **MotionPlanning** 面板
2. 在 **Planning** 标签页：
   - **Planning Group** 选择 `arm`（机械臂）或 `gripper`（夹爪）
   - 拖动末端执行器（橙色球）到目标位置
   - 点击 **Plan & Execute** 执行规划

3. 预设位姿：
   - 在 **Goal State** 下拉菜单选择：
     - `home`：零位
     - `open`：夹爪张开
     - `close`：夹爪闭合

**验证要点**：
- [ ] 能拖动末端执行器
- [ ] 规划路径显示（绿色轨迹）
- [ ] 机械臂按规划运动
- [ ] 预设位姿正常切换

**可能的问题**：
- **规划失败**：目标位置超出工作空间，调整到机械臂可达范围
- **碰撞检测报错**：目标位置与机器人自身碰撞，调整位置

---

### 0.3 Gazebo 物理仿真

**目标**：在 Gazebo 中仿真机器人物理交互。

#### 0.3.1 基础仿真

```bash
# 启动 Gazebo 仿真
ros2 launch car1_gazebo gazebo_arm.launch.py
```

**操作说明**：
1. Gazebo 启动后，机器人会出现在仿真世界中
2. 打开新终端，测试控制器：

```bash
# 检查控制器状态
ros2 control list_controllers

# 应该看到 3 个 active 控制器：
# - joint_state_broadcaster
# - arm_controller
# - gripper_controller
```

3. 测试机械臂运动：

```bash
# 移动机械臂
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5'],
  points: [{
    positions: [0.5, 0.3, 0.3, 0.2, 0.0],
    time_from_start: {sec: 2}
  }]
}" --once

# 回到零位
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5'],
  points: [{
    positions: [0.0, 0.0, 0.0, 0.0, 0.0],
    time_from_start: {sec: 2}
  }]
}" --once
```

4. 测试夹爪：

```bash
# 张开夹爪
ros2 topic pub /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['r_joint'],
  points: [{
    positions: [0.5],
    time_from_start: {sec: 1}
  }]
}" --once

# 闭合夹爪
ros2 topic pub /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['r_joint'],
  points: [{
    positions: [0.0],
    time_from_start: {sec: 1}
  }]
}" --once
```

**验证要点**：
- [ ] Gazebo 正常启动
- [ ] 机器人模型正确加载
- [ ] 控制器状态为 active
- [ ] 机械臂响应指令运动
- [ ] 夹爪开合正常

**可能的问题**：
- **Gazebo 启动慢**：首次启动需下载模型，耐心等待
- **控制器未激活**：等待几秒后重新检查
- **模型不显示**：检查终端错误信息，可能是 mesh 文件路径问题

#### 0.3.2 固定位置抓取演示

**目标**：在 Gazebo 中完成自动抓取演示。

```bash
# 启动抓取演示（默认使用 IK）
ros2 launch car1_gazebo grasp_demo.launch.py

# 或使用手动模式（预设关节角度）
ros2 launch car1_gazebo grasp_demo.launch.py mode:=manual

# 或仅启动环境（用于手动控制）
ros2 launch car1_gazebo grasp_demo.launch.py mode:=none
```

**操作说明**：
- **mode:=ik**（默认）：使用逆运动学自动计算关节角度
- **mode:=manual**：使用预设关节角度
- **mode:=none**：仅启动环境，不执行抓取

**演示流程**：
1. 张开夹爪
2. 移动到抓取位置上方（pre-grasp）
3. 下降到抓取高度
4. 闭合夹爪
5. 提升物体
6. 移动到放置位置
7. 释放物体
8. 回到零位

**验证要点**：
- [ ] 红色方块正确生成
- [ ] 机械臂自动执行抓取流程
- [ ] 夹爪能稳定夹住方块
- [ ] 方块被成功搬运到目标位置

**可能的问题**：
- **抓取失败**：方块位置或夹爪力度问题，多尝试几次
- **IK 无解**：目标位置不可达，检查方块位置参数

#### 0.3.3 视觉引导抓取演示

**目标**：通过深度相机识别物体位置，动态抓取。

```bash
# 启动视觉抓取（默认位置）
ros2 launch car1_gazebo grasp_demo_vision.launch.py

# 指定方块位置
ros2 launch car1_gazebo grasp_demo_vision.launch.py cube_x:=0.22 cube_y:=0.03
```

**分层验证**：

**第 1 层：相机话题**
```bash
# 检查话题
ros2 topic list | grep camera
# 期望输出：/camera/image, /camera/depth, /camera/points

# 检查帧率
ros2 topic hz /camera/image
# 期望：~10-20 Hz
```

**第 2 层：图像可视化**
```bash
# 启动 rviz2，添加 Image display
# 话题选择 /camera/image，应能看到红色方块
# 话题选择 /detection_image，应能看到绿色检测框
```

**第 3 层：物体检测**
```bash
# 检查检测结果
ros2 topic echo /detected_object_pose
# 应输出 PoseStamped，position 接近方块实际位置
```

**第 4 层：动态抓取**
```bash
# 测试不同位置
ros2 launch car1_gazebo grasp_demo_vision.launch.py cube_x:=0.25 cube_y:=0.0
ros2 launch car1_gazebo grasp_demo_vision.launch.py cube_x:=0.22 cube_y:=0.03
ros2 launch car1_gazebo grasp_demo_vision.launch.py cube_x:=0.20 cube_y:=-0.03
```

**验证要点**：
- [ ] 相机图像正常发布
- [ ] 物体检测准确（位置误差 < 1cm）
- [ ] 能抓取默认位置的方块
- [ ] 能抓取不同位置的方块

**可能的问题**：
| 现象 | 检查方法 | 解决方案 |
|------|---------|---------|
| 无相机话题 | `ros2 topic list` | 检查 ros_gz_bridge 是否运行 |
| 图像全黑 | rviz2 查看 | 检查 Gazebo 传感器插件配置 |
| 检测不到物体 | 查看 /camera/image | 调整 HSV 阈值 |
| 坐标跳动 | `ros2 topic hz /camera/depth` | 增大深度采样区域 |
| 抓取超时 | 查看日志 | 确认方块在相机视野内 |

---

### Level 0 小结

完成 Level 0 后，你应该：
- ✅ 理解机器人的结构和自由度
- ✅ 掌握 RViz 和 Gazebo 的基本操作
- ✅ 了解机械臂运动规划的基本概念
- ✅ 能在仿真中完成抓取任务

**下一步**：如果有实际硬件，进入 Level 1；否则可继续深入学习仿真功能。

---

## Level 1: 硬件驱动层

> **目标**：启动实际硬件，验证底盘和机械臂基本控制  
> **所需时间**：1-2 小时  
> **硬件要求**：STM32 控制板、电机、舵机

### 1.1 硬件连接

**连接清单**：
1. STM32 控制板通过 USB 连接到电脑
2. 确认串口设备（udev 按物理 USB 口固定别名）：`ls -l /dev/car1_stm32 /dev/car1_lidar`
3. 给机器人上电（注意电源电压）

**串口权限设置**：
```bash
# 添加当前用户到 dialout 组
sudo usermod -aG dialout $USER

# 重新登录或重启生效
# 或临时授权（一般无需，udev 规则 99-car1-serial.rules 已设 0666）：
sudo chmod 666 /dev/car1_stm32
```

### 1.2 启动驱动层

```bash
# 编译驱动层包
cd ~/xflz_robot_car1_ros2_ws
colcon build --packages-select \
  car1_base_driver_msgs \
  car1_servo_controller_msgs \
  car1_sdk \
  car1_base_driver \
  car1_servo_controller \
  car1_chassis_controller \
  car1_arm_controller \
  car1_bringup

source install/setup.bash

# 启动完整驱动
ros2 launch car1_bringup bringup.launch.py
```

**启动内容**：
- 板级驱动（STM32 串口通信）
- 底盘控制器（运动学 + 里程计 + EKF）
- 舵机控制器（关节状态 + 轨迹执行）

### 1.3 验证底盘控制

**检查话题**：
```bash
# 检查 IMU 数据
ros2 topic echo /car1_base_driver/imu_raw

# 检查里程计
ros2 topic echo /odom_raw
ros2 topic echo /odometry/filtered  # EKF 融合后

# 检查 TF 树
ros2 run tf2_tools view_frames
# 生成 frames.pdf，检查坐标系关系
```

**测试底盘运动**：
```bash
# 前进
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

# 后退
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.1}}" --once

# 左转
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once

# 右转
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -0.5}}" --once

# 左平移（麦轮特有）
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.1}}" --once

# 停止
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{}" --once
```

**验证要点**：
- [ ] 串口连接成功（无报错）
- [ ] IMU 数据正常发布（~100 Hz）
- [ ] 里程计数据正常发布（~50 Hz）
- [ ] 底盘响应 cmd_vel 指令
- [ ] 麦轮全向运动正常

**可能的问题**：
- **串口打不开**：检查设备号和权限
- **IMU 数据异常**：检查 STM32 固件版本
- **底盘不动**：检查电机接线和电源
- **里程计漂移**：需要校准轮径和轮距参数

### 1.4 启动机械臂控制器

```bash
# 新终端，启动机械臂控制器
ros2 launch car1_arm_controller arm_controller.launch.py
```

**测试机械臂**：
```bash
# 回到零位
ros2 service call /arm_controller/go_home std_srvs/srv/Trigger

# 移动到指定关节角度
ros2 topic pub /arm_controller/joint_command sensor_msgs/msg/JointState "{
  name: ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5'],
  position: [0.0, -1.0, -1.0, -1.0, 0.0],
  effort: [2.0]
}" --once

# 张开夹爪
ros2 service call /arm_controller/set_gripper std_srvs/srv/SetBool "{data: true}"

# 闭合夹爪
ros2 service call /arm_controller/set_gripper std_srvs/srv/SetBool "{data: false}"
```

**运行演示**：
```bash
# 自动演示（抓取流程）
ros2 run car1_arm_controller arm_demo
```

**验证要点**：
- [ ] 舵机控制器正常启动
- [ ] 关节状态正常发布（`ros2 topic echo /joint_states`）
- [ ] 机械臂响应关节指令
- [ ] 夹爪开合正常
- [ ] 演示脚本完整执行

**可能的问题**：
- **舵机不动**：检查舵机 ID 和供电
- **舵机抖动**：检查舵机负载和电源电流
- **IK 无解**：目标位置超出工作空间

### 1.5 机械臂舵机零位校准（可选）

> **是否需要校准**：如果你的机械臂上电后姿态正常，IK 求解准确，**可以跳过这一步**。只有在发现以下问题时才需要校准：
> - 上电后机械臂姿态明显歪斜（某个关节偏了 5° 以上）
> - IK 求解后实际位置与预期不符
> - 抓取任务经常失败

#### 什么是舵机零位校准？

**问题**：舵机安装时存在机械误差，导致"理论零位"（PWM 1500）与"实际零位"不一致。

**例子**：
- 理论零位：PWM 1500（所有舵机上电后的默认位置）
- 实际零位：由于机械误差，可能需要 PWM 1450 才能真正"垂直"
- offset = 1450 - 1500 = -50

**解决方案**：记录每个舵机的 offset，软件层自动补偿。

#### 校准步骤（3 步完成）

**Step 1：启动校准工具**

```bash
cd ~/ros2_ws/src/xflz_robot_car1
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select calibration
source install/setup.bash

# 启动舵机校准（会启动板级驱动 + 校准节点）
ros2 launch calibration servo_calib.launch.py
```

**Step 2：移动到理论零位并微调**

```bash
# 所有舵机移动到 PWM 1500（理论零位）
ros2 service call /calibrate_servo/move_to_center std_srvs/srv/Trigger
```

观察机械臂姿态，如果某个关节不在理想位置，通过命令行微调：

```bash
# 例如：调整 servo 1（大臂）到垂直位置
# 如果 1500 时不垂直，尝试 1450
ros2 topic pub /car1_base_driver/bus_servo/set_position car1_base_driver_msgs/msg/ServosPosition "{
  duration: 1.0,
  position: [{id: 1, position: 1450}]
}" --once

# 继续微调，直到目测垂直
ros2 topic pub /car1_base_driver/bus_servo/set_position car1_base_driver_msgs/msg/ServosPosition "{
  duration: 1.0,
  position: [{id: 1, position: 1455}]
}" --once

# 依次调整其他舵机...
ros2 topic pub /car1_base_driver/bus_servo/set_position car1_base_driver_msgs/msg/ServosPosition "{
  duration: 1.0,
  position: [{id: 2, position: 1530}]
}" --once
```

**Step 3：保存 offset**

```bash
# 保存当前位置作为 offset
ros2 service call /calibrate_servo/save_offsets std_srvs/srv/Trigger
```

**保存后会生成**：
- `calibration/config/servo_offsets.yaml` 配置文件
- 下次启动 `car1_base_driver` 时会自动加载并应用这些 offset

**验证**：

```bash
# 重启驱动以加载新的 offset
# Ctrl+C 停止当前的 servo_calib.launch.py
ros2 launch car1_bringup bringup.launch.py

# 新终端，启动机械臂控制器
ros2 launch car1_arm_controller arm_controller.launch.py

# 回到零位，确认姿态正确
ros2 service call /arm_controller/go_home std_srvs/srv/Trigger
```

#### 常见问题

**Q: 为什么不写入舵机 EEPROM？**
- EEPROM 写入次数有限（约 10 万次），频繁写入会损坏舵机
- 软件层补偿更安全，出问题只需修改配置文件
- 参考 lander_pi 的实现，也是用软件层补偿

**Q: 校准后重启机器人，offset 会丢失吗？**
- 不会。offset 保存在 `servo_offsets.yaml` 配置文件中
- `car1_base_driver` 启动时会自动加载并应用这些 offset

**Q: 如何查看当前的 offset 值？**
```bash
# 查看配置文件
cat ~/ros2_ws/src/xflz_robot_car1/calibration/config/servo_offsets.yaml
```

**Q: 如何手动修改 offset？**
```bash
# 直接编辑配置文件
nano ~/ros2_ws/src/xflz_robot_car1/calibration/config/servo_offsets.yaml

# 修改后重启 car1_base_driver 生效
ros2 launch car1_bringup bringup.launch.py
```

**Q: 校准后 IK 求解还是不准怎么办？**
- 检查 URDF 模型中的连杆长度参数是否正确
- 检查舵机 ID 是否与 URDF 中的关节顺序一致
- 确认校准时定义的零位姿态与 URDF 中的零位定义一致

---

### 1.6 机械臂动作录制与回放

> **目标**：使用交互工具调试机械臂动作，录制为 YAML 文件，通过 ROS2 回放  
> **所需时间**：30-60 分钟  
> **硬件要求**：机械臂舵机正常工作

#### 概述

机械臂动作控制分为两个阶段：
1. **录制阶段**：通过 `test_servo_direct.py` 交互式调试，将动作导出为 YAML 文件
2. **回放阶段**：通过 `action_player` ROS2 节点回放 YAML 动作文件

已有的动作文件（位于 `arm/car1_arm_controller/config/actions/`）：

| 文件 | 说明 |
|------|------|
| `home.yaml` | 所有关节回中位（1500） |
| `work_ready.yaml` | 工作准备姿态（臂前伸，夹爪张开） |
| `pick_and_place.yaml` | 完整抓取放置动作 |

#### Step 1：启动交互调试工具

```bash
# 先确保 bringup 没有运行（避免串口冲突）
# 确认串口权限（udev 别名 /dev/car1_stm32 通常已 0666）
sudo chmod 666 /dev/car1_stm32

# 启动交互工具
python3 ~/ros2_ws/test_servo_direct.py
```

选择 `1` 进入交互模式：

```
选择模式:
  1. 交互模式 - 手动输入PWM值调试
  2. 回放模式 - 执行 ACTIONS 列表

输入 1 或 2: 1
```

#### Step 2：交互模式命令说明

| 命令 | 说明 | 示例 |
|------|------|------|
| `1500 1100 2200 1850 1500` | 同时控制 5 个关节（J0~J4）PWM | 直接输入 5 个数字 |
| `g <pwm>` | 控制夹爪（ID=5） | `g 1800`（闭合） / `g 1000`（张开） |
| `s <id> <pwm>` | 单独控制某个舵机 | `s 1 1050`（只动 J1） |
| `home` | 所有关节回中位 1500 | |
| `work` | 进入默认工作姿态 | |
| `list` | 查看已保存的动作 | |
| `del <n>` | 删除第 n 个动作 | `del 3` |
| `play` | 回放所有已保存的动作 | |
| `yaml` | 导出为 YAML 文件 | |
| `quit` | 退出 | |

舵机 PWM 范围：500~2500，中位 1500。

#### Step 3：录制动作（以抓取放置为例）

完整操作示例：

```
> work
  保存? 输入名称(回车跳过): pre_grasp

> g 1000
  保存? 输入名称(回车跳过): gripper_open

> 1500 2000 1800 2020 1500
  保存? 输入名称(回车跳过): grasp_down

> g 1500
  保存? 输入名称(回车跳过): gripper_close

> 2000 1800 2000 2000 1500
  保存? 输入名称(回车跳过): lift

> g 1000
  保存? 输入名称(回车跳过): gripper_open

> 1500 1100 2200 1850 1500
  保存? 输入名称(回车跳过): back_work
```

**调试技巧**：

- 每步执行后观察机械臂姿态，不满意则**直接回车跳过**（不保存），重新输入 PWM 值
- 用 `s <id> <pwm>` 微调单个关节，找到合适位置后再输入完整的 5 关节值保存
- 用 `list` 随时查看已保存的动作列表
- 用 `del <n>` 删除录错的步骤
- 用 `play` 完整回放一次，确认动作连贯

```
# 微调示例：发现 J1 还不够低
> s 1 2050
# 观察到位后，输入完整值保存
> 1500 2050 1800 2020 1500
  保存? 输入名称(回车跳过): grasp_down
```

#### Step 4：导出 YAML 文件

确认动作无误后，输入 `yaml`：

```
> yaml
  动作组名称 (如 pick_front): pick_and_place
  描述 (回车跳过): 完整抓取放置动作

  已导出到: .../config/actions/pick_and_place.yaml
  动作步数: 7

  使用方法:
    ros2 run car1_arm_controller action_player --ros-args -p action_file:=pick_and_place
```

导出的 YAML 文件格式：

```yaml
name: pick_and_place
description: 完整抓取放置动作
actions:
- name: pre_grasp
  duration: 2.0
  servos: {0: 1500, 1: 1100, 2: 2200, 3: 1850, 4: 1500}
- name: gripper_open
  duration: 1.0
  servos: {5: 1000}
- name: grasp_down
  duration: 2.0
  servos: {0: 1500, 1: 2000, 2: 1800, 3: 2020, 4: 1500}
# ... 后续步骤
```

#### Step 5：通过 ROS2 回放

```bash
# 编译（安装新的 YAML 文件）
cd ~/ros2_ws
colcon build --packages-select car1_arm_controller
source install/setup.bash

# 启动 bringup（提供串口通信）
ros2 launch car1_bringup bringup.launch.py

# 另一个终端，回放动作
ros2 launch car1_arm_controller action_player.launch.py action:=pick_and_place
```

**回放模式选项**：

```bash
# 普通模式（自动连续执行）
ros2 launch car1_arm_controller action_player.launch.py action:=pick_and_place

# 单步模式（每步等待确认）
ros2 launch car1_arm_controller action_player.launch.py action:=pick_and_place step_mode:=true

# 循环模式
ros2 launch car1_arm_controller action_player.launch.py action:=pick_and_place loop:=true

# 不自动开始（手动触发）
ros2 launch car1_arm_controller action_player.launch.py action:=pick_and_place auto_play:=false
```

**运行时控制**（另开终端）：

```bash
# 紧急停止
ros2 service call /action_player/stop std_srvs/srv/Trigger

# 手动触发播放（auto_play:=false 时使用）
ros2 service call /action_player/play std_srvs/srv/Trigger

# 单步推进（step_mode 时使用）
ros2 service call /action_player/next std_srvs/srv/Trigger
```

#### 验证要点

- [ ] `test_servo_direct.py` 能正常连接串口并控制舵机
- [ ] 录制的动作可通过 `play` 完整回放
- [ ] 导出的 YAML 文件格式正确
- [ ] `action_player` 节点正常启动并执行动作
- [ ] 紧急停止功能有效
- [ ] 单步模式正常工作

#### 常见问题

**Q: 串口打不开**
```bash
# 确认 bringup 已停止（会占用串口）
# 检查设备
ls -l /dev/car1_stm32
# 授权（通常已 0666）
sudo chmod 666 /dev/car1_stm32
```

**Q: 舵机不动**
- 检查电源是否接通
- 检查舵机 ID 是否正确（0~4 关节，5 夹爪）
- 用 `s 0 1500` 测试单个舵机

**Q: action_player 找不到 YAML 文件**
- 确认已重新编译：`colcon build --packages-select car1_arm_controller`
- 确认已 source：`source install/setup.bash`
- 或使用绝对路径：`action_file:=/full/path/to/file.yaml`

**Q: 动作太快/太慢**
- 直接编辑 YAML 文件中每步的 `duration` 值（单位：秒）
- 建议：大幅度运动 2.0~3.0s，小幅度 1.0~1.5s，夹爪 0.5~1.0s

---

### 1.7 交互式 IK 求解测试

> **目标**：交互式输入目标位置，实时计算 IK 解并可选发送到实机  
> **所需时间**：15 分钟  
> **硬件要求**：机械臂舵机（可选，纯计算模式无需硬件）

`interactive_ik_test.py` 是一个交互式 IK 测试工具，支持两种模式：

- **ROS2 模式**：通过 `/car1_base_driver/bus_servo/set_position` 话题发送指令（需先启动 bringup）
- **直连串口模式（`--direct`）**：直接通过串口发送指令，绕开 ROS2，适合底层调试

#### 启动方式

```bash
cd ~/ros2_ws/src/xflz_robot_car1/arm/kinematics/kinematics

# ROS2 模式（需先 ros2 launch car1_bringup bringup.launch.py）
python3 interactive_ik_test.py

# 直连串口模式（需先停掉 bringup，避免串口冲突）
sudo chmod 666 /dev/car1_stm32
python3 interactive_ik_test.py --direct
```

#### 操作流程

1. **输入目标 TCP 位置**：
   ```
   输入目标 TCP 位置 (输入 'q' 退出)
   ----------------------------------------------------------------------
   X 坐标 (米) [默认: 0.25]: 0.22
   Y 坐标 (米) [默认: 0.0]: 0.03
   Z 坐标 (米) [默认: 0.15]: 0.12
   俯仰角 (度) [默认: 0]: -90
   ```

2. **查看 IK 求解结果**：
   - 显示关节角度（弧度/角度）
   - 显示 PWM 值（发送到舵机）
   - FK 验证：计算位置 vs 目标位置的误差

3. **发送到实机（可选）**：
   ```
   是否发送到实机? (y/n) [n]: y
   运动时长 (秒) [默认: 2.0]: 3.0
   ```

4. **读取实际 PWM（验证）**：
   ```
   是否读取舵机实际 PWM? (y/n) [y]: y
   等待运动完成 (3.3s)...
   
   读取舵机实际位置:
   ----------------------------------------------------------------------
     关节          ID    目标PWM   实际PWM   误差
     J1-底座旋转    0     1523      1525      +2
     J2-大臂       1     1850      1848      -2
     ...
   ```

#### 功能特点

| 功能 | 说明 |
|------|------|
| **IK 求解** | 解析解 IK，快速计算（~0.05ms） |
| **FK 验证** | 自动计算 FK 并显示误差（应 < 1.5mm） |
| **限位检查** | 自动检查 PWM 是否在 500-2500 安全范围 |
| **PWM 转换** | 直接显示发送到舵机的 PWM 值 |
| **闭环验证** | 发送后读取实际 PWM，对比目标值 |
| **两种模式** | ROS2 模式（需 bringup）/ 直连串口模式（独立） |

#### 坐标系说明

```
base_link 原点（ROS 右手坐标系）:
  +X: 向前（机器人正前方）
  +Y: 向左
  +Z: 向上

tcp_link 位置:
  = arm_link5 位置 + [0, 0, 0.0762]m

指尖位置:
  = tcp_link 位置 - [0, 0, 0.033]m

俯仰角 (pitch):
  0°: 水平向前
  -90°: 垂直向下（抓取姿态）
  +90°: 垂直向上
```

#### 测试用例

**测试 1：中位姿态验证**
```
X: 0.25
Y: 0.0
Z: 0.15
俯仰角: 0
```
期望：机械臂水平前伸，FK 误差 < 1.5mm

**测试 2：抓取姿态**
```
X: 0.22
Y: 0.03
Z: 0.10
俯仰角: -90
```
期望：机械臂向下，适合抓取桌面物体

**测试 3：偏侧位置**
```
X: 0.20
Y: -0.05
Z: 0.12
俯仰角: -45
```
期望：测试 Y 轴偏移和斜向俯仰

#### 验证要点

- [x] IK 求解成功（误差 < 1.5mm）
- [x] 计算结果在舵机安全范围内（PWM 500-2500）
- [x] （实机模式）发送后机械臂正确到达目标位置（已通过直连串口模式闭环验证）
- [x] （实机模式）实际 PWM 与目标 PWM 误差 < 30（系统性偏移见下方说明）

#### 实测系统性偏移（2026-06-12 验证发现）

在直连串口模式下做闭环验证（发送目标 PWM → 读回实际 PWM）时,发现两个关节存在稳定的系统性偏移:

| 关节 | 目标→实际 PWM 偏差 | 角度偏差 | 末端位置影响 |
|------|------------------|---------|-------------|
| ID0 J1-底座旋转 | ±1 | ~0° | 可忽略 |
| **ID1 J2-大臂** | **+22~24** | **+4°** | 5~8mm |
| **ID2 J3-小臂** | **+13~14** | **+2.3°** | 2~3mm |
| ID3 J4-腕部 | 0 | 0° | 无 |
| ID4 J5-末端旋转 | 0 | 0° | 无 |

**原因**：机械装配偏差 + 舵机中位校准 + 负载稳态误差。这是物理特性,不是代码问题。

**处理方式**：
- ✅ **推荐**：通过 Level 4 视觉闭环自动补偿（相机看到的物体位置反映真实姿态,无需手动矫正）
- ⚠️ **可选**：在 `transform.py` 添加 per-servo offset 做软件补偿
- ⚠️ **可选**：用舵机协议 `#001PSCK!` 命令重新校准舵机硬件中位

对于一般抓取任务（30~50mm 物体）,5~8mm 末端误差在夹爪容错范围内,**不必矫正即可进入 Level 4**。

#### 常见问题

**Q: IK 求解失败，提示 "target unreachable"**
- 目标位置超出工作空间
- 检查距离：最大伸展 ~0.4m，最小距离 ~0.1m
- 减小 X 或增大 Z 坐标

**Q: FK 验证误差过大（> 5mm）**
- 检查 URDF 模型参数是否正确
- 检查 transform.py 中的转换参数
- 可能是舵机零位偏移，需要校准（参考 1.5 节）

**Q: 直连串口模式无法打开串口**
```bash
# 确保 bringup 已停止
pkill -f bringup

# 授权串口（udev 别名 car1_stm32 通常已 0666）
sudo chmod 666 /dev/car1_stm32

# 检查设备
ls -l /dev/car1_stm32
```

**Q: ROS2 模式发送后无响应**
- 确认 bringup 正在运行：`ros2 node list | grep base_driver`
- 检查话题：`ros2 topic echo /car1_base_driver/bus_servo/set_position`

**Q: 读取实际 PWM 失败或数据错位**

**现象**：ROS2 模式下读取 PWM 时，数值明显错位（如 ID1 显示 1500，ID2 显示 1870，实际应该是 ID1=1870, ID2=2094）

**原因**：`board_sdk` 的消息队列机制导致：
- 发送"读位置"命令后，串口缓冲区可能有其他消息（IMU、电池电压、其他舵机响应）
- ROS2 服务调用时，响应被队列中的消息干扰，读到的是错位数据
- 这是 `car1_base_driver` 串口通信架构的已知限制

**解决方案**：
- ✅ **推荐**：使用 `--direct` 直连串口模式进行验证
  - 直连模式会先清空串口缓冲区，然后只接收 `func=0x05` 的响应包
  - 成功率 >95%，数据准确
  
- ⚠️ **不推荐**：ROS2 模式下读取 PWM
  - 仅用于发送指令，不要依赖读取功能
  - 如必须使用 ROS2 模式，只关注 FK 验证误差（< 1.5mm），忽略 PWM 读取结果

**验证建议**：
```bash
# 停掉 bringup
pkill -f bringup

# 使用直连串口模式
sudo chmod 666 /dev/car1_stm32
python3 interactive_ik_test.py --direct
```

#### Level 3 完整验证

此工具是 Level 3（IK 分步抓取）的核心验证工具，覆盖以下验证点：

1. **Transform 层验证** — PWM ↔ 角度转换准确性
2. **IK 闭环验证** — 目标 → IK → PWM → 发送 → 读取 → 对比
3. **已知点验证** — 输入物理标记坐标，人工目测到达
4. **FK 验证** — 计算位置与目标位置误差 < 1.5mm

完成验证后，可进入 Level 4（视觉 + IK 抓取）。

---

## Level 2: 传感器集成

> **目标**：启动雷达和深度相机，实现键盘遥控  
> **所需时间**：30 分钟  
> **硬件要求**：COIN-D6 雷达、HP60C 深度相机

### 2.1 依赖安装

```bash
# 雷达驱动（m1ct_d2）和相机驱动（ascamera）需单独安装
# 假设已安装在工作空间中
```

### 2.2 启动传感器

```bash
# 编译传感器包
colcon build --packages-select car1_peripherals
source install/setup.bash

# 启动所有传感器
ros2 launch car1_peripherals peripherals.launch.py
```

**验证雷达**：
```bash
# 检查雷达数据
ros2 topic echo /scan
ros2 topic hz /scan  # 应为 ~10 Hz

# 在 RViz 中可视化
rviz2
# 添加 LaserScan display，话题选择 /scan
```

**验证深度相机**：
```bash
# 检查相机话题
ros2 topic list | grep depth_cam
# 应有：/depth_cam/rgb/image_raw, /depth_cam/depth/image_raw, /depth_cam/depth/points

# 查看图像
ros2 run rqt_image_view rqt_image_view
# 选择 /depth_cam/rgb/image_raw
```

**验证要点**：
- [ ] 雷达数据正常发布
- [ ] 雷达扫描范围正确（0.01-16m）
- [ ] 深度相机 RGB 图像正常
- [ ] 深度图正常
- [ ] 点云数据正常

**可能的问题**：
- **雷达无数据**：检查串口设备和波特率（230400）
- **相机无图像**：检查 USB 连接和驱动
- **点云数据异常**：检查相机内参配置

### 2.2.1 单独验证 HP60C 深度相机（厂商原始案例）

> 上面的 `peripherals.launch.py` 是项目集成入口。如果只想**单独验证相机硬件本身**，可直接用厂商自带的 ascamera 案例启动，这条路径已实测可用。

**首次使用前：配置 udev 权限 + 编译**

```bash
# 1. USB 设备授权（首次必做）
cd ~/ros2_ws/src/xflz_eai_car1/examples/camera/HP60C/src/ascamera/scripts
sudo ./create_udev_rules.sh

# 2. 编译并 source
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ascamera
source install/setup.bash
```

> **编译说明**：原始 CMakeLists 只安装可执行文件、不安装厂商 SDK 动态库，启动会报
> `libAngstrongCameraSdk.so: cannot open shared object file`（退出码 127）。
> 已在 `CMakeLists.txt` 修复：把 `libs/lib/<arch>/*.so` 一并装进可执行文件同目录，
> 并给 `ascamera_node` 设 `INSTALL_RPATH "$ORIGIN"`。重新编译即可正常启动。

**启动相机**

```bash
ros2 launch ascamera hp60c.launch.py
```

启动成功的标志：日志依次出现 `attached` → `camera opened` → `start streaming`。
末尾出现 `stop streaming` 是**正常**的——相机按需推流，没有订阅者时自动停流省资源，
一旦有节点订阅（如下面的 rqt）会自动重新 `start streaming`。
中途的 `getMjpegSize ... failed` 紧跟着 `mjpeg size:640x480`，走了 fallback，可忽略。

**用 rqt_image_view 查看图像（已验证可用）**

```bash
# 另开一个终端
cd ~/ros2_ws
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在 rqt 窗口左上角的话题下拉框中选择：

| 话题 | 内容 |
|------|------|
| `/ascamera_hp60c/camera_publisher/rgb0/image` | RGB 彩色图 |
| `/ascamera_hp60c/camera_publisher/depth0/image_raw` | 深度图（16UC1，单位 mm，0=无效） |

> **话题命名空间是 `ascamera_hp60c`**（不是项目集成版的 `/depth_cam/...`）。
> 厂商驱动靠话题名里的 `rgb0`/`depth0` 子串决定开哪路数据流，**切勿 remap 话题名**。

**验证要点**：
- [ ] `ros2 launch ascamera hp60c.launch.py` 正常启动（出现 `start streaming`）
- [ ] rqt_image_view 能显示 RGB 图像
- [ ] rqt_image_view 能显示深度图（近处灰、远处亮，近距 <0.5m 有空洞属正常硬件盲区）

### 2.2.2 HP60C 深度相机 Web 查看器（局域网浏览器）

> 位于 `test/camera_web_viewer/`。它把 HP60C 的 RGB / 深度 / 点云通过浏览器实时显示，
> 局域网内任何设备（手机、平板、另一台电脑）打开网页即可查看，无需在本机装 RViz/rqt。
> 支持 2D 双画面、3D 点云、深度点击测距。

**功能一览**：

| 功能 | 说明 |
|------|------|
| 2D 视图 | RGB（左）+ Depth（右）并排实时显示，各自带 FPS |
| 3D 视图 | 点云渲染（Three.js），鼠标可旋转/缩放，支持重置视角、调点大小 |
| 深度测距 | 点击 Depth 画面任意点，显示该像素 `X / Y / 深度(m)`，并随画面持续刷新；右键清除 |
| 深度可视化 | TURBO 伪彩色，量程 0–5m |
| FPS 监控 | 侧边栏显示 RGB / Depth / 点云 / Web 接收四路帧率 |

**一次性依赖安装**（Web 框架）：

```bash
# 注意：要装到运行 ROS 节点的同一个 Python。launch 入口脚本使用 /usr/bin/python3，
# 所以这里用系统 python 安装，避免启动时报 ModuleNotFoundError: flask
/usr/bin/python3 -m pip install flask flask-socketio simple-websocket
```

> 前端依赖（three.js r128、OrbitControls、pako、socket.io）已随包打包在
> `static/libs/`，局域网内**无需联网**即可加载。

**编译**：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_web_viewer --symlink-install
source install/setup.bash
```

**启动**（一条命令同时拉起相机驱动 + Web 服务）：

```bash
ros2 launch camera_web_viewer camera_viewer.launch.py
```

启动后日志会打印监听地址，例如：

```
Web server: http://0.0.0.0:8000  (LAN accessible)
 * Running on http://10.1.134.42:8000      ← 当前网卡 IP（会随网络变化）
```

`0.0.0.0` 表示监听所有网卡，局域网其他设备用**本机当前 IP** 访问即可：

```
http://<本机IP>:8000
```

本机 IP 用 `hostname -I` 或 `ip addr` 查询。

> **相机话题**：launch 里启用了 `color_pcl:=True`（点云带 RGB 上色），
> Web 服务订阅的是相机**原生话题**（带 `camera_publisher` 段，切勿 remap）：
> ```
> /ascamera_hp60c/camera_publisher/rgb0/image
> /ascamera_hp60c/camera_publisher/depth0/image_raw
> /ascamera_hp60c/camera_publisher/depth0/points
> ```

**验证要点**：

- [ ] 浏览器打开 `http://<本机IP>:8000`，左 RGB 右 Depth 实时显示
- [ ] 点击 Depth 画面，标签显示 `X / Y / 深度`，且数值随画面刷新
- [ ] 深度无效区域（太近/太远/空洞）标签显示红色"无效"
- [ ] 切到 3D 标签能看到点云，鼠标可旋转缩放
- [ ] 侧边栏四路 FPS 正常跳动

**常见问题**：

| 现象 | 原因 | 解决方案 |
|------|------|---------|
| `Address already in use` / `Port 8000 is in use` | 上次的服务进程没退干净 | `ss -ltnp \| grep :8000` 查 PID，`kill -9 <pid>` |
| `ModuleNotFoundError: flask` | flask 没装到 `/usr/bin/python3` | 用 `/usr/bin/python3 -m pip install flask flask-socketio simple-websocket` |
| 网页打开但无画面 | 相机话题没数据 / 订阅错话题 | `ros2 topic hz /ascamera_hp60c/camera_publisher/rgb0/image`（等≥10s）确认有数据 |
| 延迟好几秒 | QoS 队列堆积 | 已用 sensor_data QoS（best_effort + keep_last 1）修复；如自行改过订阅需保持 |
| 深度值不刷新 | — | 已实现"粘性追踪点"，点击后每帧更新；右键可清除 |
| 局域网其他设备打不开 | 防火墙挡了 8000 端口 | 放行 8000：`sudo ufw allow 8000`（如启用了 ufw） |

> **停止服务**：`Ctrl+C`，或后台运行时 `pkill -f "launch camera_web_viewer"`。

> **另一个 Web 工具**：`test/car1_web_server/` 是纯静态的 URDF 浏览器查看器
> （`python3 server_cors.py [端口]` 启动，默认 8080，访问 `http://<IP>:端口/viewer/`），
> 用于在浏览器里查看机器人 URDF 模型，与相机查看器互不相关。

### 2.3 键盘遥控

```bash
# 启动键盘遥控
ros2 launch car1_peripherals teleop_key.launch.py
```

**控制键位**：
- `w` / `s` — 前进 / 后退
- `a` / `d` — 左转 / 右转
- `q` / `e` — 左平移 / 右平移（麦轮）
- `空格` — 停止
- `Ctrl+C` — 退出

**验证要点**：
- [ ] 键盘控制响应灵敏
- [ ] 麦轮全向运动正常
- [ ] 速度平滑无突变

---

## Level 3: SLAM 建图

> **目标**：使用雷达进行 SLAM 建图  
> **所需时间**：30 分钟  
> **硬件要求**：雷达 + 底盘  
> **前置条件**：必须先完成底盘校准（见下方 3.0 节）

### 3.0 底盘里程计校准（SLAM 前必做）

> **为什么必须先校准**：不准确的里程计会导致 SLAM 地图变形、重影、回环检测失败。这是 SLAM 建图的前置条件，必须在建图前完成。

> 🧭 **SLAM 前必做校准清单**（缺一不可，详见对应小节）：
> 1. **线速度标定**（3.0.1）→ `linear_correction_factor`
> 2. **角速度标定**（3.0.2）→ `angular_correction_factor`
> 3. **IMU 校准与航向验证**（3.10）→ 麦轮原地转打滑严重，**航向完全靠 IMU**，IMU/EKF 不正确会直接导致地图航向漂移（曾漂 162°）。**务必在建图前先看 3.10 完成验证，不要看完 3.4 就直接建图。**
>
> 校准系数文件统一在：`~/ros2_ws/src/xflz_eai_car1/driver/controller/config/calibrate_params.yaml`

> 🔴 **重要前置（2026-06-23 排查确认）**：校准前必须确保底层速度单位正确，否则标的是错的。
> 历史上曾出现「小车超速 3.98 倍 / 指令转 1 圈实际转 3.75 圈」的问题，**根因不是校准，而是固件与 ROS 的速度单位错配**：
> - **ROS 端** `mecanum.py` 原本经 `speed_to_rps` 发 **rps**，但 STM32 固件 raw 逐轮模式期望 **轮缘线速度 m/s**。
> - **固件端** `kinematics.h` 的 `MEC_WHEEL_DIAMETER` 原为 0.08（真实 0.097），编码器反馈换算偏差。
> - **已修复**：`mecanum.py` 直接发 m/s；固件 `kinematics.h` 轮径改 0.097、轮距/轴距改 0.1946/0.2117（需 Windows Keil 重新编译烧录）。
> - 修复后残留误差仅约 6%，正是 `correction_factor` 该补偿的量级。
>
> **如果你的小车出现「速度异常快 / 转好几圈才停」，先检查上述两处是否已修复，不要急着调校准系数。**
> 直连下位机验证工具见 `calibration/scripts/chassis_serial_test.py`（绕过 ROS，复刻 board_sdk 协议：查表 CRC8、开串口前 rts/dtr=False、先发 set_motor_type 初始化）。

> ⚠️ **rqt_reconfigure 不是必需的**。若环境未装（`Package 'rqt_reconfigure' not found`），
> 直接用 `ros2 param set` 命令行设置参数即可，效果完全一样。本节用命令行方式。

#### 3.0.1 线速度校准

**原理**：让机器人前进一段，对比里程计报告值与尺子实测值，得出修正系数（乘在 odom 输出上，让 odom 报告匹配真实距离）。

**校准脚本说明（2026-09-07 更新）**：
- `calibrate_linear` 同时订阅 `/odom_raw` 与 `/odometry/filtered`(EKF)，判停/报告源可切换。
- 支持 `direction` 参数：`1.0`=向前，`-1.0`=向后（默认向前）。
- 支持 `use_ekf` 参数：`false`=用 odom_raw 判停，`true`=用 EKF 判停（默认 `false`）。
- 结束时同时打印 `finish: odom_raw=Xm EKF=Ym target=Zm`，方便用尺子量物理距离后做三层对比。

**准备工作**：
1. 在车头正下方某点贴胶带/粉笔做起点标记
2. 确保地面平整，轮子不打滑（瓷砖偶有打滑，属正常随机误差）

**操作步骤**：

```bash
# 终端1：启动底盘
ros2 launch car1_bringup bringup.launch.py

# 终端2：启动直线校准节点
ros2 run calibration calibrate_linear

# 终端3：用命令行设参数并触发
ros2 param set /calibrate_linear test_distance 1.0
ros2 param set /calibrate_linear speed 0.2
ros2 param set /calibrate_linear direction 1.0      # 1.0=向前, -1.0=向后
ros2 param set /calibrate_linear use_ekf false       # false=odom_raw判停, true=EKF判停
ros2 param set /calibrate_linear odom_linear_scale_correction 1.0
ros2 param set /calibrate_linear start_test true
```

小车自动前进，odom 判断走够 1m 就停，终端2 打印 `finish: odom_raw=Xm EKF=Ym target=Zm`。

**用尺子量实际距离**，然后计算修正系数：

```
linear_correction_factor = 实际距离 / odom报告距离
```

> ⚠️ **方向别搞反**：系数 = 真实 / odom报告（不是目标/实际）。
> 例：odom 报告 0.978m，尺子实测 0.94m → 系数 = 0.94 / 0.978 ≈ 0.961。
> 含义：odom 多报了，要乘 0.961 把数字缩小到匹配真实。

**反复验证**：

```bash
# 填入算出的系数再测一次
ros2 param set /calibrate_linear odom_linear_scale_correction 0.961
ros2 param set /calibrate_linear start_test true
# 这次尺子实测应接近 1.0m（误差 <3cm 合格）
```

> 实测案例：系数 0.961 验证后实际走 0.995m，误差 0.5%。

#### 3.0.1.1 速度依赖性（重要！）

**实测发现**：标定速度影响校准结果，**必须用建图实际使用的速度标定**。

实测案例（轮径 0.0935m，轮距 0.216m 实测值更新后）：

| 标定速度 | odom 报告 | 实际距离 | 偏差 | 推荐系数 |
|---------|---------|---------|------|---------|
| 0.2 m/s | 0.993 m | 1.10 m | +10% | 1.108 |
| 0.3 m/s | 0.993 m | 1.04 m | +4% | 1.04 |
| 0.5 m/s | （未测） | （未测） | — | — |

**原因分析**：
- ROS odom 是**开环积分** cmd_vel（不读编码器），见 `odom_publisher_node.py:_odom_loop`
- 不同速度下加速段/减速段占比不同，引入非线性误差：
  - 低速时加速段占比大，加速期间命令已发但实际未到速 → odom 多报
  - 高速时减速段惯性大，停车后还会滑行 → 实际多走
- 二者方向相反，部分抵消，导致系数随速度变化

**推荐做法**：
1. **决定你建图常用速度**（推荐 0.3 m/s，平衡建图质量和效率）
2. **用该速度标定**，得到对应系数
3. **建图时保持同样速度**，标定才有意义

> 速度依赖性是开环 odom 的固有局限。长期修复方案：固件上报编码器实测轮速，
> ROS 端 odom 改用编码器反馈而非命令积分。当前 ~5% 残余误差对 SLAM 足够。

#### 3.0.1.2 完整验证流程（带几何参数修改）

如果你刚修改了麦轮几何参数（轮径/轮距/轴距），旧标定系数**全部失效**，必须重做。

**完整步骤**：

```bash
# Step 1: 确认参数已更新
# - ROS 端 mecanum.py / odom_publisher_node.py 改了几何参数
# - 固件 kinematics.h MEC_ACLE_BASE / MEC_WHEEL_DIAMETER 改了且重烧
# - calibrate_params.yaml 系数先临时设为 1.0（清空旧标定）

# Step 2: 编译并重启 bringup
cd ~/ros2_ws
colcon build --packages-select controller car1_base_driver --symlink-install
source install/setup.bash
ros2 launch car1_bringup bringup.launch.py

# Step 3: 确认系数加载
# 另开终端
ros2 param get /odom_publisher linear_correction_factor
# 应显示 1.0（未标定状态）

# Step 4: 启动标定节点
ros2 run calibration calibrate_linear

# Step 5: 设置参数（用你建图速度）
ros2 param set /calibrate_linear test_distance 1.0
ros2 param set /calibrate_linear speed 0.3   # ← 建图速度
ros2 param set /calibrate_linear odom_linear_scale_correction 1.0
ros2 topic pub /set_odom geometry_msgs/msg/Pose2D "{x: 0.0, y: 0.0, theta: 0.0}" --once

# Step 6: 触发测试
ros2 param set /calibrate_linear start_test true

# Step 7: 量实际距离，计算系数
# 公式：factor = 实际距离 / odom报告
# 例：odom 报 0.993, 实际 1.04 → factor = 1.04 / 0.993 ≈ 1.047

# Step 8: 保存到配置文件
nano ~/ros2_ws/src/xflz_eai_car1/driver/controller/config/calibrate_params.yaml
# 填入算出的系数

# Step 9: 重启 bringup 使生效
# Ctrl+C 停掉 bringup，重新启动
ros2 launch car1_bringup bringup.launch.py

# Step 10: 验证（关键！）
# 重新启动标定节点（这次脚本内系数设 1.0，验证 odom 单独效果）
ros2 run calibration calibrate_linear
ros2 param set /calibrate_linear speed 0.3
ros2 param set /calibrate_linear odom_linear_scale_correction 1.0
ros2 topic pub /set_odom geometry_msgs/msg/Pose2D "{x: 0.0, y: 0.0, theta: 0.0}" --once
ros2 param set /calibrate_linear start_test true

# Step 10 预期结果：
# - odom 报告 ≈ 0.97~0.99m（受 tolerance=0.03m 影响，提前停止）
# - 实际距离 ≈ odom 报告（两者吻合 = 标定成功）
# 例：odom 报 0.98m，实际量 0.98m → ✅ 完全吻合
```

**双系数说明（避免混淆）**：

| 系数 | 位置 | 作用 | 标定时 | 验证时 |
|------|------|------|--------|--------|
| `linear_correction_factor` | calibrate_params.yaml<br/>(odom_publisher) | 放大 odom 输出，全局生效 | 暂设 1.0 | 填实测值 |
| `odom_linear_scale_correction` | calibrate_linear 脚本运行时参数 | 脚本内部判断距离 | 暂设 1.0 | 暂设 1.0 |

⚠️ **两个系数会叠加**：脚本系数 × odom_publisher 系数。验证时脚本系数必须设 1.0，否则会双重修正。

**验证通过判据**：odom 报告值 ≈ 实际尺子量值（误差 <3cm 优秀，<5cm 合格）。

**随时停车**（任意时刻）：
```bash
ros2 param set /calibrate_linear start_test false
# 或兜底： pkill -f calibrate_linear
```

**保存结果**：

```bash
nano ~/ros2_ws/src/xflz_eai_car1/driver/controller/config/calibrate_params.yaml
```

```yaml
/**:
  ros__parameters:
    linear_correction_factor: 0.961   # ← 填入你的校准值
    angular_correction_factor: 1.00
```

> 保存后**重启 bringup** 即生效（yaml 启动时被读取，无需编译）。

#### 3.0.2 角速度校准

**原理**：让机器人原地旋转，用手机指南针量真实转角，对比里程计报告。

**校准脚本说明（2026-09-07 更新）**：
- `calibrate_angular` 同时订阅 `/odom_raw`、`/odometry/filtered`(EKF)、`/imu` 三个源。
- 支持 `use_ekf` 参数：`false`=用 odom_raw 判停，`true`=用 EKF 判停（默认 `false`）。
- 结束时同时打印 `finish: IMU=X° odom_raw=Y° EKF=Z° target=W°`，一次定位建图斜角根因。
- **IMU/EKF 是三个源中最接近物理真值的**（麦轮原地转打滑，odom_raw 是开环命令积分、会虚报）。
- `angular_correction_factor` 已在 `odom_publisher_node.py:196` 接线生效（yaw 增量乘该系数）。

**准备工作**：
1. 手机打开指南针 APP，固定在车上（放车后部，远离电机避免磁干扰）
2. 记录起始朝向
3. 确保四轮均匀接地

**操作步骤**：

```bash
# 终端2：停掉直线校准，启动角度校准
pkill -f calibrate_linear
ros2 run calibration calibrate_angular

# 终端3：设参数并触发
ros2 param set /calibrate_angular test_angle 360.0
ros2 param set /calibrate_angular speed 0.3
ros2 param set /calibrate_angular use_ekf false       # false=odom_raw判停, true=EKF判停(推荐建图用)
ros2 param set /calibrate_angular odom_angular_scale_correction 1.0
ros2 param set /calibrate_angular start_test true
```

小车自动旋转，判停源到目标角就停，终端2 打印 `finish: IMU=X° odom_raw=Y° EKF=Z° target=W°`。

> ⚠️ **关键**：脚本「转够」的判断基于判停源（odom_raw 或 EKF），**停车 ≠ 真转了 360°**。
> 麦轮原地转打滑，odom_raw 是开环命令积分会虚报，**必须以 IMU 或指南针量真实转角**为准。

**计算修正系数**：

```
angular_correction_factor = 真实转角 / 360
```

> 例：脚本停车时指南针显示实际转了 347°（少转 13°）→ 系数 = 347 / 360 ≈ 0.964。

**跨 0° 换算**（指南针跨越 360/0 边界时）：
```
偏差 = (结束角度 - 起始角度 + 180) % 360 - 180
```

**反复验证**：

```bash
ros2 param set /calibrate_angular odom_angular_scale_correction 0.964
ros2 param set /calibrate_angular start_test true
# 指南针量真实转角，应接近 360°
```

> 实测案例：系数 0.964 验证后残留均值约 4°（1.1%）。残留在 2~10° 间随机跳动，
> 是**起步惯性 + 瓷砖打滑 + 指南针精度** 的随机误差，不是系数问题，不必强求归零。

**保存结果**：

```yaml
# driver/controller/config/calibrate_params.yaml
/**:
  ros__parameters:
    linear_correction_factor: 0.961
    angular_correction_factor: 0.964   # ← 填入你的校准值
```

#### 3.0.3 校准验证

完成校准后，**重启 bringup** 让系数生效，确认参数被读取：

```bash
ros2 launch car1_bringup bringup.launch.py

# 确认两个系数已加载
ros2 param get /odom_publisher linear_correction_factor    # 应为 0.961
ros2 param get /odom_publisher angular_correction_factor   # 应为 0.964
```

**验证要点**：
- [ ] `ros2 param get` 返回标定值（非默认 1.0）
- [ ] 线速度：走 1m，尺子实测误差 < 3cm
- [ ] 角速度：转 360°，指南针真实转角误差 < 5°

**常见问题**：
- **速度异常快 / 转好几圈才停**：底层单位错配，检查固件轮径(0.097)和 mecanum.py(发m/s)是否已修复，见本节顶部前置说明
- **小车前后横跳不停**：旧版 calibrate_linear 闭环震荡 bug，已重写为单向前进；确认已重新编译 calibration 包并重启节点
- **每次结果不同（误差 2~10° 跳动）**：瓷砖打滑 + 起步惯性 + 指南针精度的随机误差，正常，取多次均值
- **`rqt_reconfigure` 找不到**：用 `ros2 param set` 命令行替代，见上方步骤

---

### 3.1 依赖确认

```bash
# 确保以下依赖已安装
sudo apt install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher
```

### 3.2 编译相关包

```bash
cd ~/ros2_ws

# 编译雷达驱动（从源码编译，确保兼容 ROS2 参数传递）
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select m1ct_d2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --build-base src/xflz_robot_car1/build --install-base install

# 编译 SLAM 及依赖包
colcon build --packages-select \
    car1_base_driver_msgs \
    car1_servo_controller_msgs \
    car1_sdk \
    car1_base_driver \
    car1_servo_controller \
    car1_chassis_controller \
    car1_bringup \
    car1_peripherals \
    car1_description \
    car1_slam \
    --symlink-install

source install/setup.bash
```

### 3.3 串口配置

机器人有两个 USB 串口（均为 CH340 芯片），需要正确识别：

- `/dev/car1_stm32`：STM32 底盘控制板（波特率 1000000，协议头 `0xAA 0x55`）
- `/dev/car1_lidar`：COIN-D6 激光雷达（波特率 230400）

> 别名由 `/etc/udev/rules.d/99-car1-serial.rules` 按物理 USB 口固定，不依赖 `ttyUSB` 枚举序号（插拔顺序不影响）。

**确认方法**：
```bash
# 运行识别脚本，通过数据特征区分
sudo python3 -c "
import serial, time
for port in ['/dev/car1_stm32', '/dev/car1_lidar']:
    try:
        ser = serial.Serial(port, 1000000, timeout=2)
        time.sleep(0.5)
        data = b''
        start = time.time()
        while time.time() - start < 2:
            if ser.in_waiting > 0:
                data += ser.read(ser.in_waiting)
            time.sleep(0.1)
        ser.close()
        if b'\xaa\x55' in data:
            print(f'{port}: STM32 底盘 (检测到 0xAA 0x55 协议头)')
        elif len(data) == 0:
            print(f'{port}: 无数据 (可能是雷达，需要初始化命令)')
        else:
            print(f'{port}: 未知设备 ({len(data)} bytes)')
    except Exception as e:
        print(f'{port}: 错误 - {e}')
"
```

**修改配置文件**（如果串口分配与默认不同）：
- 底盘串口：`driver/car1_base_driver/config/base_driver.yaml` 中的 `serial_port`
- 雷达串口：`peripherals/car1_peripherals/launch/lidar.launch.py` 中的 `port` 参数

### 3.4 启动 SLAM

```bash
source ~/ros2_ws/install/setup.bash

# 启动 SLAM（包含底盘驱动 + 雷达 + URDF TF + slam_toolbox + RViz）
ros2 launch car1_slam slam.launch.py
```

**启动内容**：
- `car1_bringup`：底盘驱动（STM32 通信 + IMU 滤波 + 里程计 + EKF + 舵机控制）
- `m1ct_d2`：COIN-D6 雷达驱动
- `robot_state_publisher`：从 URDF 发布完整 TF 树
- `joint_state_publisher`：发布关节状态（用于 RViz 模型显示）
- `slam_toolbox`：SLAM 建图（延迟 3 秒启动，等待传感器稳定）
- `rviz2`：可视化界面（自动加载保存的配置）

### 3.5 RViz 显示设置

如果 RViz 没有自动加载配置，手动设置：

1. **Fixed Frame**：设为 `map`
2. **Add → By topic → /scan → LaserScan**
   - Topic → Reliability 改为 **Best Effort**（雷达使用 SensorDataQoS）
3. **Add → By topic → /map → Map**
   - Topic → Reliability: `Reliable`
   - Topic → Durability: `Transient Local`
4. **Add → By display type → RobotModel**
   - Description Topic: `/robot_description`
5. **Add → By display type → TF**

设置完成后保存配置：File → Save Config As → `src/xflz_robot_car1/slam/car1_slam/rviz/slam.rviz`

### 3.6 建图操作

**启动键盘遥控**（新终端）：
```bash
source ~/ros2_ws/install/setup.bash
ros2 run car1_peripherals teleop_key_control
```

**键位**（麦轮全向）：
- `w` / `s` — 前进 / 后退
- `a` / `d` — 左转 / 右转
- `q` / `e` — 左平移 / 右平移
- `空格` — 停止
- `Ctrl+C` — 退出

节点内部 `LIN_VEL=0.2, ANG_VEL=0.5`，按住键才走、松开立即停（`get_key()` 有 0.1s 超时，无按键时发零速），发布到 `/controller/cmd_vel`（绕过 `/cmd_vel` 的 ±0.2 m/s 钳位）。SSH 也能直接用，无需 xterm。

> **注意**：如果没法用键盘（比如 nohup 后台运行），可以用 `ros2 topic pub` 兜底：
> ```bash
> # 前进 3 秒
> timeout 3 ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10
>
> # 左转 2 秒
> timeout 2 ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" --rate 10
>
> # 左平移（麦轮）
> timeout 2 ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10
> ```
> 用 `/controller/cmd_vel`（不是 `/cmd_vel`）避开 `odom_publisher` 的安全钳位，才能跑到 0.3 m/s 与标定速度对齐。

**建图技巧**：
- 速度与标定速度保持一致（linear ≈ 0.3 m/s，angular ≤ 0.3 rad/s）。**标定速度=建图速度**，否则线速度系数失效（见 3.0.1.1）
- 匀速移动，避免急加速和急转弯
- 尽量覆盖整个环境
- 走闭环（回到起点），slam_toolbox 会自动做回环检测优化地图
- 避免长走廊等特征稀少的环境

### 3.7 保存地图

```bash
# 建图完成后保存
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/xflz_robot_car1/slam/car1_slam/maps/my_map
```

保存后会生成两个文件：
- `my_map.pgm`：地图栅格图像
- `my_map.yaml`：地图元数据（分辨率、原点等）

### 3.8 验证要点

- [ ] 所有节点正常启动（无 SIGSEGV 或串口错误）
- [ ] RViz 中能看到机器人模型、激光点云、TF 树
- [ ] 移动机器人后地图实时构建
- [ ] 地图质量良好（无重影、墙壁清晰）
- [ ] 地图保存成功

### 3.9 常见问题

| 现象 | 原因 | 解决方案 |
|------|------|---------|
| `No such file: /dev/car1_stm32` | 串口设备名不对 / udev 别名缺失 | 运行串口识别脚本，确认 `/dev/car1_stm32` 存在；修改 `base_driver.yaml` |
| `imu_filter_madgwick` not found | 依赖未安装 | `sudo apt install ros-${ROS_DISTRO}-imu-filter-madgwick` |
| `No module named 'control_msgs'` | 依赖未安装 | `sudo apt install ros-${ROS_DISTRO}-control-msgs` |
| m1ct_d2 崩溃 (exit code -6) | 旧版二进制不支持 ROS2 参数 | 从源码重新编译 m1ct_d2 |
| RViz 中 LaserScan 无显示 | QoS 不匹配 | LaserScan 的 Reliability 改为 Best Effort |
| Map 显示 "No map received" | slam_toolbox 未处理 scan | 确认 `transform_timeout` ≥ 1.0（在 `slam.yaml` 中） |
| RobotModel 不显示 | 缺少 joint_states | 确认 `joint_state_publisher` 节点在运行 |
| 轮子不停 | cmd_vel 超时未停止 | 系统已内置 0.5 秒超时自动停止 |
| 地图重影 | 移动速度过快或与标定速度不符 | 匀速 ≈ 0.3 m/s（与标定一致），angular ≤ 0.3 |
| 定位丢失 | 环境特征不足 | 增加环境特征物，避免空旷区域 |

### 3.10 IMU 校准与航向验证（关键经验，2026-06）

> 本节记录一次完整的航向问题排查。麦轮原地转打滑严重（实测 ~45%），**航向必须靠 IMU**，
> 因此 IMU 链路 + EKF 融合是 SLAM 成败的核心。曾出现建图航向漂移 162°，根因是下面 5 个 bug 叠加。

#### 3.10.1 航向问题的 5 个根因（全部已修复）

| # | 问题 | 现象 | 修复 |
|---|------|------|------|
| 1 | IMU gyro scale 2× 过报告 | 转 1 圈 IMU 积分报 2 圈 | 固件 bmi088.c：soft-reset 后加 `HAL_Delay(30)` + 0x0F 回读校验 |
| 2 | IMU 协方差全 0 | EKF 静默丢弃 IMU 数据 | base_driver_node.py：填非零协方差（角速度对角 0.0025） |
| 3 | imu0_relative=true | vyaw 被减去初值 | ekf.yaml：改 `imu0_relative: false` |
| 4 | **缺 base_link→imu_link TF** | EKF 无法变换 IMU 坐标（主因） | bringup.launch.py：加 robot_state_publisher |
| 5 | 轮速 vyaw 与 IMU vyaw 冲突 | 打滑轮速污染航向 | ekf.yaml：odom0 关 vyaw，航向只用 IMU |

#### 3.10.2 IMU scale 验证（转动法）

```bash
# 停掉 ROS，直接串口测（脚本独占串口，不能开 bringup）
python3 src/xflz_eai_car1/test/imu_chassis_test.py
```

判读"缩小倍数"：
- ~1.0 → IMU 正常（本项目实测转 300° / IMU 积分 303°，1.01×）
- ~2.0 → scale 错误（量程寄存器问题）

#### 3.10.3 EKF 融合 IMU 验证（分步排查）

```bash
ros2 launch car1_bringup bringup.launch.py

# 1. IMU 链路三段验证
ros2 topic hz /car1_base_driver/imu_raw   # STM32 源头，~10 Hz
ros2 topic hz /imu                         # 滤波输出，~10 Hz
ros2 node info /ekf_filter_node | grep -A5 Subscribers  # 含 /imu

# 2. 协方差非零（关键！）
ros2 topic echo /imu --once --field angular_velocity_covariance
# 应为 [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]，全 0 则 EKF 丢弃

# 3. TF 链完整（关键！）
ros2 run tf2_ros tf2_echo base_link imu_link
# 应看到静态变换，"frame does not exist" 说明缺 robot_state_publisher

# 4. 核心：转动时 EKF 角速度是否跟随
#    终端A
ros2 topic echo /odometry/filtered --field twist.twist.angular.z
#    终端B
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -r 20
# 车转时终端A应显示 ~0.5（恒为 0 = EKF 没用 IMU）
```

#### 3.10.4 航向闭环验证（最终判据）

```bash
# 终端A：监控航向
ros2 run tf2_ros tf2_echo odom base_footprint
# 记录起始 yaw(degree)

# 终端B：原地转整数圈（0.5 rad/s 转一圈约 12.5s）
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -r 20
# 目视转回原朝向后 Ctrl+C
```

**判据**：转 N 整圈后 yaw 回到起始值。本项目实测闭环误差 **0.25°/圈**（合格线 <5°）。

#### 3.10.5 常见陷阱

| 陷阱 | 说明 |
|------|------|
| imu_chassis_test.py 与 bringup 抢串口 | 两者都占 /dev/car1_stm32，必须先停 bringup 再跑脚本 |
| bringup 缺 robot_state_publisher | 修复后已加入 bringup；EKF 依赖 base_link→imu_link TF |
| 协方差全 0 | robot_localization 无法求逆 → 静默丢弃，必须填非零 |
| tf2_echo 时序误判 | 静止/转动日志要严格对齐时间，否则误判"yaw 不变" |
| cmd_vel 持续发布转太多圈 | `-r 20` 不会自动停，用 `timeout 12.5` 控制圈数 |

---

## Level 4: 自主导航

> **目标**：使用已有地图进行自主导航  
> **所需时间**：30 分钟  
> **硬件要求**：雷达 + 底盘 + 已保存的地图

### 4.1 编译导航包

```bash
cd ~/ros2_ws
colcon build --packages-select car1_navigation
source install/setup.bash
```

### 4.2 启动导航系统

#### 方式 1：完整启动（推荐，包含所有组件）

```bash
# 终端 1：启动导航（默认地图为 slam_map）
ros2 launch car1_navigation navigation.launch.py map:=slam_map

# 如果使用其他地图
# ros2 launch car1_navigation navigation.launch.py map:=my_map
```

**此命令会自动启动**：
- 底盘驱动（bringup）
- 激光雷达（lidar）
- 定位系统（AMCL + map_server，地图自动加载）
- 导航服务器（planner、controller、bt_navigator）
- RViz 可视化界面（如果 `use_rviz:=true`，默认启用）

**验证地图加载成功**：
```bash
# 查看日志，应该看到：
# [map_server]: Loading yaml file: .../slam_map.yaml
# [map_server]: Loaded map with size: XX x YY
# [amcl]: Received a XX X YY map @ 0.050 m/pix

# 或通过话题验证
ros2 topic echo /map --once
```

> **✅ 修复说明**（2026-07-06）：地图加载问题已解决，`map_server` 现在会自动加载地图，无需手动调用服务。

### 4.3 RViz 配置

如果 RViz 显示不正常，按以下步骤配置：

1. **设置 Fixed Frame**：
   - 左上角 **Global Options → Fixed Frame** 设为 `map`

2. **添加地图**：
   - 点击左下角 **Add** 按钮
   - 选择 **By topic** → `/map` → **Map**
   - 展开 Map 设置，确保：
     - **Topic** → **Reliability**: `Reliable`
     - **Topic** → **Durability**: `Transient Local`

3. **添加激光扫描**：
   - **Add** → **By topic** → `/scan` → **LaserScan**
   - 设置 **Reliability** 为 `Best Effort`

4. **添加 AMCL 粒子云**：
   - **Add** → **By display type** → **PoseArray**
   - 展开 PoseArray，设置 **Topic** 为 `/particle_cloud`

5. **添加机器人模型**：
   - **Add** → **By display type** → **RobotModel**
   - **Description Topic** 设为 `/robot_description`

6. **添加路径规划显示**（可选）：
   - **Add** → **By topic** → `/plan` → **Path**（全局路径）
   - **Add** → **By topic** → `/local_plan` → **Path**（局部路径）

7. **添加代价地图**（可选）：
   - **Add** → **By topic** → `/global_costmap/costmap` → **Map**
   - **Add** → **By topic** → `/local_costmap/costmap` → **Map**

### 4.4 导航操作步骤

#### 步骤 1：设置初始位姿

机器人需要知道自己在地图上的初始位置。

1. 在 RViz 顶部工具栏点击 **2D Pose Estimate** 按钮
2. 在地图上**点击**机器人的当前实际位置
3. **按住鼠标拖动**设置机器人的朝向（箭头方向）
4. 释放鼠标

**验证定位成功**：
- AMCL 粒子云（绿色箭头）应该快速收敛到机器人周围
- 机器人模型应该与实际位置对齐
- 激光扫描点云应该与地图墙体重合

> **提示**：如果定位不准，可以多次设置初始位姿，或者稍微移动机器人让 AMCL 重新收敛。

#### 步骤 2：发送导航目标

1. 在 RViz 顶部工具栏点击 **2D Goal Pose** 按钮
2. 在地图上**点击**目标位置
3. **按住鼠标拖动**设置目标朝向
4. 释放鼠标

**观察导航过程**：
- **绿色路径**：全局规划路径（从起点到终点的完整路径）
- **红色路径**：局部规划路径（当前实时跟随的路径）
- **蓝色/紫色区域**：代价地图（障碍物膨胀区域）
- 机器人会自动沿路径导航到目标点

#### 步骤 3：取消导航（如需要）

如果想中途取消导航：
```bash
# 新终端
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --feedback
# 按 Ctrl+C 取消
```

或者在 RViz 中重新设置一个新的目标位置。

### 4.5 验证要点

完成以下验证项：

- [ ] **地图加载成功**：RViz 中能看到地图
- [ ] **AMCL 定位正常**：粒子云收敛，机器人位姿与地图对齐
- [ ] **路径规划成功**：设置目标后能看到绿色全局路径
- [ ] **机器人移动**：机器人沿路径自动移动
- [ ] **到达目标点**：机器人能准确到达目标位置（误差 <10cm）
- [ ] **自动避障**：路径上放置障碍物，机器人能自动绕开

### 4.6 常见问题与解决

#### 问题 1：定位不准确

**现象**：机器人模型与实际位置不对齐，粒子云发散。

**解决**：
1. 重新设置初始位姿，尽量准确
2. 移动机器人让 AMCL 重新收敛
3. 检查激光扫描是否正常：`ros2 topic echo /scan`
4. 检查 TF 树是否完整：`ros2 run tf2_tools view_frames`

#### 问题 2：路径规划失败

**现象**：设置目标后没有绿色路径，或显示 "Goal is not reachable"。

**解决**：
1. 确认目标点不在障碍物内（不要设在黑色区域）
2. 确认目标点在地图范围内
3. 尝试设置更近的目标点
4. 检查全局代价地图：`ros2 run rviz2 rviz2`，添加 `/global_costmap/costmap`

#### 问题 3：机器人不移动

**现象**：路径规划成功，但机器人静止不动。

**解决**：
1. 检查 `controller_server` 是否运行：
   ```bash
   ros2 node list | grep controller_server
   ```
2. 检查 `/cmd_vel_nav` 话题是否有数据：
   ```bash
   ros2 topic echo /cmd_vel_nav
   ```
3. 检查底盘驱动是否正常：
   ```bash
   ros2 topic echo /odom
   ```

#### 问题 4：导航震荡或抖动

**现象**：机器人在原地震荡，或路径跟随不稳定。

**解决**：
1. 调整控制器参数（`nav2_params.yaml` 中的 DWB 参数）
2. 检查里程计数据质量
3. 降低速度限制

#### 问题 5：地图未加载（已修复）

**现象**：AMCL 显示 "Waiting for map"，RViz 中看不到地图。

**解决**：
- ✅ **此问题已在 2026-07-06 修复**，地图现在会自动加载
- 如果仍遇到此问题，检查：
  1. 地图文件是否存在：`ls ~/ros2_ws/src/xflz_eai_car1/slam/car1_slam/maps/slam_map.*`
  2. map_server 节点是否运行：`ros2 node list | grep map_server`
  3. 查看 map_server 日志：检查是否有加载错误

### 4.7 多点导航（Python API）

如果需要让机器人自动巡逻多个点，可以使用 Python API：

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy

rclpy.init()
navigator = BasicNavigator()

# 等待导航系统准备就绪
navigator.waitUntilNav2Active()

# 定义多个目标点
goal_poses = []

# 目标点 1
goal_pose1 = PoseStamped()
goal_pose1.header.frame_id = 'map'
goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
goal_pose1.pose.position.x = 1.0
goal_pose1.pose.position.y = 0.0
goal_pose1.pose.orientation.w = 1.0
goal_poses.append(goal_pose1)

# 目标点 2
goal_pose2 = PoseStamped()
goal_pose2.header.frame_id = 'map'
goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
goal_pose2.pose.position.x = 0.0
goal_pose2.pose.position.y = 1.0
goal_pose2.pose.orientation.w = 1.0
goal_poses.append(goal_pose2)

# 循环导航
for goal_pose in goal_poses:
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # 可以在这里处理反馈信息
        pass
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'到达目标点')
    else:
        print(f'导航失败')

navigator.lifecycleShutdown()
```

保存为 `multi_point_nav.py`，运行：
```bash
python3 multi_point_nav.py
```

---

## Level 5: 智能应用

> **目标**：运行视觉和雷达相关的智能应用  
> **所需时间**：1 小时  
> **硬件要求**：深度相机 + 雷达 + 底盘 + 机械臂

### 5.1 巡线行驶

**功能**：识别地面黑线，沿线行驶。

```bash
# 编译应用包
colcon build --packages-select car1_app
source install/setup.bash

# 启动巡线
ros2 launch car1_app line_following.launch.py
```

**使用说明**：
- 在地面铺设黑色胶带作为引导线
- 机器人会自动识别并沿线行驶
- 可在 `config/app_params.yaml` 中调整 LAB 阈值和 PID 参数

**验证要点**：
- [ ] 能识别黑线
- [ ] 沿线行驶平稳
- [ ] 转弯响应及时

### 5.2 颜色目标追踪

**功能**：识别指定颜色物体，跟随目标移动。

```bash
# 启动颜色追踪
ros2 launch car1_app object_tracking.launch.py
```

**使用说明**：
- 在相机前放置红色物体（默认追踪红色）
- 机器人会自动跟随物体移动
- 保持一定距离（约 0.5m）

**验证要点**：
- [ ] 能识别目标颜色
- [ ] 跟随目标移动
- [ ] 保持合适距离

### 5.3 手势控制

**功能**：通过手势控制底盘运动。

```bash
# 启动手势控制
ros2 launch car1_app hand_gesture.launch.py
```

**手势说明**：
- **手掌张开**：停止
- **握拳**：前进
- **竖起食指**：后退
- **OK 手势**：左转
- **点赞**：右转

**验证要点**：
- [ ] 手势识别准确
- [ ] 底盘响应及时
- [ ] 控制流畅

### 5.4 雷达避障

**功能**：3 扇区雷达避障，自动规避障碍物。

```bash
# 启动雷达避障
ros2 launch car1_app lidar_controller.launch.py
```

**使用说明**：
- 机器人会自动前进
- 检测到障碍物时自动转向
- 可调整避障距离阈值

**验证要点**：
- [ ] 检测到障碍物
- [ ] 自动转向避障
- [ ] 避障后继续前进

### 5.5 视觉抓取（旧版,移动底盘）

**功能**：识别目标物体，机械臂自动抓取。

```bash
# 启动视觉抓取
ros2 launch car1_app track_and_grab.launch.py
```

**使用说明**：
- 在机器人前方放置红色物体
- 机器人会自动识别并抓取
- 抓取后放置到指定位置

**状态机流程**：
1. SEARCH - 搜索目标
2. APPROACH - 接近目标
3. ALIGN - 对准目标
4. GRAB - 抓取
5. LIFT - 提升
6. DONE - 完成

**验证要点**：
- [ ] 能识别目标物体
- [ ] IK 求解成功
- [ ] 抓取成功
- [ ] 放置准确

> ⚠️ **此节点已不推荐**：依赖底盘移动 + 硬编码 PWM,与 Level 3 已验证的 IK 接口不互通。
> 新项目请使用 5.6 节的 `visual_grasp`,基于 IK + look-then-move,不需要底盘移动。

---

### 5.6 视觉抓取（推荐，静态桌面 + look-then-move）

> **目标**：通过深度相机识别桌面物体，基于 IK 自动抓取（机器人底盘不动）  
> **所需时间**：30-60 分钟  
> **硬件要求**：HP60C 深度相机 + 机械臂 + 红色物体（30~50mm）  
> **前置条件**：完成 Level 1.7（交互式 IK 验证通过）  
> **当前状态（2026-07-08）**：**一站式启动 + 重复抓取已完成**，支持服务触发的自动化抓取流程

#### 5.6.1 功能概述

`visual_grasp` 是 Level 4 的主节点,采用 **look-then-move（先看后动）** 策略:

```
观察姿态(俯视) → HSV 检测 → 深度+TF 算 base_link 坐标
   → 暂停确认（可选） → 调用 /arm_controller/pick → 完成 → 回 IDLE
```

**核心特性**（2026-07-08 更新）：
- **一站式启动**：一条命令启动所有依赖（bringup + arm_controller + depth_camera + rqt_image_view）
- **长驻式节点**：启动后保持 IDLE 状态，通过 `/visual_grasp/start` 服务触发抓取
- **重复抓取**：抓取完成后自动回到 IDLE，可连续触发多次抓取，无需重启
- **实时可视化**：三栏并排（RGB + Depth + HSV Mask）+ 坐标信息，通过 rqt 远程查看
- **服务化控制**：所有交互通过 ROS 服务（start/confirm/skip/abort），SSH 远程友好

**为什么不做持续视觉伺服**：相机挂在机械臂末端 `arm_link4`（eye-in-hand）,下压抓取时相机会跟着进去,失去目标视野。look-then-move 在视野好的高位看准,再 IK 下去抓,工程上最稳。

**坐标变换无需手眼标定**: 相机通过 URDF(`camera_mount_joint` 挂 arm_link4)+ robot_state_publisher 提供 TF。物体坐标走 TF 链 `camera_depth_optical_frame → base_link`,实时反映机械臂姿态。

**pitch 约定(与 ik_analytical/web_server 一致)**: `0=朝上, 90°=水平, 180°=夹爪朝下`。抓取用 180°。

#### 5.6.2 一站式启动（推荐方式）

**单条命令启动所有组件**：

```bash
cd ~/ros2_ws
source install/setup.bash

# 一键启动（默认配置）
ros2 launch car1_app visual_grasp.launch.py
```

**启动内容**（自动分阶段启动，带延迟保证依赖就绪）：
```
t=0s    bringup（base_driver + servo_controller + robot_state_publisher）
t=5s    arm_controller（IK + pick/place 服务）
t=8s    depth_camera（HP60C 驱动）
t=12s   visual_grasp 节点（IDLE 等待触发）
t=14s   rqt_image_view（可视化窗口，自动打开）
```

**可选参数**（按需覆盖）：

```bash
# 如果某些组件已经在运行，可以单独关闭
ros2 launch car1_app visual_grasp.launch.py start_bringup:=false

# 可用参数：
# - start_bringup:=false    # 跳过 bringup（如已在别处启动）
# - start_arm:=false        # 跳过 arm_controller
# - start_camera:=false     # 跳过 depth_camera
# - start_rqt:=false        # 不打开 rqt 可视化窗口
# - wait_for_confirmation:=false  # 检测后直接抓取，不暂停确认
# - dry_run:=true           # 只检测不抓取（调试用）
```

**启动成功标志**：

终端日志最后会显示：
```
[visual_grasp-11] ============================================================
[visual_grasp-11] Ready. Waiting for /visual_grasp/start service call...
[visual_grasp-11]   Trigger grasp: ros2 service call /visual_grasp/start std_srvs/srv/Trigger
[visual_grasp-11] ============================================================
[visual_grasp-11] Camera intrinsics: fx=594.5, fy=594.8, cx=350.3, cy=237.5
```

同时 rqt_image_view 窗口自动打开，显示实时三栏可视化。

#### 5.6.3 重复抓取工作流

**第一次抓取**：

```bash
# 在桌面放置红色物体（机械臂前方 20-30cm）

# 另开终端，触发抓取
ros2 service call /visual_grasp/start std_srvs/srv/Trigger
```

visual_grasp 会自动执行：
```
IDLE → INIT → MOVE_TO_OBSERVE → WAIT_STABLE → REFRESH_JOINTS 
  → DETECT → WAIT_CONFIRMATION（暂停）
```

**观察可视化**（rqt_image_view 窗口）：
- **RGB 栏**：检测框（黄色）、轮廓（绿色）、质心十字（红色）
- **Depth 栏**：深度伪彩色图、质心十字（白色）、深度值标注
- **Mask 栏**：HSV 匹配区域（白色）
- **底部坐标**：
  - `Camera: (x, y, z) m` — 红块在相机坐标系的位置
  - `Base: (x, y, z) m` — 红块在 base_link 的坐标（**抓取目标**）
  - `Joints: [j1°, j2°, ..., j5°]` — 当前关节角
  - `TF age: X.XXs` — TF 数据新鲜度

**确认坐标后触发抓取**：

```bash
# 检查 Base 坐标是否合理（x ≈ 0.2-0.3, y ≈ ±0.05, z ≈ 0.01-0.03）
# 确认后触发抓取
ros2 service call /visual_grasp/confirm std_srvs/srv/Trigger
```

机械臂自动执行：
```
COMPUTE_TARGET → PICK → DONE → 回到 IDLE
```

**第二次及后续抓取**：

```bash
# 放置新的红色物体

# 再次触发（无需重启任何节点）
ros2 service call /visual_grasp/start std_srvs/srv/Trigger

# 确认
ros2 service call /visual_grasp/confirm std_srvs/srv/Trigger
```

**跳过或中止**：

```bash
# 检测后不想抓这个物体，跳过
ros2 service call /visual_grasp/skip std_srvs/srv/Trigger

# 紧急中止
ros2 service call /visual_grasp/abort std_srvs/srv/Trigger
```

#### 5.6.4 无确认模式（自动抓取）

如果坐标已经验证准确，可以关闭确认环节：

```bash
# 停掉当前 launch（Ctrl+C）
# 重启时加参数
ros2 launch car1_app visual_grasp.launch.py wait_for_confirmation:=false
```

这样每次调用 `/visual_grasp/start` 后，检测成功就直接抓取，无需手动 confirm。

#### 5.6.5 实时可视化说明

**可视化布局**（`/visual_grasp/visualization` 话题）：

```
┌────────────────────────────────────────────────────────┐
│ State: WAIT_CONFIRMATION | Centroid: (u,v) | Depth: Xmm │  ← 顶部状态栏
├────────────┬────────────┬─────────────────────────────┤
│ RGB        │ Depth      │ HSV Mask                    │
│ + 检测轮廓 │ + 深度伪彩 │ + 匹配区域                  │  ← 三栏并排
│ + BBox     │ + 质心十字 │ + 质心十字                  │
│ + 质心十字 │ + 深度值   │                             │
│ + TARGET   │            │                             │
└────────────┴────────────┴─────────────────────────────┘
│ Camera: (x, y, z) m                                    │  ← 底部坐标栏
│ Base:   (x, y, z) m                                    │
│ Joints: [j1°, j2°, j3°, j4°, j5°]  |  TF age: 0.05s   │
└────────────────────────────────────────────────────────┘
```

**关键信息判读**：

| 元素 | 用途 |
|------|------|
| `State` | 定位当前状态（IDLE/DETECT/WAIT_CONFIRMATION/PICK） |
| `Centroid: (u, v)` | 像素坐标，验证检测中心 |
| `Depth: Xmm` | 质心处深度，用尺子对比验证 |
| RGB 红十字 vs Depth 白十字 | 对齐判断（偏差 <5px 正常） |
| HSV Mask 白色区域 | 判断阈值是否合适（太宽=背景污染，太窄=红块缺失） |
| `Base: (x, y, z)` | **核心坐标**，用尺子实测验证（x=前方距离, y=左右偏移, z=高度） |
| `TF age` | <0.2s 正常，>0.5s 说明 joint_states 更新慢 |

**如果 rqt 窗口未自动打开或需要手动启动**：

```bash
ros2 run rqt_image_view rqt_image_view /visual_grasp/visualization
```

#### 5.6.6 状态机流程

```
IDLE（待命）
  ↓ 收到 /visual_grasp/start 服务调用
INIT → MOVE_TO_OBSERVE → WAIT_STABLE(5s) → REFRESH_JOINTS
  → DETECT → WAIT_CONFIRMATION（暂停，等 confirm/skip/abort）
  ↓ 收到 /visual_grasp/confirm
COMPUTE_TARGET → PICK → DONE → 回到 IDLE（可再次触发）
```

**关键改进**（2026-07-08）：
- **IDLE 状态**：长驻等待，不再一次性运行后退出
- **REFRESH_JOINTS**：检测前同步读取真实舵机位置，确保 TF 准确
- **服务化交互**：所有触发通过 ROS 服务，解决 SSH 下键盘输入问题
- **自动回 IDLE**：DONE/FAILED 后自动重置状态，支持重复抓取

#### 5.6.7 可调参数

在 launch 文件中预设的参数（可通过 launch 参数覆盖）：

```python
# 观察姿态 [x, y, z, pitch(rad)]，pitch=π 表示夹爪朝下
observe_pose: [0.20, 0.0, 0.22, 3.14159]

# 红色 HSV 阈值（双区间，覆盖 H=0-15 和 H=160-180）
target_color_hsv_lower: [0, 100, 80]
target_color_hsv_upper: [15, 255, 255]
target_color_hsv_lower2: [160, 100, 80]
target_color_hsv_upper2: [180, 255, 255]

# 检测重试次数
max_detection_attempts: 10

# 其他
dry_run: false                # true=只检测不抓取
enable_viz: true              # 发布可视化图像
wait_for_confirmation: true   # 检测后暂停确认
```

**修改方式**（编辑 launch 文件或运行时覆盖）：

```bash
# 运行时覆盖示例（不推荐，参数较多）
ros2 launch car1_app visual_grasp.launch.py \
  wait_for_confirmation:=false \
  dry_run:=true
```

推荐直接编辑 `app/car1_app/launch/visual_grasp.launch.py` 文件中的参数。

#### 5.6.8 分层验证（逐环节排查）

**第 1 层：相机话题**
```bash
ros2 topic list | grep ascamera
ros2 topic hz /ascamera_hp60c/camera_publisher/rgb0/image  # 等≥10秒，应~11Hz
ros2 topic echo /ascamera_hp60c/camera_publisher/depth0/camera_info --once
```

**第 2 层：可视化窗口**
- rqt_image_view 自动打开
- 能看到红色物体
- RGB/Depth/Mask 三栏都有内容

**第 3 层：TF 链路**
```bash
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
# 应持续打印变换，且随机械臂姿态变化
```

**第 4 层：检测功能**
- 触发 start 后进入 WAIT_CONFIRMATION
- RGB 栏有黄色检测框框住红块
- Base 坐标显示合理值（x ≈ 0.2-0.3, z ≈ 0.01-0.03）

**第 5 层：完整抓取**
- confirm 后机械臂下压
- 夹爪闭合
- 提起物体
- 回到 IDLE

#### 5.6.9 常见问题

| 现象 | 原因 | 解决方案 |
|------|------|---------|
| 启动卡住 | 依赖未就绪 | 等待足够时间（~15秒），看日志定位哪个组件未启动 |
| rqt 窗口未打开 | 环境问题 | 手动启动：`ros2 run rqt_image_view rqt_image_view /visual_grasp/visualization` |
| 可视化全黑 | 相机推流延迟 | 等待 10-15 秒，相机按需推流 |
| 检测不到红色 | HSV 阈值不合适 | 看 Mask 栏，调整 launch 文件中的 HSV 参数 |
| Base z 坐标异常大（>0.1m） | joint_states 不准 | 手动调用：`ros2 service call /servo_controller/refresh_positions std_srvs/srv/Trigger` |
| confirm 服务调用卡住 | 旧版本 bug（已修复） | 确认使用 2026-07-08 之后的版本，callback group 已修复 |
| 夹爪不张开 | gripper_duration 不够 | 见 5.6.17，增大 `arm_params.yaml` 中的 `gripper_duration: 2.0` |
| 夹爪撞桌面 | TCP 偏移未补偿 | 见 5.6.17，设置 `gripper_depth: -0.033` |

#### 5.6.10 架构说明（可选阅读）

**一站式 launch 的实现**：

- 使用 `IncludeLaunchDescription` 复用现有 launch 文件
- 使用 `TimerAction` 实现阶段性延迟启动（避免竞态）
- 使用 `launch_arguments` 显式传参避免参数名冲突（如 `start_bringup`）
- `rqt_image_view` 用 `ExecuteProcess` 启动并传入话题参数

**重复抓取的实现**：

- 状态机增加 `IDLE` 状态
- `DONE` 和 `FAILED` 不再 `raise SystemExit`，而是 `_change_state(State.IDLE)`
- `_reset_for_next_run()` 清空 per-run 状态（检测结果、future、attempts）
- 用户服务（start/confirm/skip/abort）使用独立 `ReentrantCallbackGroup` 避免阻塞
- `main()` 使用 `MultiThreadedExecutor` 支持并发回调

**服务化交互的原因**：

- ROS launch 重定向 stdin，`select.select([sys.stdin], ...)` 检测不到键盘输入
- ROS 服务通过网络调用，SSH 远程、后台运行、自动化脚本都友好
- 服务调用有返回值（success/message），便于脚本判断

#### 5.6.11 前置修改总结（已完成）

以下底层 bug 已在 2026-06~07 修复，首次使用前需确保重新编译：

| 组件 | 关键改动 |
|------|---------|
| `servo_controller` | 按需读回架构 + `refresh_positions` 服务 + callback group 修复 |
| `base_driver` | `_recv_task` 异常兜底 + 响应包字节布局修正 |
| `arm_controller` | IK 改用 `ik_analytical.py` + pitch 约定对齐 + gripper_depth 参数 |
| `depth_camera.launch.py` | 去掉 remap（保留原生话题名） |
| `visual_grasp.py` | IDLE 状态机 + 服务化控制 + 三栏可视化 + 黑帧跳过 |
| `visual_grasp.launch.py` | 一站式启动 + TimerAction 延迟 + rqt 自动打开 |
| `car1.urdf` | 相机光心标定（camera_depth_joint y 偏移） |
| `arm_params.yaml` | gripper_depth/gripper_duration 参数 |

```bash
# 确保重新编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  car1_base_driver car1_servo_controller car1_arm_controller \
  car1_peripherals car1_description car1_app --symlink-install
source install/setup.bash
```

视觉抓取系统需要 5 个 launch 文件协同工作，每个文件负责系统的一部分功能：

##### 1. bringup.launch.py — 底层驱动启动

**位置**：`bringup/launch/bringup.launch.py`

**功能**：
- 启动 STM32 板级驱动（`base_driver.launch.py`）：串口通信、IMU、电机、舵机、LED、蜂鸣器
- 启动底盘控制器（`chassis_controller.launch.py`）：
  - IMU 滤波（imu_filter_madgwick）
  - 里程计发布（odom_publisher）
  - EKF 融合（robot_localization）
  - 舵机控制器（servo_controller）
  - 初始位姿发布（init_pose）

**启动命令**：
```bash
ros2 launch car1_bringup bringup.launch.py
```

**关键话题/服务**：
- `/car1_base_driver/imu_raw` — 原始 IMU 数据
- `/imu` — 滤波后 IMU
- `/odom_raw` — 轮式里程计
- `/odometry/filtered` — EKF 融合里程计
- `/joint_states` — 舵机关节状态（20Hz 异步回读）
- `/car1_base_driver/bus_servo/set_position` — 舵机位置控制服务

**验证方法**：
```bash
# 检查 IMU 数据
ros2 topic hz /imu

# 检查关节状态回读
ros2 topic echo /joint_states --once

# 检查板级驱动初始化完成
ros2 topic echo /car1_base_driver/init_finish --once
```

---

##### 2. arm_controller.launch.py — 机械臂高层控制

**位置**：`arm/car1_arm_controller/launch/arm_controller.launch.py`

**功能**：
- 启动机械臂控制器节点（`arm_controller`）
- 提供高层 IK 接口和抓取服务
- 加载预设位姿配置（`preset_poses.yaml`）
- 加载机械臂参数（`arm_params.yaml`）

**可选参数**：
- `start_bringup:=true` — 同时启动 bringup（默认 false，假设已运行）

**启动命令**：
```bash
# 假设 bringup 已运行
ros2 launch car1_arm_controller arm_controller.launch.py

# 或一键启动（包含 bringup）
ros2 launch car1_arm_controller arm_controller.launch.py start_bringup:=true
```

**关键服务/话题**：
- `/arm_controller/target_pose` (PoseStamped) — IK 位姿控制
- `/arm_controller/joint_command` (JointState) — 直接关节角度控制
- `/arm_controller/pick` (Trigger) — 执行抓取序列
- `/arm_controller/place` (Trigger) — 执行放置序列
- `/arm_controller/set_gripper` (SetBool) — 夹爪开合
- `/arm_controller/go_home` (Trigger) — 回零位
- `/arm_controller/emergency_stop` (Trigger) — 紧急停止

**验证方法**：
```bash
# 检查服务是否就绪
ros2 service list | grep arm_controller

# 测试回零位
ros2 service call /arm_controller/go_home std_srvs/srv/Trigger

# 测试夹爪
ros2 service call /arm_controller/set_gripper std_srvs/srv/SetBool "{data: true}"
```

---

##### 3. depth_camera.launch.py — 深度相机驱动

**位置**：`peripherals/car1_peripherals/launch/depth_camera.launch.py`

**功能**：
- 启动 HP60C 深度相机节点（ascamera_node）
- 配置相机参数（分辨率、帧率、标定文件路径）
- **关键设计**：不使用 remap，保留原生话题名

**为什么不能 remap**：  
ascamera 驱动通过话题名子串 `"rgb0"` / `"depth0"` / `"ir0"` 判断流类型（见 `CameraPublisher.cpp:getSubTopicStreamType`）。如果 remap 将话题名改为 `/depth_cam/rgb/image_raw`，丢失 `"0"` 子串，驱动无法识别流类型，永不推流。

**启动命令**：
```bash
ros2 launch car1_peripherals depth_camera.launch.py
```

**发布话题（原生路径，不可 remap）**：
- `/ascamera_hp60c/camera_publisher/rgb0/image` — RGB 图像
- `/ascamera_hp60c/camera_publisher/depth0/image_raw` — 深度图（uint16, mm）
- `/ascamera_hp60c/camera_publisher/depth0/camera_info` — 相机内参
- `/ascamera_hp60c/camera_publisher/depth0/points` — 点云（可选）

**验证方法**：
```bash
# 检查相机话题（等≥10秒，驱动按需推流）
ros2 topic hz /ascamera_hp60c/camera_publisher/rgb0/image

# 检查相机内参
ros2 topic echo /ascamera_hp60c/camera_publisher/depth0/camera_info --once

# 查看 RGB 图像（需要 rqt_image_view）
ros2 run rqt_image_view rqt_image_view
```

**注意事项**：
- 相机驱动**按需推流**：无订阅者时会自动 `stop streaming`（日志正常），有订阅者 1-1.5 秒后自动 `start streaming`
- 检查帧率时需等待 ≥10 秒，避免过早退出 `ros2 topic hz`
- 第一帧通常是黑帧（自动曝光未生效），visual_grasp 会自动跳过

---

##### 4. rsp_only.launch.py — TF 树发布

**位置**：`app/car1_app/launch/rsp_only.launch.py`

**功能**：
- 启动 robot_state_publisher 节点
- 从 URDF 加载机器人模型
- 结合 `/joint_states` 发布完整 TF 树
- **关键作用**：提供 `base_link` → `camera_depth_optical_frame` 变换

**启动命令**：
```bash
ros2 launch car1_app rsp_only.launch.py
```

**发布 TF**：
- `base_link` → `arm_base_link` → ... → `arm_link4` → `camera_depth_optical_frame`
- TF 随 `/joint_states` 实时更新，反映机械臂当前姿态

**验证方法**：
```bash
# 检查 TF 树
ros2 run tf2_tools view_frames
# 生成 frames.pdf，检查 camera_depth_optical_frame 是否存在

# 实时查看特定变换
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

**为什么需要**：  
visual_grasp 需要将深度相机坐标系中的物体位置变换到 base_link 坐标系。由于相机安装在机械臂末端 `arm_link4`（eye-in-hand），TF 变换随机械臂姿态动态变化，robot_state_publisher 自动处理这种动态变换。

---

##### 5. visual_grasp.launch.py — 视觉抓取主节点

**位置**：`app/car1_app/launch/visual_grasp.launch.py`

**功能**：
- 启动 robot_state_publisher（与 rsp_only 功能相同，可复用或单独启动）
- 启动 visual_grasp 节点（视觉抓取主逻辑）
- 配置抓取参数（观察姿态、HSV 阈值、重试次数）

**启动命令**：
```bash
ros2 launch car1_app visual_grasp.launch.py

# 或单独运行节点（假设 rsp_only 已启动）
ros2 run car1_app visual_grasp
```

**可配置参数**：
```python
observe_pose: [0.20, 0.0, 0.22, 3.14159]  # 观察姿态 [x, y, z, pitch(rad)]
target_color_hsv_lower: [0, 80, 60]       # 红色 HSV 下界（低 H 端）
target_color_hsv_upper: [10, 255, 255]
target_color_hsv_lower2: [160, 80, 60]    # 红色 HSV 上界（高 H 端）
target_color_hsv_upper2: [180, 255, 255]
max_detection_attempts: 10                # 最多检测尝试次数
verify_after_pick: false                  # 抓取后是否验证
dry_run: false                            # 仅检测定位，不执行抓取（调试用）
```

**订阅话题**：
- `/ascamera_hp60c/camera_publisher/rgb0/image` — RGB 图像（HSV 检测）
- `/ascamera_hp60c/camera_publisher/depth0/image_raw` — 深度图（3D 定位）
- `/ascamera_hp60c/camera_publisher/depth0/camera_info` — 相机内参
- `/joint_states` — 关节状态（通过 robot_state_publisher → TF）

**发布话题**：
- `/arm_controller/target_pose` — 目标位姿（观察姿态、抓取位置）
- `/visual_grasp/debug_image` — 调试图像（RGB + 检测框）
- `/visual_grasp/mask` — HSV 掩码（白色=匹配像素）

**调用服务**：
- `/arm_controller/pick` — 执行抓取序列

**验证方法**：
```bash
# 查看调试图像（检测框、轮廓）
ros2 run rqt_image_view rqt_image_view
# 选择 /visual_grasp/debug_image

# 查看 HSV 掩码（白色区域=检测到的红色）
# 选择 /visual_grasp/mask

# 监控节点日志
ros2 run car1_app visual_grasp
# 观察状态机流程：INIT → MOVE_TO_OBSERVE → DETECT → PICK → DONE
```

---
#### 5.6.12 下一步与进阶

完成基础视觉抓取后，可以：

1. **调整抓取参数**：根据不同物体调整 `gripper_depth`、HSV 阈值
2. **多物体抓取**：循环调用 `/visual_grasp/start`，实现批量抓取
3. **集成导航**：结合 Level 4 导航，实现"导航到目标点 → 抓取 → 导航到放置点"
4. **更换颜色**：修改 launch 文件中的 HSV 参数，追踪蓝色、绿色等物体
5. **深度学习检测**：替换 HSV 检测为 YOLO/MobileNet，提升鲁棒性

**故障排查优先级**（遇到问题时按顺序检查）：
1. 可视化窗口（rqt）— 确认检测框、深度、坐标是否合理
2. Base 坐标 — 用尺子实测验证 x/y/z
3. gripper_duration — 夹爪不张开时增大到 2.0s
4. gripper_depth — 撞桌面时减小（更负），夹不到时增大

**相关章节**：
- 1.7 交互式 IK 验证 — 验证 IK 准确性
- 3.0 底盘校准 — 如需移动底盘配合抓取
- 5.6.11 前置修改总结 — 确保代码是最新版本

---

### 5.7 导航 + 视觉抓取 + 运输（全自主任务）

> **目标**：机器人自主导航到抓取点 → 视觉识别红色方块 → 抓取 → 导航到放置点 → 放置 → 返回工作姿态
> **所需时间**：60-90 分钟（含调试）
> **硬件要求**：完整硬件（底盘 + 机械臂 + 深度相机 + 激光雷达）
> **前置条件**：已完成 Level 4（自主导航）+ Level 5.6（视觉抓取）
> **当前状态（2026-07-08）**：完整实现，与 visual_grasp 完全解耦（零侵入设计）

#### 5.7.1 功能概述

`navigation_grasp_transport` 是 Level 5 的全自主任务节点，实现完整的**"导航 → 抓取 → 运输 → 放置 → 返回"**闭环。

**核心特性**：
- **零侵入设计**：不修改 visual_grasp.py 和 arm_controller.py 核心逻辑，两个案例互不影响
- **13 状态状态机**：完整覆盖任务全流程，每个阶段独立可调试
- **旋转搜索**：导航到达后自动多角度搜索目标（应对 ±10cm 定位误差）
- **多次检测取稳**：连续检测 5 次，用最后一次结果消除舵机稳定过程中的坐标漂移
- **深度反投影**：与 visual_grasp 相同的定位算法（HSV + 深度 + TF）
- **服务化触发**：ROS 服务调用启动任务，SSH 远程友好
- **可配置放置**：放置点位置和姿态完全可配置

#### 5.7.2 状态机流程

```
IDLE（待命）
  ↓ 收到 /navigation_grasp_transport/start
SET_INITIAL_POSE（发布初始位姿到 AMCL）
  ↓
WAIT_LOCALIZATION（等待 AMCL 收敛 3秒）
  ↓
NAV_TO_PICK（Nav2 导航到抓取点）
  ↓ 到达
MOVE_TO_OBSERVE（机械臂移到观察姿态 [0.20, 0.0, 0.22, π]）
  ↓
WAIT_ARM_STABLE（等待舵机完全稳定 5秒）
  ↓
ROTATE_SEARCH（旋转搜索红色方块，覆盖 ±45°）
  ↓ 找到目标
DETECT_AND_GRASP（连续检测5次取稳，发布 set_pick_target）
  ↓
CALL_PICK（调用 /arm_controller/pick 服务）
  ↓
WAIT_GRASP（等待抓取完成）
  ↓ 抓取成功
NAV_TO_PLACE（导航返回放置点）
  ↓ 到达
PLACE_OBJECT（多步操作：place → go_work → 关闭夹爪）
  ↓
DONE（任务完成，5秒后回 IDLE，可再次触发）
```

#### 5.7.3 一站式启动

```bash
cd ~/ros2_ws
source install/setup.bash

# 一键启动（默认地图 slam_map）
ros2 launch car1_app navigation_grasp_transport.launch.py map:=slam_map
```

**启动内容**（自动分阶段启动）：
```
t=0s    navigation.launch.py（bringup + lidar + AMCL + Nav2）
t=8s    arm_controller.launch.py（机械臂控制器）
t=10s   depth_camera.launch.py（HP60C 相机）
t=15s   navigation_grasp_transport 节点（IDLE 等待触发）
```

**启动成功标志**：
```
[navigation_grasp_transport-22] navigation_grasp_transport ready (IDLE)
[navigation_grasp_transport-22]   initial_pose: [-0.059, -0.103, 0.020]
[navigation_grasp_transport-22]   pick_pose:    [0.756, 0.872, 3.071]
[navigation_grasp_transport-22]   place_pose:   [-0.059, -0.103, 0.020]
[navigation_grasp_transport-22]   search enabled: True
```

#### 5.7.4 位置配置

配置文件：`app/car1_app/config/navigation_grasp_transport.yaml`

**如何测量并写入你的地图坐标**：

```bash
# Step 1: 启动导航（只启动导航栈）
ros2 launch car1_navigation navigation.launch.py map:=slam_map

# Step 2: 在 RViz 中用 2D Pose Estimate 校准初始位姿

# Step 3: 用键盘遥控到目标位置
ros2 launch car1_peripherals teleop_key.launch.py

# Step 4: 到位后读取当前地图坐标
ros2 run tf2_ros tf2_echo map base_footprint

# 输出示例：
# - Translation: [0.756, 0.872, 0.000]
# - Rotation: in RPY (radian) [0.000, -0.000, 3.071]

# Step 5: 将坐标写入配置文件
# initial_pose: [x, y, yaw_rad]
# pick_pose: [x, y, yaw_rad]
# place_pose: [x, y, yaw_rad]
```

**关键配置项**：
```yaml
navigation_grasp_transport:
  ros__parameters:
    # 位姿配置（map 坐标系）[x_m, y_m, yaw_rad]
    initial_pose: [-0.059, -0.103, 0.020]
    pick_pose: [0.756, 0.872, 3.071]
    place_pose: [-0.059, -0.103, 0.020]  # 可与起点相同

    # 旋转搜索（应对定位误差）
    enable_search: true
    search_angles_deg: [0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0]
    red_pixel_threshold: 1000

    # 观察姿态（机械臂前伸俯视）
    observe_pose: [0.20, 0.0, 0.22, 3.14159]
    observe_settle_sec: 5.0  # 舵机稳定等待时间（关键！）

    # 检测稳定性（消除舵机稳定过程漂移）
    detection_repeat_count: 5           # 检测5次
    detection_repeat_interval_sec: 0.3  # 间隔0.3秒

    # 放置目标（base_link 坐标系，相对机器人）[x, y, z, pitch]
    place_target_arm: [0.26, 0.0, 0.08, 3.14159]

    # HSV 阈值（与 visual_grasp 一致）
    target_color_hsv_lower: [0, 100, 80]
    target_color_hsv_upper: [15, 255, 255]
    target_color_hsv_lower2: [160, 100, 80]
    target_color_hsv_upper2: [180, 255, 255]
```

#### 5.7.5 触发任务

**基本操作**：

```bash
# 触发完整任务
ros2 service call /navigation_grasp_transport/start std_srvs/srv/Trigger

# 紧急停止
ros2 service call /navigation_grasp_transport/stop std_srvs/srv/Trigger

# 运行时保存当前位置为抓取点（调试用）
ros2 service call /navigation_grasp_transport/set_pick_pose_from_current std_srvs/srv/Trigger
```

**监控状态**：

```bash
# 实时状态机状态
ros2 topic echo /navigation_grasp_transport/status

# 当前搜索角度
ros2 topic echo /navigation_grasp_transport/current_search_angle

# 错误原因（失败时）
ros2 topic echo /navigation_grasp_transport/error_reason
```

#### 5.7.6 关键技术点

##### 5.7.6.1 参考 visual_grasp 的定位算法

采用 `visual_grasp.py` 验证过的**深度反投影法**（非平面投影）：

```
1. HSV 检测 → 最大轮廓 → 质心像素 (u, v)
2. 深度采样：轮廓内中值（带 MAD 离群值剔除）
3. 反投影到相机坐标：
   x_cam = (u - cx) * z_cam / fx
   y_cam = (v - cy) * z_cam / fy
4. refresh_positions 强制读取舵机真实位置
5. TF 变换：camera_depth_optical_frame → base_link
6. 得到 (x_base, y_base, z_base) 抓取目标
```

##### 5.7.6.2 多次检测取稳（关键优化）

**问题**：舵机移到观察姿态后，前 3-5 秒还在微调（j2/j3/j4 逐步稳定），此时检测的坐标会有 ~2cm 的 x 方向偏差。

**解决**：
- 等待时间从 3s 增加到 **5s**（与 visual_grasp 一致）
- 连续检测 **5 次**（间隔 0.3s）
- 使用**最后一次成功**的检测结果（此时舵机已完全稳定）

**日志示例**：
```
Detect [1/5]: (0.243, -0.012, 0.045)  # 稍有偏差
Detect [2/5]: (0.247, -0.010, 0.045)
Detect [3/5]: (0.250, -0.010, 0.045)
Detect [4/5]: (0.251, -0.010, 0.045)
Detect [5/5]: (0.251, -0.010, 0.045)  # 最终稳定值 ← 使用
FINAL target: (0.251, -0.010, 0.045)
```

##### 5.7.6.3 状态分离避免竞态

**问题**：如果在同一个 tick 内发布 `set_pick_target` 后立即调用 `pick` 服务，arm_controller 的订阅回调可能还没跑到，导致 `pick` 使用**旧的 pick_target**（观察姿态坐标），机械臂在错误位置合拢夹爪。

**解决**：新增 `CALL_PICK` 独立状态
```
DETECT_AND_GRASP: 发布 set_pick_target → 进入 CALL_PICK
（间隔 100ms 让订阅回调处理）
CALL_PICK: 调用 pick 服务
```

##### 5.7.6.4 放置阶段的完整夹爪管理

**放置流程**：
1. 发布 `set_pick_target = [0.26, 0.0, 0.08, π]`（base_link 坐标）
2. 调用 `/arm_controller/place`（自动打开夹爪释放物体）
3. 调用 `/arm_controller/go_work`（机械臂到工作姿态）
4. 调用 `/arm_controller/set_gripper (data=False)`（关闭夹爪，准备下次抓取）
5. → DONE

##### 5.7.6.5 arm_controller 新增 go_work 服务

**Work 姿态定义**（关节角度，弧度）：
```
[0.0, -0.9424, 1.6492, 0.8246, 0.0]
```

对应 PWM `[1500, 1100, 2200, 1850, 1500]`（与 `test_servo_direct.py` 和 `work_ready.yaml` 一致）。

**为什么用 work 姿态而不是 home**：
- **home**（零位）：所有关节 0°，机械臂完全竖直向上
- **work**（工作准备）：机械臂前伸，重心低，导航时更稳定，也便于下次抓取

#### 5.7.7 分层验证

**第 1 层：Nav2 导航**
```bash
# 单独验证导航能到达抓取点
ros2 launch car1_navigation navigation.launch.py map:=slam_map
# RViz 手动设置目标点，验证能到达 pick_pose 坐标附近
```

**第 2 层：机械臂观察姿态**
```bash
# 单独验证机械臂能到观察姿态
ros2 launch car1_arm_controller arm_controller.launch.py start_bringup:=true

# 发布观察姿态
ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.20, y: 0.0, z: 0.22}, orientation: {y: 1.0, w: 0.0}}}" --once
```

**第 3 层：视觉抓取（不带导航）**
```bash
# 用 visual_grasp 验证抓取
ros2 launch car1_app visual_grasp.launch.py
ros2 service call /visual_grasp/start std_srvs/srv/Trigger
```

**第 4 层：完整流程**
```bash
# 一次完整任务
ros2 launch car1_app navigation_grasp_transport.launch.py map:=slam_map
ros2 service call /navigation_grasp_transport/start std_srvs/srv/Trigger
```

#### 5.7.8 验收标准

**功能性**：
- [ ] 初始位姿设置成功，AMCL 3秒内收敛
- [ ] 导航到抓取点误差 <10cm
- [ ] 旋转搜索能在 ±45° 范围内找到红色方块
- [ ] 5次检测坐标稳定收敛（最后一次与前一次差 <5mm）
- [ ] 抓取成功率 >70%（10次测试）
- [ ] 放置动作完整：place → go_work → 夹爪关闭
- [ ] 完整任务端到端成功率 >60%
- [ ] 连续 5 次抓取无程序崩溃

**性能**：
- [ ] 单次完整任务耗时 <3 分钟
- [ ] 状态机切换延迟 <0.5 秒

#### 5.7.9 常见问题

| 现象 | 原因 | 解决方案 |
|------|------|---------|
| 导航到达但没抓取 | 舵机未稳定就检测 | 增大 `observe_settle_sec` 到 5-7s |
| x 方向偏差 2cm | 单次检测取到未稳定值 | 已通过多次检测取最后一次解决 |
| 夹爪不合拢 | 竞态：pick 用了旧 target | 已通过 CALL_PICK 独立状态解决 |
| 到位置但没夹到 | 抓取位置不准 | 检查 `[TF]` 日志的 base_link 坐标，用尺子实测验证 |
| 放置时机械臂碰自身 | place_target_arm x 太近 | 增大 x（如从 0.20 → 0.26） |
| 放置后一直循环 | 状态机分支条件缺失 | 已通过检查所有 futures 解决 |
| Ray intersects behind | 平面投影法失败（已废弃） | 已切换到深度反投影 |
| 搜索检测不到 | 旋转搜索范围太窄 | 扩大 `search_angles_deg` |
| 抓取偏移大 | TF 不准 | 确保 `refresh_positions` 服务调用成功 |

#### 5.7.10 架构说明

**零侵入设计**：

```
路径 A（Level 5.6, 手动触发）：
  visual_grasp.py [独立] → arm_controller pick

路径 B（Level 5.7, 全自主）：
  navigation_grasp_transport.py [独立]
    → Nav2 goToPose
    → arm_controller target_pose (观察姿态)
    → 自实现 HSV 检测 + 深度反投影
    → arm_controller set_pick_target + pick
    → Nav2 goToPose (返回)
    → arm_controller set_pick_target + place
    → arm_controller go_work
    → arm_controller set_gripper
```

**两条路径完全解耦**：
- 都使用 arm_controller 的**同一套服务接口**
- 都使用相同的检测算法
- 但各自的状态机、参数、启动方式完全独立
- Level 5.6 单独启动**不受 Level 5.7 影响**

#### 5.7.11 相关文件

| 文件 | 作用 |
|------|------|
| `app/car1_app/car1_app/navigation_grasp_transport.py` | 主节点 |
| `app/car1_app/launch/navigation_grasp_transport.launch.py` | 一站式启动 |
| `app/car1_app/config/navigation_grasp_transport.yaml` | 参数配置 |
| `arm/car1_arm_controller/car1_arm_controller/arm_controller.py` | 新增 `go_work` 服务 |

#### 5.7.12 下一步

完成基础任务后，可以尝试：
1. **多点巡逻抓取**：扩展 `pick_pose` 为列表，循环访问多个抓取点
2. **多种颜色**：扩展 HSV 参数，识别不同颜色物体分类放置
3. **多个放置区**：根据物体颜色/大小选择不同放置点
4. **动态目标**：结合追踪算法，抓取移动中的物体
5. **协作任务**：多机器人协作，一个抓取一个搬运

---

## Level 6: 语音交互

> **目标**：测试板载 ES8388 音频硬件，后续集成语音识别与语音合成  
> **所需时间**：30-60 分钟  
> **硬件要求**：ES8388 音频编解码器（板载）  
> **前置条件**：alsa-utils 已安装

### 6.0 硬件概述

RK3576 板卡集成了 **ES8388** 音频编解码器，通过 I2S 接口与主控通信：

| 项目 | 说明 |
|------|------|
| 芯片 | ES8388 (rockchip-es8388) |
| ALSA 声卡 | `hw:0,0` |
| 麦克风 | 板载单路模拟麦克风（左声道） |
| 输出 | 板载喇叭 + 耳机插孔 |
| 采样格式 | S16_LE, 8000-48000 Hz |
| 驱动 | ALSA 内核驱动 + amixer 混音控制 |

**声卡信息查看**：

```bash
# 查看录音设备
arecord -l

# 查看播放设备
aplay -l

# 查看混音控制器
amixer -c 0 scontrols
```

典型输出中 `card 0: rockchipes8388` 即为板载 ES8388。

### 6.1 音频硬件测试（audio_test.py）

> **目标**：验证 ES8388 录音和播放链路是否正常工作，并调优增益  
> **所需时间**：5-15 分钟  
> **硬件要求**：ES8388 音频编解码器（板载麦克风 + 4Ω 3W 扬声器）

#### 6.1.1 工具概述

`audio_test.py` 位于 `interaction/car1_speech/tools/audio_test.py`，是一个**无 ROS2 依赖**的独立 ALSA 测试工具，提供三种模式：

| 模式 | 说明 | 使用场景 |
|------|------|---------|
| **loopback** | 录音+播放同时进行（arecord \| aplay 管道） | 快速验证全链路 |
| **record** | 只录音，保存为 WAV 文件 | 检查麦克风信号质量 |
| **play** | 只播放 WAV 文件 | 检查喇叭/耳机输出 |

**默认参数**（已针对 ES8388 + 板载硬件优化）：

```
设备:       hw:0,0
采样率:     16000 Hz（ASR 标准）
格式:       S16_LE
声道:       1 (单声道，板载麦克风只有左声道，用 -c 2 强制立体声)
mixer:      不修改（除非传入 --voice-preset / --capture-volume / --playback-volume）
```

**两个推荐预设**（实测优化）：

| 预设 | 场景 | 麦克风 | 扬声器 | 数字增益 |
|------|------|--------|--------|---------|
| `--voice-preset` | ASR 语音识别 | +18dB (88%) | -3dB（中等音量） | `--gain 2.0` |
| `--play-preset` / play 模式默认 | 纯播放（TTS/提示音） | 不变 | +4.5dB（硬件极限） | `--gain 2.0` |

**关键硬件参数**（RK3576 + ES8388 + NS4150B）：
- ES8388 Output 硬件范围：**-30dB ~ +4.5dB**（上限不可超越）
- NS4150B 功放：BTL D-class，约 **12dB 电压增益**（用户 Rin=20K 反推）
- 实测最响不破音：**`--gain 2.0` + `--playback-volume 100`**（Output=+4.5dB）
- ALC 关闭：避免静音时自动拉高 PGA 增益造成噪声底噪抬升

**录音增益映射表**（板载 MIC2 差分麦克风）：

| `--capture-volume` | PGA gain | 适用场景 |
|-------------------|----------|----------|
| 50% | 0dB | 中性（无增益） |
| 67% | +8dB | 近距离（10-20cm） |
| 75% | +12dB | 正常距离（20-30cm） |
| **88%** | **+18dB** | **推荐默认（30-50cm）** |
| 100% | +24dB | 远距离或低灵敏度麦克风 |

**播放音量映射表**（ES8388 → NS4150B → 4Ω 3W 扬声器）：

| `--playback-volume` | Output dB | 相对响度 | 场景 |
|---------------------|-----------|---------|------|
| 0 | 硬件静音 | — | 完全静默 |
| 25 | -18dB | 很小 | 安静提示 |
| 50 | -12dB | 小 | 夜间使用 |
| 67 | -6dB | 中等 | 室内正常 |
| 88 | -1dB | 较响 | 环境嘈杂 |
| **100** | **+4.5dB** | **最响** | **default / --play-preset** |

⚠️ **注意**：Output +4.5dB 是 ES8388 硬件极限，超过 `--gain 2.5` 会出现明显破音（NS4150B 饱和）。默认 `--gain 2.0` 是实测下最响的不破音组合。

#### 6.1.2 基本用法

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_speech/tools

# 1. 边录边播（loopback）— 验证全链路
python3 audio_test.py -d 5

# 2. 只录音 — 检查麦克风
python3 audio_test.py --mode record -d 5 -o rec.wav

# 3. 只播放 — 检查喇叭/耳机
python3 audio_test.py --mode play rec.wav

# 4. 查看设备列表
python3 audio_test.py -l

# 5. 查看混音控制器状态
python3 audio_test.py --list-mixer
```

#### 6.1.3 Loopback 模式（录音+播放）

```bash
# 无限录音+播放（Ctrl+C 停止）
python3 audio_test.py

# 限时 5 秒
python3 audio_test.py -d 5

# 同时保存录音文件
python3 audio_test.py -d 5 --playback-volume 100 -o test.wav
```

#### 6.1.4 录音模式

```bash
# 默认参数录音 5 秒（16000 Hz, 立体声, S16_LE）
python3 audio_test.py --mode record -d 5 -o rec.wav

# 调整录音增益为 50% (-6dB)
python3 audio_test.py --mode record -d 5 --capture-volume 50 -o rec.wav

# 录音增益拉满（0dB = 最大信号）
python3 audio_test.py --mode record -d 5 --capture-volume 100 -o rec.wav

# 单声道录音（板载麦克风只有左声道）
python3 audio_test.py --mode record -d 5 -c 1 -o rec_mono.wav
```

#### 6.1.5 播放模式（详细教程）

##### 硬件链路

RK3576 板卡的音频播放链路：

```
WAV 文件
  │
  │ [1] --gain 数字增益（软件，tanh 软削波）
  ↓
ES8388 DAC (I2S)
  │
  │ [2] --playback-volume → Output 1/2 (硬件模拟, -30dB ~ +4.5dB)
  ↓
NS4150B 功放 (BTL D-class, ~12dB 增益, Rin=20K)
  │
  │ 差分输出到 SPK_P/SPK_N
  ↓
板载扬声器 (4Ω 3W)
```

**关键硬件事实**：
- **ES8388 Output 硬件极限**：+4.5dB（不可超越）
- **NS4150B 功放**：BTL 差分输出，约 12dB 电压增益（Rin=20K 反推）
- **实测最响不破音组合**：`--gain 2.0` + `--playback-volume 100`（Output=+4.5dB）
- **超过 2.5x 数字增益** 在 Output=+4.5dB 时会出现明显破音

##### 快速开始

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_speech/tools

# 最常用：默认播放（数字增益 2.0x + 硬件音量 100%）—— 最响不破音
python3 audio_test.py --mode play test.wav

# 显式声明使用最响播放预设（等同于上面的默认行为）
python3 audio_test.py --play-preset --mode play test.wav
```

**默认行为**：`--mode play` 时如果没有指定 `--playback-volume`，会自动把 Output 拉到硬件极限 +4.5dB，加上 `--gain` 默认 2.0，就是**实测下最响的不破音组合**。

##### 完整参数

| 参数 | 默认值 | 范围 | 说明 |
|------|-------|------|------|
| `--gain` | `2.0` | 0.5-4.0 | 数字增益倍数（软件层，tanh 软削波） |
| `--playback-volume` | 自动（100）| 0-100 | 硬件音量百分比 → Output 1/2 dB |
| `--play-preset` | — | 开关 | 应用最响预设（Output=+4.5dB） |
| `--playback-device` | `hw:0,0` | ALSA 设备 | 播放设备（默认自动 mono→stereo） |

##### 音量控制的三个层次（可叠加）

**1. 数字增益 `--gain`（软件层）**

在软件里对 WAV 波形做数字放大，配合 tanh 软削波防止硬失真：

```bash
# 原始不放大
python3 audio_test.py --gain 1.0 --mode play test.wav

# 默认 2.0x（+6dB）—— 推荐
python3 audio_test.py --gain 2.0 --mode play test.wav

# 更响一些（可能破音，看信号本身电平）
python3 audio_test.py --gain 3.0 --mode play test.wav

# 最大 4.0x（+12dB）—— 结合 Output=+4.5dB 会明显破音
python3 audio_test.py --gain 4.0 --mode play test.wav
```

**2. 硬件音量 `--playback-volume`（模拟层）**

控制 ES8388 Output 1/2 的模拟输出电平（-30dB 到 +4.5dB）：

| `--playback-volume` | Output dB | 相对响度 | 用途 |
|---------------------|-----------|---------|------|
| 0 | 硬件静音 | — | 完全静默 |
| 25 | -18dB | 很小 | 安静提示 |
| 50 | -12dB | 小 | 夜间使用 |
| 67 | -6dB | 中等 | 室内正常 |
| 88 | -1dB | 较响 | 环境嘈杂 |
| **100** | **+4.5dB** | **最响** | **默认推荐** |

```bash
# 硬件静音
python3 audio_test.py --playback-volume 0 --mode play test.wav

# 中等音量
python3 audio_test.py --playback-volume 50 --mode play test.wav

# 最响（等同默认）
python3 audio_test.py --playback-volume 100 --mode play test.wav
```

**3. 单声道 → 立体声转换（自动）**

ES8388 的 I2S 硬件配置固定为立体声。播放单声道 WAV 时如果直接送 `hw:0,0`，硬件按立体声时钟读单声道数据，会出现 **2 倍速播放**。

解决方案：`audio_test.py` 自动检测单声道 WAV，把 mono 复制到 L/R 后写临时立体声 WAV 再播放，用户无需干预。

##### 常见使用案例

**案例 1：播放 TTS 合成的语音**

TTS 输出通常电平适中，用默认参数即可：

```bash
python3 audio_test.py --mode play tts_output.wav
```

**案例 2：播放录音回放（低电平）**

自己录的语音峰值往往只有 -6 到 -12dB FS，需要数字增益补偿：

```bash
# 默认 gain=2.0 通常够用
python3 audio_test.py --mode play recording.wav

# 如果录音特别小，用 gain=3.0
python3 audio_test.py --gain 3.0 --mode play recording.wav
```

**案例 3：夜间安静播放**

```bash
python3 audio_test.py --playback-volume 30 --gain 1.0 --mode play notification.wav
```

**案例 4：紧急提示音（最响）**

```bash
python3 audio_test.py --play-preset --gain 2.0 --mode play alert.wav
```

**案例 5：完全原始播放（调试用）**

```bash
python3 audio_test.py --gain 1.0 --playback-volume 100 --mode play test.wav
```

**案例 6：批量播放脚本**

```bash
# 播放多个 WAV 文件（同一音量设置）
for f in *.wav; do
    python3 audio_test.py --mode play "$f"
done
```

##### 音量调优决策树

```
播放时觉得声音小？
  │
  ├─ 是低电平录音回放？
  │    → 提高 --gain（试 2.5, 3.0）
  │      注意：>2.5 在 --playback-volume=100 时可能破音
  │
  ├─ 是 TTS 合成结果？
  │    → 默认参数应该够响
  │      如果小，试 --gain 2.5
  │
  └─ 已经 gain=2.0 + volume=100 还是小？
       → 这就是硬件极限了
         受 NS4150B (~12dB 增益) 和扬声器 (4Ω 3W) 的物理限制

播放时声音破音？
  │
  ├─ gain > 2.5 且 volume=100
  │    → 降 gain 到 2.0（保持最响不破音）
  │
  ├─ gain=2.0 但录音本身接近满幅
  │    → 用 gain=1.0（原始播放）
  │
  └─ 依然破音
       → 检查录音源本身是否失真
         检查扬声器是否松动/损坏
```

##### 输出电平参考

在**默认参数** `--gain 2.0 --playback-volume 100`（Output=+4.5dB）下：

| 源信号峰值 | 播放感受 | 备注 |
|-----------|---------|------|
| -3 dB FS（大声说话录音） | 响，无破音 | 理想情况 |
| -6 dB FS（正常说话录音） | 响，无破音 | 推荐目标 |
| -12 dB FS（小声说话录音） | 中等 | 可以加 --gain 2.5 |
| -20 dB FS（很小录音） | 偏小 | 应重新录音，不建议靠数字增益补偿 |

##### 硬件保护说明

- ES8388 Output 最大 **+4.5dB**（硬件极限，不可超越）
- NS4150B BTL 输出峰值受 5V 电源限制（约 8-9Vpp 差分）
- 4Ω 3W 扬声器在当前硬件配置下不会被烧坏（NS4150B 会在饱和前 clipping）
- **破音是硬件饱和的自然保护**，不会立即损坏扬声器，但长期使用会加速老化

##### 与录音 `--voice-preset` 的区别

| 预设 | 场景 | 播放响度 | 原因 |
|------|------|---------|------|
| `--voice-preset` | ASR 语音识别 | 中等（Output=-3dB） | 避免播放太响串扰麦克风 |
| `--play-preset` / play 模式默认 | 纯播放（TTS/提示音） | 最响（Output=+4.5dB） | 用户需要清晰听到内容 |

#### 6.1.6 混音器音量控制

`--playback-volume` 和 `--capture-volume` 参数通过 **dB 值**控制混音器：

| 百分比 | dB 值 | 音量感受 |
|--------|-------|----------|
| 100% | 0 dB | 最大（不衰减） |
| 50% | -6 dB | 一半电压 |
| 25% | -12 dB | 四分之一 |
| 10% | -20 dB | 很轻 |

**为什么用 dB 而不是原始百分比**：ES8388 的某些混音控制器（如 PCM、Capture Digital）是非线性刻度 —
`amixer sset PCM 50%` 直接设为 50% raw 值会得到 -48 dB（近乎静音），而非直觉中的"一半音量"。
工具内部使用 `20 × log₁₀(pct/100)` 换算为 dB 后通过 `amixer sset <ctl> -- -6.0dB` 设置，
确保 50% = -6dB（真正的一半电压）。

**影响的混音控制器**：

| 参数 | 控制对象 | 说明 |
|------|---------|------|
| `--playback-volume` | 播放：Output 1, Output 2, PCM | Output 1/2 为模拟音量，PCM 为数字音量 |
| `--capture-volume` | 录音：Left Channel, Right Channel, Capture Digital | Left/Right Channel 为前放（0~+24dB），Capture Digital 为数字增益 |

**录音路径特殊处理**：Left/Right Channel（前放）始终固定在最大值 +24dB（最佳信噪比），
只用 Capture Digital 实现音量衰减。这样 `--capture-volume 50` 时前放保持 +24dB，
Capture Digital 设 -6dB，信号既不会被前放压死，又能获得准确的 50% 衰减。

```bash
# 播放 50% + 录音 50%
python3 audio_test.py --playback-volume 50 --capture-volume 50 -d 5 -o test.wav

# 播放 80% + 录音 100%（调试推荐）
python3 audio_test.py --playback-volume 80 --capture-volume 100 -d 5 -o test.wav

# 不传参数则不改动 mixer，使用系统当前状态
python3 audio_test.py
```

#### 6.1.7 高级参数

| 参数 | 说明 | 示例 |
|------|------|------|
| `--rec-device` | 独立的录音 PCM 设备 | `--rec-device plughw:0,0` |
| `--playback-device` | 独立的播放 PCM 设备 | `--playback-device plughw:0,0` |
| `-r / --rate` | 采样率 (Hz) | `-r 48000` |
| `-c / --channels` | 声道数 | `-c 1` (单声道) |
| `-f / --format` | 采样格式 | `-f S32_LE` |
| `--period-size` | ALSA period 大小（帧），越小延迟越低 | `--period-size 512` |
| `--buffer-size` | ALSA buffer 大小（帧），越大越稳定 | `--buffer-size 2048` |
| `--card` | 声卡编号（用于 mixer 控制） | `--card 0` |
| `-q / --quiet` | 静默模式 | |

```bash
# 低延迟 loopback
python3 audio_test.py -d 5 --period-size 512 --buffer-size 2048

# 独立录/放设备
python3 audio_test.py -d 5 --rec-device hw:0,0 --playback-device plughw:0,0

# 48kHz 录音
python3 audio_test.py --mode record -r 48000 -d 5 -o rec_48k.wav
```

#### 6.1.8 常见问题

**Q: 录音文件一个声道有声音、另一个声道没声音**

这是**硬件正常现象**。ES8388 板子只接了一路模拟麦克风（左声道），右声道没有物理信号输入。
右声道只会录到极低电平的底噪（峰值 ≈ 0.1%，RMS ≈ 6）。

**解决方案**：
- 使用 `-c 1` 参数（单声道录音），WAV 文件尺寸减半且无冗余静音通道
- 或在应用层取左声道数据，忽略右声道

**验证方法**：
```bash
# 检查录音文件左右声道信号
python3 audio_test.py --mode record -d 3 -c 2 -o stereo.wav
python3 -c "
import struct
with open('stereo.wav', 'rb') as f:
    f.read(44)
    data = f.read()
    l_peak = r_peak = 0
    for i in range(0, len(data), 4):
        l_peak = max(l_peak, abs(struct.unpack_from('<h', data, i)[0]))
        r_peak = max(r_peak, abs(struct.unpack_from('<h', data, i+2)[0]))
    print(f'Left peak:  {l_peak:5d} ({100*l_peak/32768:.1f}%)')
    print(f'Right peak: {r_peak:5d} ({100*r_peak/32768:.1f}%)')
    print(f'Right silent: {r_peak < 20}')
"
# 预期：Left peak ~500-7000 (1-20%)，Right peak ~15-30 (<0.1%)
```

**Q: 播放音量太小**

播放链路涉及多个混音控制器，逐一排查：

```bash
# 查看当前混音状态
python3 audio_test.py --list-mixer

# 拉满播放音量
python3 audio_test.py --playback-volume 100 --mode play test.wav

# 或手动设置
amixer -c 0 sset "Output 1" -- 0dB
amixer -c 0 sset "Output 2" -- 0dB
amixer -c 0 sset "PCM" -- 0dB
```

**Q: 录音音量太小 / 信号微弱**

录音链路的增益由 Left/Right Channel（前放 0~+24dB）决定，检查是否被意外拉低：

```bash
# 查看前放增益
amixer -c 0 sget "Left Channel"
# 显示 [0%] [0.00dB] = 前放被压死，几乎无信号
# 显示 [100%] [24.00dB] = 最大增益

# 恢复工厂默认
amixer -c 0 sset "Left Channel" -- 24.0dB
amixer -c 0 sset "Right Channel" -- 24.0dB
amixer -c 0 sset "Capture Digital" -- 0dB
```

**Q: 播放单声道 WAV 时听起来像"快进"（2 倍速）**

**原因**：ES8388 的 I2S 硬件配置固定为立体声（每帧 2 slot）。直接用 `hw:0,0` 播放单声道 WAV 时，硬件以立体声速率读取单声道数据，导致听起来变速。

**解决方案**：`audio_test.py` 会自动将 `hw:X,Y` 升级为 `plughw:X,Y`，通过 ALSA 插件处理声道转换。用户无需干预：

```bash
# 默认（推荐）：自动使用 plughw:0,0，兼容任何 WAV
python3 audio_test.py --mode play mono.wav

# 如果需要绕过自动升级（使用原始 hw 设备）
python3 audio_test.py --playback-device hw:0,0 --mode play stereo_only.wav
```

**手动播放的解决方式**（如果用 aplay 直接播放）：

```bash
# 错误：单声道 WAV 会加速
aplay -D hw:0,0 mono.wav

# 正确：用 plughw 让 ALSA 处理声道转换
aplay -D plughw:0,0 mono.wav
```

**Q: 喇叭/耳机没声音，但 aplay 正常退出**

检查功放开关：

```bash
# 查看开关状态
amixer -c 0 sget "spk switch"    # 喇叭功放
amixer -c 0 sget "hp switch"     # 耳机功放

# 如果显示 [off]，手动开启
amixer -c 0 sset "spk switch" on
amixer -c 0 sset "hp switch" on
```

#### 6.1.9 音频调优与质量分析

以上是基础测试。如需深入调优录音/播放增益、验证音质、检查扬声器功率安全，请参考：

**📘 完整音频调优指南**：`interaction/car1_speech/tools/AUDIO_TUNING_GUIDE.md`

该指南包含：
- **录音增益调优流程**：如何找到适合你环境的最佳 PGA 增益（默认 88% = +21dB）
- **播放音量调优**：如何在响度和扬声器安全之间平衡（默认 67% = -15dB = 1.08W）
- **音频质量分析**：使用 `audio_analyzer.py` 自动检测削峰、噪声底噪、信号强度
- **扬声器功率安全检查**：验证输出功率是否在 3W 扬声器的安全范围内
- **降噪策略**：为什么必须关闭 ALC，如何优化 SNR
- **故障排查**：常见问题的完整诊断流程

**快速调优命令**：

```bash
# 推荐默认配置（适合大多数场景）
python3 audio_test.py --voice-preset --mode record -o test.wav -d 5 --analyze
# = mic +21dB (88%), speaker -15dB (67%), ALC off, 单声道

# 分析已有录音质量
python3 audio_analyzer.py test.wav

# 检查扬声器功率安全
python3 audio_analyzer.py --check-speaker -15.0  # 67% 音量（推荐）
python3 audio_analyzer.py --check-speaker -12.0  # 100% 音量（最大安全）

# 测量环境噪声底噪
python3 audio_analyzer.py --measure-noise
```

**增益快速参考**：

| 场景 | 录音增益 | 播放音量 | 命令 |
|------|---------|----------|------|
| **日常 ASR（推荐）** | 88% (+18dB) | 67% (-15dB, 1.08W) | `--voice-preset` |
| **纯播放（TTS/音乐）** | 不变 | 100% (-12dB, 2.15W) | `--play-preset` |
| 安静环境/远距离 ASR | 100% (+24dB) | 67% | `--capture-volume 100` |
| 嘈杂环境/近距离 ASR | 75% (+12dB) | 67% | `--capture-volume 75` |
| 自定义播放音量 | 不变 | 任意 0-100% | `--playback-volume 50` |
| 静音播放 | 不变 | 0% (硬件静音) | `--playback-volume 0` |

**两个预设的区别**：

| 预设 | 录音增益 | 播放音量 | 适用场景 |
|------|---------|----------|----------|
| `--voice-preset` | +18dB (88%) | -15dB (67%, 1.08W) | ASR 语音识别，播放较柔和避免与麦克风串扰 |
| `--play-preset` | 不变 | -12dB (100%, 2.15W) | 纯播放场景（TTS、音乐、提示音），最大安全音量 |

**播放模式默认行为**：`--mode play` 时，如果没指定 `--playback-volume` 且没有其他预设，会**自动应用 100% 最大安全音量**（-12dB, 2.15W）。这样普通 `python3 audio_test.py --mode play test.wav` 就能获得最响的可用音量。

**播放模式音量控制的三个层次**（可叠加）：

1. **数字增益** `--gain`（默认 2.0）：在软件层放大信号幅度，用于低电平录音回放。使用 tanh 软削波防止失真
2. **硬件音量** `--playback-volume`（默认 100% = -12dB）：控制 ES8388 的 Output 1/2 输出电平
3. **单声道→立体声转换**（自动）：单声道 WAV 会自动复制到左右声道，避免 ES8388 I2S 立体声时钟错配导致的 2 倍速播放

```bash
# 默认（数字增益 2.0x + 硬件音量 100%）
python3 audio_test.py --mode play test.wav

# 声音太小 → 加大数字增益
python3 audio_test.py --gain 3.0 --mode play test.wav

# 声音失真 → 降低数字增益
python3 audio_test.py --gain 1.5 --mode play test.wav

# 播放原始信号（无处理）
python3 audio_test.py --gain 1.0 --mode play test.wav
```

**为什么用数字增益而不是继续提高硬件音量**：4Ω 3W 扬声器的硬件功率上限是 -12dB（2.15W）。再往上会超过 3W，长期使用会损坏扬声器。数字增益在软件层放大信号幅度，配合软削波，既能提升响度又保护硬件。

---

### 6.2 语音识别测试（asr_test.py）

> **目标**：使用 sherpa-onnx + RKNN + 内置端点检测实现流式语音识别  
> **所需时间**：15 分钟  
> **硬件要求**：ES8388 音频编解码器（板载麦克风）  
> **前置条件**：sherpa-onnx 已安装，模型文件已就位

#### 6.2.1 工具概述

`asr_test.py` 位于 `interaction/car1_speech/tools/asr_test.py`，是一个**无 ROS2 依赖**的独立流式语音识别测试工具。

**识别引擎架构**：

```
ALSA 麦克风 (plughw:0,0)
  ↓
ES8388 Mixer (ALC + Noise Gate 自动增益)
  ↓
OnlineRecognizer (Zipformer + RKNN NPU)
  ↓
内置端点检测 (rule1/rule2/rule3)
  ↓
终端输出 (segment_id:识别文本)
```

**核心组件**：

| 组件 | 说明 |
|------|------|
| **OnlineRecognizer** | sherpa-onnx 流式识别器，使用 Zipformer transducer 模型 |
| **内置端点检测** | 基于静音时长的规则检测（rule1/rule2/rule3），判断句子边界 |
| **RKNN Provider** | RK3576 NPU 加速推理（encoder/decoder/joiner 均为 .rknn 格式） |
| **ALSA 输入** | 直接从 `plughw:0,0` 读取 16kHz 单声道音频 |
| **ALC + Noise Gate** | ES8388 硬件自动增益控制，动态适应距离/音量变化，静音时关闭 ADC 避免底噪 |
| **AudioMixer** | 统一管理 ES8388 mixer 设置（与 audio_test.py / kws_test.py 一致） |

**模型文件**（默认路径）：

| 文件 | 路径 | 格式 |
|------|------|------|
| tokens | `~/models/zipformer/tokens.txt` | 文本 |
| encoder | `~/models/zipformer/encoder.rknn` | RKNN (NPU) |
| decoder | `~/models/zipformer/decoder.rknn` | RKNN (NPU) |
| joiner | `~/models/zipformer/joiner.rknn` | RKNN (NPU) |

#### 6.2.2 基本用法

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_speech/tools

# 默认参数运行（ALC 自动增益 + RKNN 推理）— 推荐
python3 asr_test.py

# 关闭 ALC（使用固定 PGA 增益）
python3 asr_test.py --disable-alc

# 降低录音增益（如果环境噪声大）
python3 asr_test.py --capture-volume 88

# 使用热词（需要 modified_beam_search）
python3 asr_test.py --decoding-method modified_beam_search --hotwords-file hotwords.txt
```

启动后语音识别进入流式模式：
- `0:部分识别结果` — 实时刷新当前行，显示正在识别的内容
- 识别到句子结束后换行，开始新的 segment
- `[endpoint, no text]` — 端点触发但无文字（通常是环境噪声，偶尔出现正常）
- `Ctrl+C` — 退出

#### 6.2.3 完整参数参考

**模型与设备**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--tokens` | `~/models/zipformer/tokens.txt` | 词表文件 |
| `--encoder` | `~/models/zipformer/encoder.rknn` | 编码器模型 (RKNN) |
| `--decoder` | `~/models/zipformer/decoder.rknn` | 解码器模型 (RKNN) |
| `--joiner` | `~/models/zipformer/joiner.rknn` | 连接器模型 (RKNN) |
| `-D / --device` | `plughw:0,0` | ALSA PCM 录音设备 |
| `--provider` | `rknn` | 推理后端 (rknn / cpu)。注意：.rknn 模型无法用 cpu 运行 |
| `--num-threads` | `1` | RKNN 为 NPU 核心 mask，CPU 模式建议设 4 |
| `--sample-rate` | `16000` | 期望采样率 |

**解码参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--decoding-method` | `greedy_search` | 解码方式。使用热词时需改为 `modified_beam_search` |
| `--blank-penalty` | `0.25` | blank 符号惩罚。越低减少重复字，越高减少丢字。推荐 0.2-0.3 |
| `--hotwords-file` | `""`（禁用） | 热词文件路径（需 modified_beam_search） |
| `--hotwords-score` | `2.0` | 热词加分值（越高越倾向热词） |

**端点检测参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--rule1-min-trailing-silence` | `4.5` | 从头开始的最小静音时长触发端点（秒）。增大减少噪声误触发 |
| `--rule2-min-trailing-silence` | `1.5` | 识别出文字后的最小静音触发端点（秒）。增大防止句尾丢字 |
| `--rule3-min-utterance-length` | `300.0` | 最短语音长度触发端点（秒）。设很大=禁用此规则 |

**录音增益参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--capture-volume` | `95` | 麦克风 PGA 增益 0-100%（95% ≈ +22dB） |
| `--enable-alc` | `True`（默认启用） | 启用 ALC 自动增益 + Noise Gate |
| `--disable-alc` | 不传 | 关闭 ALC，使用固定 PGA 增益 |
| `--voice-preset` | 不传 | 应用 ASR 预设（mic +18dB, ALC off, 与 audio_test.py 一致） |
| `--card` | `0` | ALSA 声卡编号 |

**其他**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--feature-dim` | `80` | 模型特征维度 |
| `--debug` | 不传 | 启用调试输出 |

#### 6.2.4 常见问题

**Q: 频繁出现 `[endpoint, no text]`**

这是 VAD 误触发（环境噪声被误判为语音）。按优先级排查：

1. **降低增益**：
   ```bash
   python3 asr_test.py --capture-volume 88
   ```

2. **增加 rule1**（让系统等更久才判定为端点）：
   ```bash
   python3 asr_test.py --rule1-min-trailing-silence 5.5
   ```

3. **检查环境噪声**（关闭风扇/空调后是否减少）

正常水平：静音 30 秒内出现 ≤3 次属于可接受范围。

**Q: 句尾丢字（如"请把东西搬运到停放点"只识别出"请把东西搬运"）**

增大 rule2，给句尾更多识别时间：

```bash
python3 asr_test.py --rule2-min-trailing-silence 2.0
```

**Q: 识别结果重复字（如"小飞同学同学"）**

降低 blank_penalty：

```bash
# 从默认 0.25 降到 0.2
python3 asr_test.py --blank-penalty 0.2
```

blank_penalty 越高越容易重复，越低越容易丢字。推荐范围 0.2-0.3。

**Q: "小飞"被识别成"小飞机"或"VIVO"**

使用热词功能纠偏（需要 modified_beam_search）：

```bash
# 创建热词文件 hotwords.txt：
# 小 飞
# 小 飞 同 学
# 你 好 小 飞

python3 asr_test.py --decoding-method modified_beam_search \
                    --hotwords-file hotwords.txt --hotwords-score 2.5
```

注意：modified_beam_search 会增加 partial results 的更新频率（屏幕闪烁更多）。

**Q: 需要离麦克风很近才能识别**

这是单麦克风系统的物理限制。有效距离约 30-60cm。改善方法：

1. **启用 ALC**（默认已启用）：自动补偿距离变化
   ```bash
   python3 asr_test.py  # ALC 默认开启
   ```

2. **提高增益**（会增加误触发）：
   ```bash
   python3 asr_test.py --capture-volume 100
   ```

3. **降低增益**（减少误触发但距离更短）：
   ```bash
   python3 asr_test.py --capture-volume 88
   ```

**Q: 识别效果时好时差**

通常是距离/音量不一致导致。ALC（默认开启）能自动调节增益适应变化。如果效果仍不稳定：

- 确保说话时面对麦克风，距离 30-50cm
- 环境噪声尽量降低
- 如果 ALC 导致问题，可关闭回到固定增益：
  ```bash
  python3 asr_test.py --disable-alc --capture-volume 95
  ```

**Q: RKNN 模型加载失败**

```
RuntimeError: Load model from decoder.rknn failed: Protobuf parsing failed.
```

.rknn 模型**只能用 `--provider rknn`**，不能用 `--provider cpu`。CPU 模式需要 .onnx 格式的模型文件。

**Q: 输出 "VI"/"VO"/"VIVO" 等奇怪字符**

这是模型在低置信度时的"乱猜"（将环境噪声或句首气流声误识别为英文字符）。解决方法：
- 说话前稍等 1 秒再开口
- 降低增益减少底噪：`--capture-volume 88`
- 使用热词纠偏

#### 6.2.5 参数调优指南

**场景化推荐配置**：

| 场景 | 命令 |
|------|------|
| **日常使用（推荐）** | `python3 asr_test.py` |
| 安静环境 + 近距离 | `python3 asr_test.py --disable-alc --capture-volume 88` |
| 嘈杂环境 | `python3 asr_test.py --capture-volume 88 --rule1-min-trailing-silence 5.5` |
| 需要热词纠偏 | `python3 asr_test.py --decoding-method modified_beam_search --hotwords-file hotwords.txt` |
| 减少重复字 | `python3 asr_test.py --blank-penalty 0.2` |
| 减少句尾丢字 | `python3 asr_test.py --rule2-min-trailing-silence 2.0` |

**blank_penalty 调优**：

| blank_penalty | 重复字 | 丢字 | 适用场景 |
|---------------|--------|------|---------|
| 0.0 | 少 | 多 | 说话快速清晰 |
| **0.2-0.3** | **少** | **少** | **推荐平衡** |
| 0.5+ | 多 | 很少 | 不推荐 |

**ALC vs 固定增益**：

| 模式 | 优点 | 缺点 | 适用场景 |
|------|------|------|---------|
| **ALC 开启（默认）** | 自动适应距离/音量变化 | 偶尔句首延迟 | 日常使用，距离不固定 |
| 固定增益 (`--disable-alc`) | 无延迟，可预测 | 距离敏感 | 距离固定（如桌面麦克风） |

#### 6.2.5 使用官方 sherpa-onnx 命令行工具

除了上述 Python 脚本，sherpa-onnx 还提供了官方命令行工具，可以直接使用。

##### 6.2.5.1 基于 ALSA 的实时识别（推荐）

**sherpa-onnx-alsa**：直接从麦克风实时识别

```bash
# RKNN NPU 加速模式（.rknn 模型）
sherpa-onnx-alsa \
  --tokens=/home/iflytek/models/zipformer/tokens.txt \
  --encoder=/home/iflytek/models/zipformer/encoder.rknn \
  --decoder=/home/iflytek/models/zipformer/decoder.rknn \
  --joiner=/home/iflytek/models/zipformer/joiner.rknn \
  --num-threads=1 \
  --provider=rknn \
  --sample-rate=16000 \
  --decoding-method=greedy_search \
  plughw:0,0

# CPU 模式（.onnx 模型）
sherpa-onnx-alsa \
  --tokens=/home/iflytek/models/zipformer/tokens.txt \
  --encoder=/home/iflytek/models/zipformer/encoder.onnx \
  --decoder=/home/iflytek/models/zipformer/decoder.onnx \
  --joiner=/home/iflytek/models/zipformer/joiner.onnx \
  --num-threads=4 \
  --provider=cpu \
  --sample-rate=16000 \
  --decoding-method=greedy_search \
  plughw:0,0
```

**参数说明**：

| 参数 | 说明 |
|------|------|
| `--tokens` | 词表文件路径 |
| `--encoder/decoder/joiner` | 模型文件路径（.rknn 或 .onnx） |
| `--provider` | 推理后端（rknn / cpu） |
| `--num-threads` | RKNN 模式用 1，CPU 模式用 4 |
| `--sample-rate` | 采样率（通常 16000） |
| `--decoding-method` | 解码方法（greedy_search / modified_beam_search） |
| `plughw:0,0` | ALSA 设备名（最后一个位置参数，无 `--device` 前缀） |

**使用说明**：
- 启动后对着麦克风说话
- 实时显示部分识别结果
- 检测到端点后输出最终结果
- `Ctrl+C` 退出

##### 6.2.5.2 基于音频文件的离线识别

**sherpa-onnx**：识别已录制的 WAV 文件

```bash
# RKNN 模式（推荐参数）
sherpa-onnx \
  --tokens=/home/iflytek/models/zipformer/tokens.txt \
  --encoder=/home/iflytek/models/zipformer/encoder.rknn \
  --decoder=/home/iflytek/models/zipformer/decoder.rknn \
  --joiner=/home/iflytek/models/zipformer/joiner.rknn \
  --num-threads=2 \
  --provider=rknn \
  --device=1 \
  --decoding-method=modified_beam_search \
  --max-active-paths=4 \
  --temperature_scale=1.0 \
  test.wav

# CPU 模式
sherpa-onnx \
  --tokens=/home/iflytek/models/zipformer/tokens.txt \
  --encoder=/home/iflytek/models/zipformer/encoder.onnx \
  --decoder=/home/iflytek/models/zipformer/decoder.onnx \
  --joiner=/home/iflytek/models/zipformer/joiner.onnx \
  --num-threads=4 \
  --provider=cpu \
  --decoding-method=modified_beam_search \
  --max-active-paths=4 \
  test.wav
```

**参数说明**：

| 参数 | 说明 |
|------|------|
| `--num-threads` | RKNN 模式推荐 2，CPU 模式推荐 4 |
| `--device` | RKNN 设备 ID（1=自动选择 NPU 核心） |
| `--decoding-method` | modified_beam_search 比 greedy_search 准确度更高 |
| `--max-active-paths` | beam search 保留的路径数，越大越准确但越慢 |
| `--temperature_scale` | 温度缩放，1.0 为默认值 |

**准备测试音频**：

```bash
# 先录制一段测试音频（5 秒）
arecord -D plughw:0,0 -r 16000 -c 1 -f S16_LE -d 5 test.wav

# 播放确认录音质量
aplay -D plughw:0,0 test.wav

# 使用 sherpa-onnx 识别
sherpa-onnx \
  --tokens=/home/iflytek/models/zipformer/tokens.txt \
  --encoder=/home/iflytek/models/zipformer/encoder.rknn \
  --decoder=/home/iflytek/models/zipformer/decoder.rknn \
  --joiner=/home/iflytek/models/zipformer/joiner.rknn \
  --num-threads=2 \
  --provider=rknn \
  --device=1 \
  --decoding-method=modified_beam_search \
  --max-active-paths=4 \
  --temperature_scale=1.0 \
  test.wav
```

##### 6.2.5.3 检查工具是否安装

```bash
# 检查 sherpa-onnx-alsa
which sherpa-onnx-alsa

# 检查 sherpa-onnx-offline  
which sherpa-onnx-offline

# 如果没有，检查 Python 包中的命令行工具
python3 -c "import sherpa_onnx; print(sherpa_onnx.__file__)"
```

如果只安装了 Python 包，命令行工具可能在：
```bash
~/.local/bin/sherpa-onnx-alsa
# 或
/usr/local/bin/sherpa-onnx-alsa
```

##### 6.2.5.4 常见问题

**Q: 提示 `sherpa-onnx-alsa: command not found`**

sherpa-onnx 有两种安装方式：
1. **Python 包**：`pip install sherpa-onnx`（只有 Python API，无命令行工具）
2. **预编译二进制**：从 GitHub Release 下载（包含命令行工具）

如果只安装了 Python 包，可以：
- 使用 `asr_test.py` 脚本（功能相同）
- 或下载预编译版本：https://github.com/k2-fsa/sherpa-onnx/releases

**Q: ALSA 设备名错误**

如果 `plughw:0,0` 不工作，尝试：
```bash
# 查看可用设备
arecord -l

# 尝试其他设备名
sherpa-onnx-alsa ... hw:0,0
sherpa-onnx-alsa ... default
```

**Q: 识别结果是乱码或英文**

确认模型是否支持中文：
```bash
# 查看 tokens 文件是否包含中文字符
grep -P '[\p{Han}]' /home/iflytek/models/zipformer/tokens.txt | head -20
```

如果没有中文字符，说明模型是纯英文模型，需要下载中文 ASR 模型。

### 6.3 语音唤醒测试（kws_test.py）

> **目标**：使用 sherpa-onnx KeywordSpotter 实现流式关键词唤醒（离线，无需重训即可自定义唤醒词）  
> **所需时间**：10 分钟  
> **硬件要求**：ES8388 音频编解码器（板载麦克风）  
> **前置条件**：sherpa-onnx 已安装，KWS 模型文件已就位

#### 6.3.1 工具概述

`kws_test.py` 位于 `interaction/car1_speech/tools/kws_test.py`，是一个**无 ROS2 依赖**的独立关键词检测工具。它持续监听麦克风，检测到预配置的唤醒词时输出提示。

**唤醒引擎架构**：

```
ALSA 麦克风 → KeywordSpotter (持续流式解码) → 检测到关键词 → 打印 [Wake] + 重置 stream
```

**核心组件**：

| 组件 | 说明 |
|------|------|
| **KeywordSpotter** | sherpa-onnx 关键词检测器，使用 Zipformer transducer 模型 |
| **CPU 推理** | KWS 模型为 ONNX 格式（INT8 量化），在 CPU 上运行，体积 ~3MB |
| **ppinyin 建模** | 中文关键词使用拼音分词（声母+韵母+声调），支持自定义任意关键词 |
| **ALSA 输入** | 直接从 `plughw:0,0` 读取麦克风音频 |
| **持续监听** | 检测到关键词后自动重置 stream，立即开始下一次检测 |

**模型文件**（默认路径）：

| 文件 | 路径 | 格式 |
|------|------|------|
| tokens | `~/models/zipformer_kws/tokens.txt` | 文本（263 个 phoneme 标记） |
| encoder | `~/models/zipformer_kws/encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx` | ONNX INT8 |
| decoder | `~/models/zipformer_kws/decoder-epoch-13-avg-2-chunk-16-left-64.onnx` | ONNX |
| joiner | `~/models/zipformer_kws/joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx` | ONNX INT8 |

#### 6.3.2 唤醒词文件

唤醒词配置在 `keywords.txt`（与 `kws_test.py` 同目录），使用 **ppinyin 拼音分词** 格式：

```
x iǎo f ēi t óng x ué @小飞同学
n ǐ h ǎo x iǎo f ēi @你好小飞
x iǎo f ēi x iǎo f ēi @小飞小飞
```

**格式说明**：
- 用音标 token（空格分隔）：声母 + 韵母（带声调）
- `@` 后面的中文是**注释**，仅用于终端显示，模型只看前面的 ppinyin 部分
- 每行一个唤醒词
- 自定义唤醒词：按同样格式加一行即可，**无需重新训练**

**如何为新唤醒词生成 ppinyin 分词**（需安装 `pypinyin` 和 `sentencepiece`）：

```bash
# 安装依赖（一次性）
pip3 install pypinyin sentencepiece

# 用 sherpa-onnx 内置工具转换
python3 -c "
import sherpa_onnx
texts = ['小飞同学', '你好小飞', '小飞小飞']
result = sherpa_onnx.text2token(
    texts,
    tokens='/home/iflytek/models/zipformer_kws/tokens.txt',
    tokens_type='phone+ppinyin',
)
for t, r in zip(texts, result):
    print(' '.join(r) + f' @{t}')
"
```

#### 6.3.3 基本用法

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_speech/tools

# 默认参数运行（ALC 自动增益 + 默认唤醒词）
python3 kws_test.py

# 使用自定义唤醒词文件
python3 kws_test.py --keywords-file my_keywords.txt

# 提高检测灵敏度（更容易触发，但可能有误报）
python3 kws_test.py --keywords-threshold 0.15

# 提高严格度（减少误报，但可能漏检）
python3 kws_test.py --keywords-threshold 0.4

# 关闭 ALC 自动增益
python3 kws_test.py --disable-alc
```

检测到唤醒词时输出：

```
[Wake] 小飞同学
```

#### 6.3.4 完整参数参考

**模型与设备**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--tokens` | `~/models/zipformer_kws/tokens.txt` | 词表文件 |
| `--encoder` | `~/models/zipformer_kws/encoder-...-int8.onnx` | 编码器模型 (INT8) |
| `--decoder` | `~/models/zipformer_kws/decoder-...onxx` | 解码器模型 |
| `--joiner` | `~/models/zipformer_kws/joiner-...-int8.onnx` | 连接器模型 (INT8) |
| `--keywords-file` | `keywords.txt` | 唤醒词文件（与 kws_test.py 同目录） |
| `-D / --device` | `plughw:0,0` | ALSA PCM 录音设备 |
| `--num-threads` | `4` | 计算线程数 |
| `--sample-rate` | `16000` | 期望采样率 |

**检测参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--keywords-threshold` | `0.25` | 检测灵敏度。**越低越灵敏**（更容易触发但误报多），**越高越严格**（不易误报但可能漏检）。推荐范围 0.15~0.4 |
| `--keywords-score` | `1.0` | 每个 token 的额外加分（增大可提高置信度门槛） |
| `--num-trailing-blanks` | `2` | 关键词后需跟随的空白帧数。如果多个唤醒词共享前缀（如都有"小飞"），适当增大可减少串扰 |
| `--max-active-paths` | `4` | beam search 活跃路径数 |

**录音增益参数**（默认启用，自动拉满）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--capture-volume` | `88` | 麦克风 PGA 增益 0-100%（88% ≈ +18dB） |
| `--enable-alc` | 默认启用 | ALC 自动增益 + Noise Gate |
| `--disable-alc` | 不传 | 关闭 ALC，使用固定 PGA 增益 |
| `--voice-preset` | 不传 | 应用 ASR 预设（mic +18dB, ALC off） |
| `--card` | `0` | ALSA 声卡编号 |

#### 6.3.5 常见问题

**Q: 检测不到唤醒词**

按顺序排查：

1. **先验证麦克风工作正常**：
   ```bash
   python3 audio_test.py --mode record -d 5 -o test.wav
   python3 audio_test.py --mode play test.wav
   # 确认录音和播放都正常
   ```

2. **降低阈值提高灵敏度**：
   ```bash
   python3 kws_test.py --keywords-threshold 0.1
   ```

3. **发音清晰**：唤醒词模型对发音标准度敏感，中文带声调的 ppinyin 建模要求音调准确。如果阈值太低导致误报太多，保持阈值在 0.2~0.3 之间，确保发音清晰、声调准确。

**Q: 误触发太多（环境噪声被误识别为唤醒词）**

```bash
# 提高检测阈值
python3 kws_test.py --keywords-threshold 0.4

# 同时增大 trailing blanks 防止部分匹配
python3 kws_test.py --keywords-threshold 0.4 --num-trailing-blanks 4
```

**Q: 说"小飞小飞"却触发了"小飞同学"（或反过来）**

多个唤醒词共享前缀（都含"小飞"）时可能串扰。增大 `--num-trailing-blanks`：

```bash
python3 kws_test.py --num-trailing-blanks 4
```

**Q: 如何添加新唤醒词**

直接编辑 `keywords.txt`，按 ppinyin 格式加一行：

```bash
# 先用 text2token 转换获取 ppinyin 分词
python3 -c "
import sherpa_onnx
print(sherpa_onnx.text2token(
    ['打开灯'],
    tokens='/home/iflytek/models/zipformer_kws/tokens.txt',
    tokens_type='phone+ppinyin',
)[0])
"
```

将输出的 token 序列复制到 `keywords.txt`，末尾加 ` @中文标注` 即可。

**Q: 为什么 KWS 用 CPU 而不是 RKNN**

KWS 模型（Zipformer）是 ONNX 格式且已 INT8 量化，~3MB 大小在 ARM CPU 上运行效率极高（RTF < 0.1），不需要 NPU 加速。CPU 推理延迟低且不占用 NPU 资源，方便与 ASR 识别器（用 RKNN）同时运行。

**Q: 能否与 ASR 同时运行**

可以。KWS 和 ASR 是独立的 sherpa-onnx 实例，互不冲突：

```bash
# 终端 1：唤醒词检测（常驻后台）
python3 kws_test.py

# 终端 2：语音识别（唤醒后手动启动或由程序自动拉起）
python3 asr_test.py
```

ALSA `plughw:0,0` 支持多个进程同时打开（dmix/dsnoop 混音），两个程序可共用同一麦克风。

---

### 6.4 本地大模型服务（llm_server.py + llm_test.py）

> **目标**：在 RK3576 上部署 Qwen3.5-0.8B 大模型，通过 OpenAI 兼容 API 提供本地推理服务  
> **所需时间**：10 分钟  
> **硬件要求**：RK3576 板卡，RKLLM 运行时已安装  
> **前置条件**：librkllmrt.so 已安装到 `/usr/lib/`，RKLLM 模型已转换

#### 6.4.1 工具概述

`llm_server.py` 和 `llm_test.py` 位于 `interaction/car1_llm/tools/`，是**无 ROS2 依赖**的独立工具，实现客户端-服务端架构的本地大模型对话。

**服务架构**：

```
┌────────────────────────────────┐     HTTP (OpenAI API)
│  llm_server.py (服务端)         │ ◄─────────────────────  llm_test.py (客户端)
│                                │                         (交互式对话)
│  RKLLM Runtime (NPU 推理)      │     /v1/chat/completions
│  Qwen3.5-0.8B  W4A16 量化     │     SSE 流式输出
│  Flask HTTP Server :8080       │
└────────────────────────────────┘
```

**核心组件**：

| 组件 | 说明 |
|------|------|
| **RKLLM Runtime** | Rockchip NPU 大模型推理引擎（librkllmrt.so） |
| **Qwen3.5-0.8B** | 通义千问 0.8B 参数模型，W4A16 量化（~500MB），专为端侧优化 |
| **Flask Server** | 轻量 HTTP 服务，OpenAI 兼容 `/v1/chat/completions` 接口 |
| **流式输出 (SSE)** | 逐 token 实时返回，体验接近云端大模型 |
| **频率锁定** | 可选的 NPU/CPU/GPU/DDR 频率锁定，提升推理性能 |

**模型文件**：

| 文件 | 路径 | 说明 |
|------|------|------|
| RKLLM 模型 | `~/models/rkllm/Qwen3.5-0.8B_w4a16_g128_rk3576.rkllm` | W4A16 量化文本模型 |
| 视觉编码器 | `~/models/rkllm/Qwen3.5-0.8B_vision_rk3576.rknn` | 视觉编码模型（暂未启用） |

#### 6.4.2 基本用法

**启动服务端**：

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_llm/tools

# 普通启动（不做频率锁定，可直接运行）
python3 llm_server.py

# 启动 + 锁定 NPU/CPU/GPU/DDR 到最高频率（会提示输入 sudo 密码）
python3 llm_server.py --fix-freq

# 使用自定义模型
python3 llm_server.py --model /path/to/custom.rkllm --platform rk3576
```

服务端启动成功后：

```
============================================================
RKLLM Server
============================================================
  Model:    Qwen3.5-0.8B_w4a16_g128_rk3576
  Platform: rk3576
  Fix freq: no (use --fix-freq to lock clocks)
  API:      http://0.0.0.0:8080/v1/chat/completions
============================================================
 * Running on http://127.0.0.1:8080
 * Running on http://10.1.134.82:8080
```

**启动客户端**（另开终端）：

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_llm/tools

# 交互式对话（流式输出，逐 token 实时显示）
python3 llm_test.py

# 连接远程服务器
python3 llm_test.py --server http://192.168.1.100:8080

# 单次提问（非交互模式）
python3 llm_test.py --prompt "你好，介绍一下自己"

# 非流式（等完整回复后一次性输出）
python3 llm_test.py --no-stream
```

**客户端交互说明**：

```
You > 你是谁？
AI  > 我是通义千问，由阿里云开发的大型语言模型...

You > 什么是麦克纳姆轮？
AI  > 麦克纳姆轮是一种全向轮...

You > clear                      # 清除对话历史
You > quit                       # 退出（也可用 exit / q / Ctrl+C）
```

#### 6.4.3 频率锁定（可选）

`--fix-freq` 参数会在启动前禁用 CPU 深度睡眠并将 NPU/CPU/GPU/DDR 锁定到最高频率，提升推理性能。

| 时钟域 | 锁定频率 | 说明 |
|--------|---------|------|
| NPU | 950 MHz | 最高推理吞吐 |
| CPU (A55) | 2.0 GHz | 后处理加速 |
| CPU (A72) | 2.2 GHz | 后处理加速 |
| GPU | 900 MHz | 避免访存瓶颈 |
| DDR | 1.85 GHz | 避免访存瓶颈 |

> **注意**：频率锁定通过写入 `/sys/` 伪文件系统实现，需要 root 权限。`llm_server.py` 内部用 `sudo` 仅执行频率锁定这一步，server 进程本身以普通用户运行。频率设置在重启后自动恢复默认值，不会永久修改硬件。

```bash
# 带频率锁定的启动（会提示输入 sudo 密码）
python3 llm_server.py --fix-freq
```

频率锁定脚本保存在同目录的 `fix_freq_rk3576.sh`，可单独运行：

```bash
sudo bash fix_freq_rk3576.sh
```

#### 6.4.4 完整参数参考

**服务端（llm_server.py）**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--model` | `~/models/rkllm/Qwen3.5-0.8B_w4a16_g128_rk3576.rkllm` | RKLLM 模型路径 |
| `--platform` | `rk3576` | 目标平台 (rk3588 / rk3576 / rv1126b / rk3562) |
| `--fix-freq` | 不传 | 锁定 NPU/CPU/GPU/DDR 到最高频率 |
| `--lora-model` | 不传 | 可选的 LoRA 适配器路径 |
| `--prompt-cache` | 不传 | 可选的 prompt cache 文件路径 |

**客户端（llm_test.py）**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--server` | `http://127.0.0.1:8080` | RKLLM 服务器地址 |
| `--prompt` | 不传 | 单次提问（非交互模式） |
| `--no-stream` | 不传 | 关闭流式输出，等完整回复 |
| `--temperature` | `0.8` | 采样温度（越高越随机，0.5 更确定） |
| `--top-p` | `0.9` | Nucleus 采样阈值 |
| `--max-tokens` | `4096` | 最大生成 token 数 |
| `--repeat-penalty` | `1.1` | 重复惩罚（>1.0 减少重复） |
| `--enable-thinking` | 不传 | 启用深度思考模式（Qwen3 特性） |
| `--timeout` | `120` | HTTP 请求超时秒数 |

#### 6.4.5 常见问题

**Q: 提示 `ModuleNotFoundError: No module named 'flask'`**

Flask 需要安装到 ros2_venv 中：

```bash
pip3 install flask
```

确认安装成功：

```bash
python3 -c "from flask import Flask; print('Flask OK')"
```

**Q: `Permission denied` 写入 `/sys/` 失败**

这些报错来自 `flask_server.py` 内部第二次调用 fix_freq 脚本，假 sudo 没有 root 权限。**不影响功能**——真正的频率锁定已在 server 启动前完成（`--fix-freq` 时）或不需要（不加 `--fix-freq` 时）。

如果这些报错干扰阅读，用不带 `--fix-freq` 的方式启动即可：

```bash
python3 llm_server.py
```

**Q: rkllm init 失败**

```
E rknn: rknn_init, load model failed!
```

模型文件路径错误或模型与平台不匹配：

```bash
# 确认模型文件存在
ls -lh ~/models/rkllm/

# 确认 librkllmrt.so 在 /usr/lib/ 下
ls -l /usr/lib/librkllmrt.so
```

**Q: 推理速度慢**

- 使用 `--fix-freq` 锁定频率：`python3 llm_server.py --fix-freq`
- 模型加载需要 ~20 秒，后续推理逐 token 流式输出
- W4A16 量化模型在 RK3576 上典型推理速度：~8-12 tokens/s

**Q: 返回内容重复**

调大 `--repeat-penalty` 减少重复：

```bash
python3 llm_test.py --repeat-penalty 1.3
```

或降低 `--temperature` 让输出更确定：

```bash
python3 llm_test.py --temperature 0.5
```

**Q: 能否与 KWS / ASR 同时运行**

可以。三者是独立的进程，互不冲突：

```bash
# 终端 1：唤醒词检测（常驻后台）
python3 kws_test.py

# 终端 2：RKLLM 大模型服务
python3 llm_server.py

# 终端 3：语音识别（按需启动）
# python3 asr_test.py

# 终端 4：LLM 客户端（按需启动）
python3 llm_test.py
```

**Q: 能否作为 ROS2 节点使用**

`car1_llm` 包中已包含 ROS2 集成：
- `car1_llm/chat_node.py`：ROS2 大模型对话节点
- `car1_llm/skill_executor.py`：技能执行器
- `launch/llm.launch.py`：启动文件
- `config/llm.yaml`：LLM 配置
- `config/skills.yaml`：技能定义

ROS2 集成的前提是 RKLLM server 已在后台运行。

**Q: 什么是预填充延迟？**

tools 模式下用户输入后停顿几秒才出结果，这是 **Transformer 预填充 (prefill)** 引起的。预填充阶段模型需要一次算完所有输入（工具定义 + 对话历史 + 用户问题），这个阶段**没有任何输出**。RK3576 上 0.8B 模型的预填充约需 4-6 秒，首 token 出来后的生成速度为 8-12 tokens/s。

这已经是 W4A16 量化 + RK3576 NPU 的硬件上限，代码层面已无可优化空间。如需更低延迟，考虑使用更小的模型（0.5B/0.3B）。

---

### 6.5 Function Calling 工具调用

> **目标**：让本地大模型自动调用预定义函数来回答问题（获取时间、查系统状态、计算器等）  
> **所需时间**：5 分钟  
> **前置条件**：RKLLM server 已在后台运行

#### 6.5.1 工作原理

Function calling 让模型在遇到它无法直接回答的问题时，输出一个结构化调用指令 `<tool_call>`，由客户端执行对应函数后返回结果。

```
用户: "现在几点"        → 模型: <tool_call>{"name":"get_current_time"}</tool_call>
                                 ↓
                        客户端: 执行 get_current_time() → {"datetime":"2026-07-14 18:30:00",...}
                                 ↓
                        直接返回 JSON 给用户
```

**关键文件**：

| 文件 | 说明 |
|------|------|
| `tools_config.py` | 工具定义（TOOLS schema + FUNCTION_MAP 实现 + 解析器） |
| `llm_test.py` | 客户端，`--mode tools` 进入 function calling 模式 |

#### 6.5.2 使用方式

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_llm/tools

# 先确保 server 在运行
python3 llm_server.py

# 另一个终端，进入 function calling 模式
python3 llm_test.py --mode tools
```

```
You > 现在几点
AI  > {
  "datetime": "2026-07-14 18:30:00",
  "date": "2026-07-14",
  "time": "18:30:00",
  "weekday": "Tuesday"
}

You > 3.14 * 2 + 100
AI  > {
  "expression": "3.14 * 2 + 100",
  "result": 106.28
}
```

输入 `tools` 查看可用工具列表，`clear` 清除对话，`quit` 退出。

#### 6.5.3 预置工具

| 工具 | 函数名 | 说明 |
|------|--------|------|
| 获取时间 | `get_current_time` | 返回当前日期、时间、星期 |
| 系统信息 | `get_system_info` | 主机名、运行时间、CPU 温度 |
| 计算器 | `calculator` | 安全计算数学表达式 |
| 电池状态 | `get_battery_status` | 电压、电量百分比 |

#### 6.5.4 添加自定义工具

只需在 `tools_config.py` 中做三件事：

**Step 1：实现函数**

```python
def get_weather(city: str):
    # 实际项目中可调用天气 API
    return {"city": city, "temperature": 25, "weather": "晴"}
```

**Step 2：注册到 FUNCTION_MAP**

```python
FUNCTION_MAP = {
    ...
    "get_weather": get_weather,   # 新增
}
```

**Step 3：添加到 TOOLS schema**

```python
TOOLS = [
    ...
    {"type": "function", "function": {"name": "get_weather", "description": "天气查询"}},
]
```

**描述文本尽量简短**（1-2 个词），因为每条工具描述都会增加预填充阶段的输入 token，延长等待时间。

#### 6.5.5 性能说明

| 指标 | 数值 |
|------|------|
| 预填充延迟 | ~5-6 秒（硬件限制） |
| 首 token 后生成速度 | 8-12 tokens/s |
| 工具调用总耗时 | ~6 秒（单轮，含预填充 + 生成） |

tools 模式已将输入精简到极致（无 system prompt，tools schema 仅 name + 1 词描述，max_tokens=128），单轮调用无二轮推理。剩余延迟均为 RK3576 NPU 的硬件预填充吞吐量上限。

#### 6.5.6 常见问题

**Q: 模型不调用工具，直接凭空回答**

小模型（0.8B）的 function calling 能力有限，某些问法可能不被识别为工具调用。试试换一下表述：

- `几点了` → `现在几点` / `获取当前时间`
- `电池` → `查看电池状态` / `读取电量`

**Q: 模型输出了格式错误**

小模型偶尔会输出不规范的 `<tool_call>` 标签，parse 失败时 `llm_test.py` 会直接把原始文本打印出来。重新问一次通常能正常。

**Q: 预填充太慢**

参考 6.4.5 中关于预填充延迟的说明。工具定义越少、越短，等待越短。不需要的工具可以从 TOOLS 列表中注释掉。

---

### 6.6 语音合成测试（tts_test.py）

> **目标**：使用 sherpa-onnx Matcha-TTS 实现中文语音合成，将文本转化为语音并播放  
> **所需时间**：10 分钟  
> **硬件要求**：ES8388 音频编解码器（板载喇叭）  
> **前置条件**：sherpa-onnx 已安装，TTS 模型文件已就位

#### 6.6.1 工具概述

`tts_test.py` 位于 `interaction/car1_speech/tools/tts_test.py`，是一个**无 ROS2 依赖**的独立语音合成工具。它将输入的中文或中英混合文本合成为语音，通过 ES8388 喇叭播放。

**合成引擎架构**：

```
输入文本 → Matcha-TTS (声学模型) → Vocos (声码器) → 16kHz float32 音频
                                                          ↓
                                                  gain 数字增益
                                                          ↓
                                             write_wav_stereo (立体声 WAV)
                                                          ↓
                                                   aplay → ES8388 喇叭
```

**核心组件**：

| 组件 | 说明 |
|------|------|
| **Matcha-TTS** | 基于 Matcha 声学模型的中英双语 TTS，支持声调准确的普通话合成 |
| **Vocos Vocoder** | 声码器，将梅尔频谱转换为 16kHz 单声道 float32 音频 |
| **数字增益** | 可调倍数放大（tanh 软削波），补偿 TTS 输出偏低的音量 |
| **立体声转换** | ES8388 I2S 硬连接立体声，工具自动将单声道复制到左右声道 |
| **ALSA 播放** | 通过 aplay 播放标准 WAV，与 audio_test.py 使用相同的 mixer 设置 |

**模型文件**：

| 文件 | 路径 | 说明 |
|------|------|------|
| 声学模型 | `~/models/tts/model-steps-3.onnx` | Matcha-TTS 声学模型 |
| 声码器 | `~/models/tts/vocos-16khz-univ.onnx` | Vocos 16kHz 声码器 |
| 词典 | `~/models/tts/lexicon.txt` | 中文发音词典 |
| 词表 | `~/models/tts/tokens.txt` | 音素标记文件 |
| eSpeak | `~/models/tts/espeak-ng-data/` | 多语言发音数据 |
| 规则 FST | `phone-zh.fst` / `date-zh.fst` / `number-zh.fst` | 中文正音规则 |

#### 6.6.2 基本用法

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_speech/tools

# 默认文本合成并播放（gain=2.0, playback-volume=100%）
python3 tts_test.py --text "你好，欢迎使用语音合成"

# 调节语速
python3 tts_test.py --text "现在时间是下午三点" --speed 1.2

# 交互模式（逐行输入文字实时合成）
python3 tts_test.py -i

# 只保存 WAV 文件，不播放
python3 tts_test.py --text "你好" -o hello.wav --no-play
```

启动后输出：

```
  Playback volume: 100% (0.0 dB)
Loading TTS model...
  Ready.

Text:  你好，欢迎使用语音合成
Speed: 1.0
Audio:  4.26s  (16000 Hz, 68128 samples)
Gen:    2.18s
RTF:    0.512  (faster than real-time)
Output: /tmp/tts_output.wav
```

**RTF (Real-Time Factor)**：生成耗时 / 音频时长。< 1.0 表示快于实时，0.5 表示只需一半时间。

#### 6.6.3 音量调节

TTS 匹配人声特性的输出波形幅度偏保守（RMS 约 -17 dBFS）。工具提供两个独立旋钮：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--gain` | `2.0` | **数字增益**。1.0=原始输出的音量。建议范围 1.0~4.0。越大越响，但也更容易削波失真 |
| `--playback-volume` | `100` | **硬件播放音量**。与 audio_test.py 一致：100%=0dB，50%=-6dB。控制 ES8388 Output 1/2 + PCM 的 dB 增益 |

```bash
# 默认音量（gain=2.0, playback=100%）
python3 tts_test.py --text "你好"

# 更大声
python3 tts_test.py --text "你好" --gain 3.0

# 降低音量
python3 tts_test.py --text "你好" --gain 1.0 --playback-volume 80
```

**Mixer 控制说明**：ES8388 的播放链路只有三个可调节的 mixer 控制项——`PCM`（数字域）、`Output 1/2`（模拟域输出电平）、`spk switch`（功放开关，aplay 运行时内核自动启用）。与 audio_test.py 使用完全相同的 dB 方式设置，没有再多的隐藏增益点。

**TTS 音量与麦克风录音对比**：TTS 原始输出已经比板载麦克风录音回放响亮约 6dB。加上默认 gain=2.0 后，比录音回放响亮约 12dB。如果还觉得小，就是这块板子喇叭的物理极限了。

#### 6.6.4 交互模式命令

```bash
python3 tts_test.py -i
```

| 输入 | 说明 |
|------|------|
| 任意文本 | 合成并播放 |
| `vol=80` | 实时调节播放音量 |
| `speed=1.2` | 调节语速 |
| `quit` / `exit` / `q` | 退出 |
| `Ctrl+C` | 退出 |

#### 6.6.5 完整参数参考

**模型与设备**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--acoustic-model` | `~/models/tts/model-steps-3.onnx` | 声学模型路径 |
| `--vocoder` | `~/models/tts/vocos-16khz-univ.onnx` | 声码器路径 |
| `--lexicon` | `~/models/tts/lexicon.txt` | 发音词典 |
| `--tokens` | `~/models/tts/tokens.txt` | 音素词表 |
| `--data-dir` | `~/models/tts/espeak-ng-data/` | eSpeak 数据目录 |
| `--rule-fsts` | 三个 FST 文件 | 中文正音规则 |

**输入与输出**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--text` / `-t` | 无 | 要合成的文本 |
| `-i` / `--interactive` | 不传 | 交互模式 |
| `--speed` | `1.0` | 语速，0.5~2.0 |
| `--sid` | `0` | 说话人 ID |
| `--output` / `-o` | `/tmp/tts_output.wav` | 输出 WAV 文件 |
| `--no-play` | 不传 | 不播放，只保存文件 |

**音量与增益**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--gain` | `2.0` | 数字增益倍数，1.0~4.0 |
| `--playback-volume` | `100` | 播放音量 0-100%，100%=0dB |
| `-D` / `--device` | `plughw:0,0` | ALSA 播放设备 |

**性能**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--num-threads` | `4` | 推理线程数 |
| `--max-num-sentences` | `1` | 每批最大句子数 |
| `--debug` | 不传 | 启用调试输出 |

#### 6.6.6 常见问题

**Q: 播放速度异常（变速 / 快进）**

ES8388 I2S 硬连接立体声，单声道 WAV 的 byte rate 是立体声的一半，导致时钟错配。工具已通过 `write_wav_stereo()` 将单声道复制到左右声道，写出标准立体声 WAV 后再播放。

如果仍然异常，检查 mixer 状态并用 aplay 直接测试：

```bash
python3 audio_test.py --list-mixer | grep -E "PCM|Output"
aplay -D plughw:0,0 /tmp/tts_output.wav
```

**Q: 声音太小**

默认 gain=2.0，可以继续加大：

```bash
python3 tts_test.py --text "你好" --gain 3.0
```

如果 gain 超过 4.0 仍然偏小，是 ES8388 板载喇叭的物理功率限制。可以考虑外接有源音箱到耳机插孔。

**Q: 声音失真 / 截幅**

gain 过高导致削波失真。降低 gain：

```bash
python3 tts_test.py --text "你好" --gain 1.5
```

**Q: 报错 `Rule fst does not exist`**

FST 文件缺失或路径不对：

```bash
ls ~/models/tts/phone-zh.fst ~/models/tts/date-zh.fst ~/models/tts/number-zh.fst
```

如果不存在，重新下载模型或去掉 `--rule-fsts` 参数（无正音规则的 TTS 仍然可以工作，但数字/时间朗读可能不标准）。

**Q: 能否同时使用 TTS + ASR + KWS**

可以。TTS（CPU 推理）与 ASR（RKNN 推理）、KWS（CPU 推理）均可同时运行，互不冲突。ASR/KWS 使用麦克风（录音链路），TTS 使用喇叭（播放链路）。


### 6.7 完整语音交互（voice_interaction.py）

> **目标**：实现完整的语音交互闭环：唤醒词检测 → 语音识别 → 大模型对话 → 语音合成  
> **所需时间**：15 分钟  
> **硬件要求**：ES8388 音频编解码器（板载麦克风+喇叭）  
> **前置条件**：6.1-6.6 所有工具测试通过，模型文件已就位

#### 6.7.1 系统概述

`voice_interaction.py` 位于 `interaction/car1_interaction/tools/`，是一个**无 ROS2 依赖**的独立语音交互工具，集成了前面所有测试工具的功能。

**完整架构**：

```
ALSA 麦克风 (持续捕获) → 音频队列
  ↓
状态机循环读取音频队列
  │
  ├─ IDLE: KWS 检测唤醒词
  │    ↓ 检测到唤醒词
  │    播放提示音 "我在"
  │    排空音频队列（避免提示音回声）
  │    ↓
  ├─ LISTENING: ASR 流式识别
  │    ↓ 内置端点检测触发
  │    过滤黑名单（VIVO/OK等噪音）
  │    ↓ 识别结果有效
  ├─ SPEAKING: LLM + TTS 流式输出
  │    LLM 逐 token 生成 → 句子切分 → TTS 合成 → 立即播放
  │    KWS 在后台继续运行（支持播放时打断）
  │    ↓ 回答完成
  └─ 回到 IDLE，等待下次唤醒
```

**核心特性**：

| 特性 | 说明 |
|------|------|
| **流式对话** | LLM 边生成边播放，无需等完整回答 |
| **低延迟** | 提示音预加载 PCM，播放管道持久连接 |
| **回声抑制** | 播放提示音后双重排空音频队列，避免 ASR 误识别回声 |
| **黑名单过滤** | 拒绝常见误识别（VIVO/OPPO/HI/OK/单字母等） |
| **自动端点** | ASR 内置端点检测，无需外部 VAD |
| **可打断** | SPEAKING 时 KWS 继续运行，支持"小飞同学"打断当前回答 |
| **自动清理** | 程序退出时自动停止 LLM 服务，无残留进程 |

#### 6.7.2 基本用法

```bash
cd ~/ros2_ws/src/xflz_eai_car1/interaction/car1_interaction/tools

# 默认启动（自动启动内嵌 LLM 服务）
python3 voice_interaction.py

# 连接外部 LLM 服务
python3 voice_interaction.py --no-server

# 调试模式
python3 voice_interaction.py --debug
```

**启动成功标志**：

```
19:30:15 [KWS  ] 唤醒词: 小飞同学
19:30:15 [KWS  ] 唤醒词: 你好小飞
19:30:15 [KWS  ] 唤醒词: 小飞小飞
19:30:16 [ASR  ] provider=rknn endpoint: r1=4.5s r2=1.5s
19:30:22 [TTS  ] speed=1.0 gain=1.0
19:30:22 [LLM  ] 启动 LLM: Qwen3.5-0.8B... (端口 8080)
19:30:35 [LLM  ] LLM 服务就绪
19:30:35 [LLM  ] server=http://127.0.0.1:8080
19:30:35 [Main ] 初始化完成
19:30:35 [Main ] 录音线程: plughw:0,0
19:30:35 [Main ] ==================================================
19:30:35 [Main ] 就绪 — 说唤醒词开始对话, Ctrl+C 退出
19:30:35 [Main ] ==================================================
```

#### 6.7.3 对话示例

**对话流程**：

```
用户: "小飞同学"          ← 说唤醒词
系统: [提示音] "我在"     ← 立即播放提示音
19:30:40 [KWS  ] 唤醒: 小飞同学
19:30:40 [Main ] IDLE → LISTENING

用户: "你是谁"            ← 说问题
[ASR] 你是谁             ← 实时部分识别结果
19:30:43 [ASR  ] 识别: 你是谁
19:30:43 [Main ] LISTENING → SPEAKING (你是谁)

系统: "我是小飞，一个智能语音助手..."  ← 边生成边播放
[LLM] 我是小飞              ← 流式生成日志
[LLM] ，一个智能语音助手
[LLM] ...

19:30:50 [Main ] SPEAKING → IDLE    ← 回答完成
```

**打断示例**：

```
系统: "关于麦克纳姆轮的工作原理，首先需要了解..."  ← 正在播放长回答
用户: "小飞同学"                              ← 说唤醒词打断
19:31:05 [KWS  ] 唤醒: 小飞同学
19:31:05 [Main ] SPEAKING → LISTENING          ← 立即停止播放

用户: "简短回答"
...
```

#### 6.7.4 配置文件

配置文件：`interaction/car1_interaction/tools/config.yaml`

**关键配置项**：

```yaml
# --- 音频设备 ---
audio:
  capture_volume: 100      # 麦克风增益 0-100%（100% ≈ +24dB）
  playback_volume: 100     # 扬声器音量 0-100%（100% = 0dB）
  mic_boost: true          # 启用 ALC 自动增益

# --- 唤醒词检测 ---
kws:
  threshold: 0.25          # 检测灵敏度（越低越灵敏，0.15-0.4）

# --- 语音识别 ---
asr:
  rule1_silence: 4.5       # 无文本时端点静音（秒）
  rule2_silence: 1.5       # 有文本后端点静音（秒）
  blank_penalty: 0.25      # blank 惩罚（0.2-0.3 推荐）

# --- 大语言模型 ---
llm:
  server_url: "http://127.0.0.1:8080"   # 修改端口避免冲突
  temperature: 0.8
  system_prompt: "你是小飞，一个智能语音助手。请用简洁口语化的中文回答问题，每次回答控制在2-3句话。重要：必须用中文回答。"

# --- LLM 服务进程 ---
llm_server:
  auto_start: true         # 是否自动启动 LLM 服务
  auto_kill_conflict: true # 端口被占用时自动清理
  fix_freq: false          # 是否锁定 NPU/CPU/GPU 频率（需 sudo）

# --- 语音合成 ---
tts:
  speed: 1.0               # 语速
  gain: 1.0                # 数字增益（>1.0 会用 tanh 软削波）
```

#### 6.7.5 完整参数参考

| 参数 | 说明 |
|------|------|
| `--config` | 配置文件路径（默认 `config.yaml`） |
| `--no-server` | 不启动内嵌 LLM 服务（连接外部服务） |
| `--debug` | 启用调试日志 |

#### 6.7.6 关键技术点

##### 6.7.6.1 提示音回声抑制

**问题**：播放提示音"我在"后，喇叭声音会被麦克风捕获，导致 ASR 误识别为"VIVO"/"会"等噪音。

**解决方案**（双重排空队列）：

```python
1. 播放提示音 PCM（预加载，无文件 I/O）
2. 等待 prompt_duration + ALSA_latency + room_echo_tail（~1.5秒）
3. 第一次排空音频队列（丢弃播放期间的音频）
4. 等待 0.2 秒（让捕获线程填入尾声回声）
5. 第二次排空音频队列（最终清理）
6. 创建 ASR stream（保证首帧是纯净语音）
```

**效果**：ASR 误识别率从 ~30% 降至 ~2%（偶尔出现"会"等单字，被黑名单过滤）。

##### 6.7.6.2 ASR 黑名单过滤

**常见误识别模式**：

- 提示音回声 → "VIVO" / "IVO" / "VI VO"
- 环境噪声 → "OPPO" / "HI" / "OK" / "会"
- 句首气流声 → 单字母 "A" / "B" / "V"

**过滤规则**：

```python
# 黑名单（大小写不敏感，去空格）
VIVO, IVO, VI, V I, V IVO, VI VO, OPPO, HI, HELLO, OK, YES, NO, A-Z, ...

# 规则：纯 ASCII 单词（无空格）且 < 6 字符 → 拒绝
"vivo" → 拒绝（5字符纯ASCII无空格）
"hello world" → 通过（有空格）
"你好" → 通过（中文）
"会" → 拒绝（1字符）
```

##### 6.7.6.3 LLM + TTS 流式pipeline

**传统方案（延迟高）**：

```
LLM 生成完整回答 → 保存文件 → TTS 合成 → 播放
延迟 = LLM 生成时间 + TTS 合成时间 + 文件 I/O
```

**流式方案（低延迟）**：

```
LLM 逐 token 生成
  ↓ 检测到句子结束（。！？；，、）
TTS 合成当前句子
  ↓ 立即写入 aplay stdin pipe
开始播放（无需等后续句子）
  ↓ 并行：LLM 继续生成下一句
```

**延迟对比**（"介绍一下麦克纳姆轮"，3 句话回答）：

| 方案 | 首句播放延迟 | 总延迟 |
|------|-------------|--------|
| 传统 | ~15 秒 | ~20 秒 |
| 流式 | ~6 秒 | ~15 秒 |

**实现要点**：

- 句子切分正则：`([。！？；，、\n]|[.?!,](?=\s|$))`（包含逗号，减少单句长度）
- TTS 合成前清洗：去除 markdown、emoji、引号、括号等
- 播放管道持久连接：aplay stdin pipe 一直开着，避免每句重启进程

##### 6.7.6.4 端口冲突自动清理

**问题**：上次运行残留的 LLM 服务进程占用端口 8080。

**解决方案**：

```python
1. 检测端口是否被占用（socket connect）
2. 尝试访问 /v1/models（验证是否为有效 LLM 服务）
3. 如果不可用：
   - auto_kill_conflict=true → 自动运行 lsof -ti :8080 | xargs kill -9
   - auto_kill_conflict=false → 提示用户手动清理
4. 重试启动
```

#### 6.7.7 常见问题

**Q: 提示音不播放 / TTS 不播放**

检查播放设备是否被占用：

```bash
fuser /dev/snd/pcmC0D0p
# 如果有输出，运行清理：
fuser -k /dev/snd/*
```

或者运行时日志会提示：

```
[Main] 播放设备被占用，清理冲突进程...
[Main] 已清理 aplay 进程
```

**Q: 频繁出现 `[ASR] 识别: 会` 等单字**

这是提示音回声残留，被黑名单过滤了（不会进入 LLM）。正常现象，出现频率 <5% 可接受。

如果频率过高，增大 `rule1_silence` 减少误触发：

```yaml
asr:
  rule1_silence: 5.5   # 从 4.5 增加到 5.5
```

**Q: ASR 识别到"VIVO"**

黑名单已包含 VIVO 及其变种，会被过滤。如果仍然出现，检查代码是否为最新版本（2026-07 之后的版本）。

**Q: LLM 回答是英文**

修改 `config.yaml` 的 system_prompt：

```yaml
llm:
  system_prompt: "你是小飞，一个智能语音助手。请用简洁口语化的中文回答问题，每次回答控制在2-3句话。重要：必须用中文回答。"
```

**Q: TTS 播放声音失真 / 破音**

降低数字增益：

```yaml
tts:
  gain: 0.8   # 从 1.0 降低
```

或降低硬件音量：

```yaml
audio:
  playback_volume: 80   # 从 100 降低
```

**Q: 端口 8080 被占用，无法启动**

自动清理功能已启用（`auto_kill_conflict: true`），会自动清理。如果仍然失败，手动清理：

```bash
lsof -ti :8080 | xargs kill -9
```

或修改端口：

```yaml
llm:
  server_url: "http://127.0.0.1:8081"  # 改成 8081
```

**Q: 句尾经常被截断（如"请把东西搬运到停放点"只识别出"请把东西搬运"）**

增大 `rule2_silence`：

```yaml
asr:
  rule2_silence: 2.0   # 从 1.5 增加到 2.0
```

**Q: 识别结果重复字（如"小飞同学同学"）**

降低 `blank_penalty`：

```yaml
asr:
  blank_penalty: 0.2   # 从 0.25 降到 0.2
```

**Q: 需要离麦克风很近才能识别**

提高麦克风增益：

```yaml
audio:
  capture_volume: 100   # 确保已拉满
  mic_boost: true       # 确保启用 ALC
```

如果仍然不够，这是单麦克风系统的物理限制（有效距离 30-60cm）。

#### 6.7.8 性能指标

| 指标 | 数值 | 说明 |
|------|------|------|
| 唤醒响应延迟 | ~0.3 秒 | KWS 检测 + 提示音播放 |
| ASR 端点延迟 | 1.5 秒 | rule2_silence 参数 |
| LLM 首 token | ~5 秒 | 预填充阶段（硬件限制） |
| LLM 生成速度 | 8-12 tokens/s | RKNN NPU W4A16 量化 |
| TTS 合成速度 | RTF ~0.5 | 2 倍实时速度 |
| 首句播放延迟 | ~6 秒 | LLM 预填充 + TTS 合成 |

**完整对话总延迟**（从说完问题到开始播放回答）：

```
用户停止说话 → ASR 端点触发: 1.5s
ASR 识别完成 → LLM 首 token: 5s
LLM 首句完成 → TTS 合成开始播放: 1s
---------------------------------------------
总计: ~7.5 秒
```

#### 6.7.9 与 ROS2 集成

`voice_interaction.py` 是独立工具，无 ROS2 依赖。如需 ROS2 集成：

- `car1_interaction` 包中已包含 ROS2 版本
- 启动文件：`ros2 launch car1_interaction voice_interaction.launch.py`
- 发布话题：`/voice/transcription`（识别结果）、`/voice/response`（LLM 回答）
- 服务：`/voice/trigger`（手动触发识别）、`/voice/stop`（停止播放）

#### 6.7.10 下一步

完成基础语音交互后，可以：

1. **集成导航**：语音控制机器人移动（"导航到厨房"）
2. **技能扩展**：添加 Function Calling（查询时间、天气、电池状态）
3. **多轮对话**：支持上下文记忆（当前版本每次唤醒是新对话）
4. **自然语言控制**：结合 Level 4/5，语音控制抓取、巡逻等任务

---

---


## 常见问题

### 编译问题

**Q: colcon build 报错找不到包**
```bash
# 检查依赖是否安装
rosdep install --from-paths src --ignore-src -r -y

# 或手动安装缺失的包
sudo apt install ros-${ROS_DISTRO}-<package-name>
```

**Q: Python 模块导入错误**
```bash
# 检查 Python 路径
echo $PYTHONPATH

# 重新 source
source install/setup.bash
```

### 硬件连接问题

**Q: 串口权限不足**
```bash
# 临时授权（udev 别名 car1_stm32 通常已 0666）
sudo chmod 666 /dev/car1_stm32

# 永久授权
sudo usermod -aG dialout $USER
# 重新登录生效
```

**Q: 找不到串口设备**
```bash
# 查看所有串口
ls /dev/tty*

# 查看 USB 设备
lsusb

# 查看内核日志
dmesg | grep tty
```

### 运行时问题

**Q: 节点启动失败**
```bash
# 查看详细日志
ros2 launch <package> <launch_file> --log-level debug

# 检查节点是否运行
ros2 node list

# 检查话题
ros2 topic list
```

**Q: TF 变换错误**
```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo base_link laser_link
```

**Q: 雷达数据异常**
```bash
# 检查雷达话题
ros2 topic echo /scan

# 检查雷达参数
ros2 param list /m1ct_d2_node
```

**Q: 相机无图像**
```bash
# 检查相机设备
ls /dev/video*

# 测试相机
cheese  # 或其他相机测试工具

# 检查 ROS2 话题
ros2 topic hz /depth_cam/rgb/image_raw
```

### 导航问题

**Q: AMCL 定位不准**
- 重新设置初始位姿（2D Pose Estimate）
- 检查地图是否正确加载
- 调整 AMCL 参数（粒子数、更新频率）

**Q: 路径规划失败**
- 检查目标点是否在障碍物内
- 检查机器人半径参数是否正确
- 清除代价地图：`ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty`

**Q: 导航震荡**
- 降低最大速度
- 调整 TEB 参数（weight_kinematics_nh, min_obstacle_dist）
- 增大 inflation_radius

### 性能问题

**Q: CPU 占用过高**
- 降低传感器发布频率
- 减少 AMCL 粒子数
- 关闭不必要的可视化

**Q: 延迟过大**
- 检查网络连接（如使用远程机器）
- 降低图像分辨率
- 使用更快的算法

---

## 学习资源

### 官方文档

- [ROS2 官方文档](https://docs.ros.org/en/jazzy/)
- [Nav2 文档](https://navigation.ros.org/)
- [slam_toolbox 文档](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo 文档](https://gazebosim.org/docs)

### 推荐教程

- ROS2 基础教程
- Nav2 导航教程
- MoveIt2 运动规划教程
- OpenCV 视觉处理教程

### 社区支持

- ROS Discourse: https://discourse.ros.org/
- ROS Answers: https://answers.ros.org/
- GitHub Issues: 项目仓库 Issues 页面

---

## 总结

完成本教程后，你应该掌握：

- ✅ ROS2 基本操作和工具使用
- ✅ 机器人仿真和实物控制
- ✅ SLAM 建图和自主导航
- ✅ 视觉和雷达应用开发
- ✅ 常见问题排查和解决

**下一步建议**：
1. 深入学习 ROS2 核心概念（节点、话题、服务、参数）
2. 学习机器人运动学和控制理论
3. 尝试开发自己的应用功能
4. 参与开源社区，贡献代码

祝学习愉快！🚀

