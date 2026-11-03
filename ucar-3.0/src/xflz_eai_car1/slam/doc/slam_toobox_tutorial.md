# SLAM Toolbox 技术文档（car1 麦轮机器人）

> 适用平台：RK3576 + STM32F407 + 4 麦轮 + COIN-D6 雷达 + BMI088 IMU
> ROS 版本：ROS 2 Humble
> 配置来源：`slam/car1_slam/config/slam.yaml`、`driver/controller/config/ekf.yaml`

---

## 目录

1. [整体框架结构](#1-整体框架结构)
2. [系统输入参数](#2-系统输入参数)
3. [输入数据校准流程](#3-输入数据校准流程)
4. [校准数据验证方法](#4-校准数据验证方法)
5. [建图核心原理](#5-建图核心原理)
6. [建图实施步骤](#6-建图实施步骤)
7. [注意事项](#7-注意事项)
8. [检验标准](#8-检验标准)
9. [输出结果说明](#9-输出结果说明)
10. [结果验证方法](#10-结果验证方法)

---

## 1. 整体框架结构

### 1.1 模块组成

slam_toolbox 是基于 Karto 的图优化 2D SLAM 框架，核心由四层组成：

```
┌─────────────────────────────────────────────────────────┐
│                     slam_toolbox                          │
│                                                           │
│  ┌────────────┐   ┌────────────┐   ┌──────────────────┐  │
│  │ 前端        │   │ 位姿图       │   │ 后端优化           │  │
│  │ Scan Match │──▶│ Pose Graph │──▶│ Ceres Solver     │  │
│  │ (相关匹配)   │   │ (节点+约束)  │   │ (Levenberg-Marq) │  │
│  └────────────┘   └────────────┘   └──────────────────┘  │
│         ▲                │                                │
│         │                ▼                                │
│  ┌────────────┐   ┌────────────┐                          │
│  │ 占据栅格     │   │ 回环检测     │                          │
│  │ Occupancy  │◀──│ Loop Closure│                          │
│  │ Grid       │   │ (分支定界)   │                          │
│  └────────────┘   └────────────┘                          │
└─────────────────────────────────────────────────────────┘
```

### 1.2 模块职责

| 模块 | 职责 | 本项目配置 |
|------|------|-----------|
| 前端 Scan Matcher | 把新 scan 与局部地图做相关匹配，估计位姿增量 | `use_scan_matching: true` |
| 位姿图 Pose Graph | 用节点（关键帧位姿）+ 边（约束）表示轨迹 | `scan_buffer_size: 100` |
| 后端优化器 | 全局优化所有节点位姿，最小化约束残差 | `solver_plugin: CeresSolver` |
| 回环检测 | 识别机器人回到已访问区域，添加回环约束 | `do_loop_closing: true` |
| 占据栅格生成 | 把优化后的 scan 投影成 `/map` | `resolution: 0.05` |

### 1.3 与外部系统的交互

```
              /scan (LaserScan, COIN-D6)
                      │
TF: odom→base_footprint (EKF 提供)
                      │
                      ▼
              ┌──────────────┐
              │ slam_toolbox │
              └──────────────┘
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
  TF: map→odom     /map        /slam_toolbox/
   (校正漂移)    (OccupancyGrid)  graph_visualization
```

**关键**：slam_toolbox 通过 **TF 查询** `odom→base_footprint` 获取里程计，**不直接订阅 `/odom` 话题**。这条 TF 在本项目由 EKF（`robot_localization`）发布。

### 1.4 Launch 启动架构（2026-06 重构）

`slam.launch.py` 是单一入口，分层组织节点。重构后 robot_state_publisher / joint_state_publisher 下沉到 bringup，保证 EKF 能拿到 URDF 静态 TF。

```
slam.launch.py
│
├─ bringup.launch.py（IncludeLaunchDescription）
│   ├─ base_driver.launch.py     → /car1_base_driver（STM32 串口：IMU/电机/舵机）
│   ├─ robot_state_publisher     → URDF 静态 TF（base_footprint→base_link→imu_link/laser_link/arm）
│   ├─ joint_state_publisher     → 补轮子关节（手臂关节来自 servo_controller）
│   └─ chassis_controller.launch.py
│       ├─ imu_filter_madgwick   → /car1_base_driver/imu_raw → /imu
│       ├─ odom_publisher        → /odom_raw（开环积分 cmd_vel + 标定系数）
│       ├─ ekf_filter_node       → 融合 odom(vx,vy) + imu(vyaw) → TF odom→base_footprint
│       ├─ servo_controller      → /joint_states（手臂）
│       └─ init_pose             → 上电初始位姿
│
├─ lidar.launch.py               → /scan（frame laser_link）
│
├─ [TimerAction 10s] slam_toolbox → TF map→odom + /map
│
└─ [可选] rviz2
```

**职责边界**：
- **bringup**：所有"机器人自身"节点（驱动 + URDF TF + 里程计 + EKF）。单独跑也能完整定位。
- **slam.launch**：在 bringup 之上叠加雷达 + slam_toolbox + RViz。

---

## 2. 系统输入参数

### 2.1 输入数据清单

| 输入 | 类型 | 来源 | 必要性 |
|------|------|------|--------|
| `/scan` | `sensor_msgs/LaserScan` | COIN-D6 雷达驱动 | 必要 |
| TF `odom→base_footprint` | `tf2` | EKF（融合 wheel odom + IMU） | 必要 |
| TF `base_footprint→laser_link` | `tf2`（static） | URDF / robot_state_publisher | 必要 |

### 2.2 /scan 数据规范

| 字段 | 本项目值 | 说明 |
|------|---------|------|
| `frame_id` | `laser_link` | 必须与 URDF TF 链一致 |
| `range_max` | 16.0 m（驱动）/ 12.0 m（SLAM 截断） | `max_laser_range: 12.0` |
| QoS | **Best Effort** / SensorDataQoS | RViz 显示需设为 Best Effort |
| 频率 | ~10 Hz | COIN-D6 默认 |

### 2.3 里程计 TF 规范

| 要求 | 说明 |
|------|------|
| 连续性 | TF 不能跳变，否则 scan matcher 发散 |
| 频率 | EKF 输出 30 Hz（`frequency: 30.0`） |
| 时间戳 | 与 scan 时间对齐，容差 `transform_timeout: 0.2` |
| 2D 模式 | `two_d_mode: true`，z/roll/pitch 锁零 |

### 2.4 帧约定

```
map → odom → base_footprint → base_link → laser_link
 │      │          │
slam   EKF      EKF base_link_frame
toolbox
```

| 参数 | slam.yaml | ekf.yaml | 必须一致 |
|------|-----------|----------|---------|
| `map_frame` | map | map | ✓ |
| `odom_frame` | odom | odom | ✓ |
| `base_frame`/`base_link_frame` | base_footprint | base_footprint | ✓ |

---

## 3. 输入数据校准流程

### 3.1 里程计三类输入的校准

里程计 TF 由三类传感器融合而成，各自的校准项：

```
轮子/麦轮  → 几何标定（轮径、轴距、装配方向）
编码器     → 线性标定（走直线）+ 角度标定（转圈，麦轮意义有限）
IMU 陀螺仪 → ① 零偏校准（静置）② scale 校准（转动，关键）
IMU 加速度 → 六面静置法（2D SLAM 用不到，可跳过）
```

#### 3.1.1 麦轮几何标定

| 参数 | 名义值 | 校准方法 |
|------|--------|---------|
| 轮径 | 0.0935 m | 走直线，实际距离 / 指令距离修正 |
| 轴距 wheelbase | 0.1946 m | 量尺 + 转圈微调 |
| 轮距 track width | 0.216 m | 量尺 + 转圈微调 |

#### 3.1.2 编码器线性标定

```bash
ros2 run calibration calibrate_linear
```
- 命令走 1.0 m，量实际距离 `d_actual`
- 线性修正系数 = `1.0 / d_actual`

#### 3.1.3 IMU 陀螺仪校准（两步，**互相独立**）

**关键认知**：零偏（加性）和 scale（乘性）是两类不同误差，校准方法不同。

| 类型 | 误差性质 | 校准方法 | 静置能否校 |
|------|---------|---------|-----------|
| 零偏 bias | 静止时输出 ≠ 0 | 静置取均值 | ✅ 能 |
| 尺度 scale | 转动时倍数错 | **转动**对比已知角度 | ❌ 不能 |

**① 零偏校准（静置法）**
- 车静止采样数百帧，取 gyro 三轴均值作为零偏
- 本项目实测静止 gyro_z ≈ ±0.01 rad/s，零偏很小
- 可交给 `imu_filter_madgwick` / `complementary_filter` 的 `do_bias_estimation` 在线完成

**② scale 校准（转动法，本项目重点）**
- 地面贴 0°/90°/180°/270° 标记，转精确已知角度
- 对比 IMU 积分 `∫gyro_z·dt` 与真实角度
- 本项目历史问题：真实转 N 圈，IMU 积分曾报 2N 圈（2× 过报告）
- 根因：BMI088 量程寄存器 `0x0F` 写入未生效（soft-reset 后缺 30ms 延时），芯片停在错误量程
- ✅ **已修复**：固件加 30ms 复位延时 + 0x0F 回读校验，scale 恢复 0.030518 °/s/LSB（±1000°/s）
- 详见 [3.5 BMI088 IMU 详解](#35-bmi088-imu-详解)

#### 3.1.4 加速度计校准（可跳过）

六面静置法可校加速度计 scale/错位/零偏，但 **2D SLAM 不积分加速度算位移**（`imu0_config` 中 ax/ay/az 全 false），故对建图无收益，可跳过。

### 3.2 时间同步机制

| 机制 | 实现 | 参数 |
|------|------|------|
| 时间戳对齐 | 各传感器 msg 用硬件/接收时刻打 stamp | — |
| TF 时间容差 | scan 时刻查不到精确 TF 时的允许误差 | `transform_timeout: 0.2` |
| TF 缓存时长 | 历史 TF 保留时间 | `tf_buffer_duration: 30.0` |
| 最小时间间隔 | 两帧 scan 处理的最小间隔 | `minimum_time_interval: 0.1` |
| 使用仿真时间 | 实机为 false | `use_sim_time: false` |

**时间漂移处理**：所有节点共用 ROS time；EKF `transform_time_offset: 0.0` 表示不额外补偿延迟。若雷达 USB 传输有固定延迟，可在驱动端调整 stamp。

### 3.3 坐标系转换配置

```
TF 链与发布者：
map → odom            : slam_toolbox 发布（校正里程计漂移）
odom → base_footprint : EKF 发布（publish_tf: true）
base_footprint → ...  → laser_link : robot_state_publisher（URDF static）
```

外参（base_footprint→laser_link）来自 URDF，错误会导致 scan 与机器人朝向不符。

### 3.4 关键技术问题与对策

| 问题 | 现象 | 对策 | 状态 |
|------|------|------|------|
| IMU scale 误差（2×） | 航向累积漂移 | 固件 30ms 复位延时 + 0x0F 回读校验 | ✅ 已修 |
| IMU 协方差全 0 | EKF 静默丢弃 IMU | base_driver 填非零协方差 | ✅ 已修 |
| 缺 base_link→imu_link TF | EKF 无法变换 IMU | robot_state_publisher 移入 bringup | ✅ 已修 |
| imu0_relative=true | vyaw 被减初值 | ekf.yaml 改 false | ✅ 已修 |
| 轮速 vyaw 污染航向 | 麦轮打滑使航向失真 | odom0 关 vyaw，航向只用 IMU | ✅ 已修 |
| 安装外参误差 | scan 与墙体不贴 | 标定 base→laser 外参 | — |
| TF 时间漂移 | scan matcher 抖动 | 统一 ROS time，调 `transform_timeout` | — |

**EKF 输入分工（关键设计）**：
```yaml
# odom0（轮速）：只提供线速度 vx, vy（直线打滑小，编码器准）
odom0_config: [F,F,F, F,F,F, T,T,F, F,F,F, F,F,F]
# imu0（IMU）：只提供 vyaw（麦轮转圈打滑大，必须用 IMU 航向）
imu0_config:  [F,F,F, F,F,F, F,F,F, F,F,T, F,F,F]
```
麦轮原地转打滑 ~45%，轮速 vyaw 不可信，故航向交给 IMU；线速度直线打滑小，仍用轮速。

### 3.5 BMI088 IMU 详解

本节深入讲解 BMI088 IMU 的工作原理、配置细节及本项目遇到的实际问题与解决方案。

#### 3.5.1 BMI088 基本参数

| 项 | 值 | 说明 |
|----|-----|------|
| 型号 | Bosch BMI088 | 6 轴 IMU（3 轴加速度计 + 3 轴陀螺仪） |
| Chip ID | 0x0F | 读寄存器 0x00 确认 |
| 接口 | I2C (本项目) / SPI | 陀螺地址 0x68/0x69，加速度地址 0x18/0x19 |
| 陀螺量程（可配） | ±125 / ±250 / ±500 / ±1000 / ±2000 °/s | 本项目设置 **±1000°/s** |
| 加速度量程 | ±3g / ±6g / ±12g / ±24g | 本项目 ±3g |
| ADC 分辨率 | 16 位有符号整数（-32768 ~ +32767） | — |

#### 3.5.2 陀螺仪量程配置（寄存器 0x0F: GYRO_RANGE）

**核心概念**：量程 = 能测的最大角速度；分辨率 = 测量精度。

| 0x0F 值 | 满量程 [°/s] | 分辨率 [LSB/°/s] | 1 LSB = | 最大圈/秒 |
|---------|------------|-----------------|---------|----------|
| **0x00（默认）** | **±2000** | 16.384 | 61.0 m°/s | 5.6 圈/s |
| **0x01（本项目）** | **±1000** | 32.768 | 30.5 m°/s | 2.8 圈/s |
| 0x02 | ±500 | 65.536 | 15.3 m°/s | 1.4 圈/s |
| 0x03 | ±250 | 131.072 | 7.6 m°/s | 0.7 圈/s |
| 0x04 | ±125 | 262.144 | 3.8 m°/s | 0.35 圈/s |

**权衡**：
- 量程大 → 能测快速旋转，但精度差（噪声大）
- 量程小 → 精度高，但容易超量程溢出

**为何选 ±1000°/s**：
- 麦轮机器人正常转速 <1.5 圈/秒（540°/s）
- ±1000 有充足余量（2.8 圈/s）且精度适中（30.5 m°/s）
- SLAM 建图推荐 ω ≤ 1.0 rad/s (57°/s)，±1000 余量巨大

**Scale 计算**：
```c
// 对于 ±1000°/s (0x0F=0x01)
scale = 1000.0 / 32768.0 = 0.030518 °/s/LSB

// raw 转真实角速度
真实角速度 (°/s) = ADC_raw × scale
例如 raw=3277 → 3277 × 0.030518 = 100 °/s
```

#### 3.5.3 初始化流程（官方时序要求）

BMI088 数据手册（Rev 1.9）规定的正确初始化顺序：

```
1. 读 Chip ID (0x00, 应为 0x0F)
   ↓
2. Soft-reset (0x14 = 0xB6)
   ↓
3. **HAL_Delay(30 ms)**        ← 关键：让芯片完成复位，否则后续写入被忽略
   ↓
4. Power mode normal (0x11 = 0x00) + delay 5ms
   ↓
5. Range (0x0F = 0x01)         ← 设置量程 ±1000°/s
   + delay 5ms
   ↓
6. **回读 0x0F 校验**           ← 确认写入生效
   ├─ readback == 0x01 → 成功
   └─ readback != 0x01 → 重试或报错
   ↓
7. Bandwidth (0x10 = 0x02)     ← ODR 1000 Hz, BW 116 Hz
```

**本项目历史 Bug**：
- 旧固件缺 30ms 延时 → soft-reset 后立即写 0x0F → 芯片未完成复位 → 写入被忽略
- 芯片停在默认量程（±2000 或错误值），但代码按 ±1000 scale 换算
- 导致 IMU 数据 2× 过报告（真实转 1 圈，IMU 报 2 圈）

**修复方案（已实施）**：
```c
BMI088_I2C_Write_Reg(hi2c, bmi088_gyro_addr, 0x14, 0xB6);
HAL_Delay(30);  // ← 加这个！

BMI088_I2C_Write_Reg(hi2c, bmi088_gyro_addr, 0x0F, 0x01);
HAL_Delay(5);

// 回读校验
uint8_t readback;
BMI088_I2C_Read_Reg(hi2c, bmi088_gyro_addr, 0x0F, &readback);
if (readback != 0x01) {
    printf("[WARN] GYRO_RANGE write failed: got 0x%02X\r\n", readback);
    // 重试或按实际 readback 选 scale
}
```

#### 3.5.4 与 SLAM 系统的集成

**数据流**：
```
STM32 BMI088_Read_Gyro() → raw (int16_t) × gyro_scale
   ↓
串口 0xAA 0x55 07 ... (6 个 float: ax/ay/az/gx/gy/gz)
   ↓
ROS car1_base_driver → /car1_base_driver/imu_raw
   ↓
imu_filter_madgwick → /imu (零偏去除 + 姿态融合)
   ↓
EKF (robot_localization) → TF: odom→base_footprint
   ↓
slam_toolbox（通过 TF 查询里程计）
```

**关键配置**：
- 固件采样率：50 Hz（`osDelay(20)`）
- 实测采样率：10 Hz（因单字节 I2C 读取慢，待优化为 burst 读取）
- EKF 频率：30 Hz
- SLAM 订阅：不直接订阅 IMU，通过 TF 间接获取融合后的里程计

#### 3.5.5 数据采集与处理流程

**固件端（STM32）**：
```c
void bmi088_read_task(void *argument) {
    for(;;) {
        BMI088_Read_All(&hi2c1, &bmi088_data);  // 读 6 轴
        
        // 单位换算
        report.accel[0] = bmi088_data.accel.ax;  // m/s²
        report.gyro[2] = bmi088_data.gyro.gz * IMU_DEG2RAD;  // rad/s
        
        // 串口上报（功能字 0x07, 6 个 float）
        packet_send(&huart1, PACKET_FUNC_IMU, &report, sizeof(report));
        
        osDelay(20);  // 50 Hz
    }
}
```

**ROS 端（Python）**：
```python
def _pub_imu(self, ax, ay, az, gx, gy, gz):
    msg = Imu()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'imu_link'
    
    # 固件已换算成物理单位，直接用
    msg.linear_acceleration.x = ax  # m/s²
    msg.angular_velocity.z = gz     # rad/s（已修正 scale）
    
    self.imu_pub.publish(msg)
```

#### 3.5.6 校准步骤详解

**Step 1：零偏校准（静置法）**

目的：消除静止时陀螺仪输出 ≠ 0 的常值漂移。

```bash
# 车静止，记录 IMU 数据
ros2 topic echo /car1_base_driver/imu_raw

# 观察 angular_velocity.z，理想应 ≈ 0
# 本项目实测 ±0.01 rad/s，零偏很小，可交给 imu_filter 在线估计
```

**imu_filter_madgwick 自动处理**：
```yaml
parameters:
  do_bias_estimation: true  # 在线估计并扣除零偏
```

**Step 2：Scale 校准（转动法）**

目的：验证 ADC→物理单位的换算系数正确。

```bash
# 1. 地面贴 0°/90°/180°/270° 标记
# 2. 跑测试脚本
python3 src/xflz_eai_car1/test/imu_chassis_test.py

# 3. 手动转车体 1 整圈（360°），记录 IMU 积分
# 4. 对比
IMU 积分 (rad) ≈ 2π × 目视圈数 → 正常
IMU 积分 ≈ 2× 目视          → scale 错误（已修复）
```

**验证公式**：
```
理论角度 = 圈数 × 2π
IMU 积分 = Σ(gyro_z × Δt)
误差率 = |IMU 积分 − 理论角度| / 理论角度

合格标准：< 5%
```

#### 3.5.7 常见问题与解决方案

| 问题 | 现象 | 根因 | 解决 |
|------|------|------|------|
| **IMU 2× 过报告** | 真实转 1 圈，报 2 圈 | soft-reset 后缺 30ms 延时 | ✅ 固件加延时 + 回读校验 |
| IMU 完全不响应 | 转动时 gyro_z ≈ 0 | I2C 通信失败 / Chip ID 错误 | 检查接线、地址跳线 |
| 采样率仅 10 Hz | 设计 50 Hz，实测 10 Hz | 单字节 I2C 读取慢 | 改 burst 6 字节读取 |
| 静止漂移大 | 静止时 gyro_z 持续变化 | 温飘 / 零偏大 | 启用 `do_bias_estimation` |
| 地图旋转/双墙 | SLAM 航向累积误差 | IMU scale 错 + EKF 未用 vyaw | 修 scale + 启用 EKF vyaw |

#### 3.5.8 性能优化建议

**提升采样率（10 Hz → 50 Hz）**：
```c
// 当前：单字节读取，6 次 I2C 事务
BMI088_I2C_Read_Reg(hi2c, addr, GYRO_X_LSB, &data[0]);
BMI088_I2C_Read_Reg(hi2c, addr, GYRO_X_MSB, &data[1]);
// ... 共 6 次

// 优化：burst 读取，1 次 I2C 事务
BMI088_I2C_Read_Reg_Multi(hi2c, addr, GYRO_X_LSB, data, 6);
// 耗时从 60ms 降到 <10ms，采样率恢复 50 Hz
```

**降低功耗**（可选）：
- 降低 ODR（0x10 寄存器）：1000 Hz → 200 Hz
- 使用低功耗模式（牺牲采样率）

**提升精度**（权衡）：
- 量程改 ±500°/s（分辨率 65.536 LSB/°/s）
- 但快速避障可能超量程，需根据场景选择

#### 3.5.9 参考资料

- [BMI088 Datasheet Rev 1.9 (Bosch Sensortec, 2024-01)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [本项目固件代码](file:///home/iflytek/ros2_ws/xflz_eai_car1_base_controller_client/base/Core/Src/bmi088.c)
- [IMU 测试脚本](file:///home/iflytek/ros2_ws/src/xflz_eai_car1/test/imu_chassis_test.py)
- Bosch Application Note BST-BMI088-AN000 "BMI088 Integration Guide"

---

## 4. 校准数据验证方法

### 4.1 定量评估指标

| 指标 | 计算方式 | 合格阈值（参考） |
|------|---------|----------------|
| IMU scale 误差 | IMU 积分角度 / 真实角度 | 0.95 ~ 1.05 |
| 直线里程误差 | \|d_odom − d_actual\| / d_actual | < 2% |
| 闭环位置误差 | 回到起点时 odom 位置与起点距离 | < 0.1 m |
| 闭环航向误差 | 回到起点时 yaw 与起点之差 | < 5° |
| 墙体厚度 | 地图中同一面墙的双线间距 | < 0.1 m（接近真实墙厚） |

### 4.2 IMU scale 验证脚本

项目内 `test/imu_chassis_test.py` 实现"静置-旋转-静置"三阶段采集，自动输出：

```
IMU 积分圈数 = Σ(gyro_z × dt) / (2π)
缩放倍数     = IMU 积分圈数 / 目视真实圈数
```

判读：

| 倍数 | 结论 |
|------|------|
| ~1.0 | IMU 正常 |
| ~2.0 | 量程寄存器问题，过报告 2× |
| <1 | 打滑或 under-report |

### 4.3 可视化验证手段

| 工具 | 命令 | 看什么 |
|------|------|--------|
| RViz LaserScan | `ros2 launch car1_slam rviz_slam.launch.py` | 实时点云是否贴合已建地图 |
| TF 树 | `ros2 run tf2_tools view_frames` | TF 链完整、无断裂 |
| TF 实测 | `ros2 run tf2_ros tf2_echo odom base_footprint` | 转动时 yaw 是否合理 |
| 话题频率 | `ros2 topic hz /scan` `/odometry/filtered` | 频率是否达标 |

### 4.4 异常数据检测

| 异常 | 检测方法 | 处理 |
|------|---------|------|
| scan 全 0 / inf | `ros2 topic echo /scan` | 检查雷达供电/串口 |
| TF 跳变 | tf2_echo 观察突变 | 检查 EKF 输入是否冲突 |
| odom 漂移过快 | 静止时 odom 是否动 | 检查编码器/IMU 零偏 |
| 时间戳乱序 | `ros2 topic echo --field header.stamp` | 检查 use_sim_time |

---

## 5. 建图核心原理

### 5.1 前端里程计估计（Scan Matching）

slam_toolbox 用**相关性扫描匹配**（Correlative Scan Matching）：

1. 以里程计预测位姿为中心，在搜索窗口内枚举候选位姿
2. 每个候选位姿把当前 scan 投影到局部地图，计算匹配响应得分
3. 取响应最高的位姿作为前端估计

搜索窗口参数（本项目）：

```yaml
correlation_search_space_dimension: 0.5      # 搜索窗 ±0.5 m
correlation_search_space_resolution: 0.01    # 步进 1 cm
fine_search_angle_offset: 0.00349            # 精搜角度 0.2°
coarse_search_angle_offset: 0.349            # 粗搜角度 20°
```

匹配响应得分（概念式）：

```
response = Σ_i  P_map(T·p_i)
其中 T 为候选位姿，p_i 为 scan 第 i 个点，P_map 为占据概率查找表
```

### 5.2 后端优化（图优化）

把轨迹建模为位姿图，节点为关键帧位姿 `x_i`，边为约束 `z_ij`：

```
最小化目标：
  X* = argmin Σ_ij  e_ij(x_i, x_j)ᵀ · Ω_ij · e_ij(x_i, x_j)

  e_ij = z_ij ⊖ (x_j ⊖ x_i)   （观测约束与相对位姿之差）
  Ω_ij                          （信息矩阵 = 协方差的逆）
```

本项目用 Ceres + Levenberg-Marquardt：

```yaml
solver_plugin: solver_plugins::CeresSolver
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
ceres_trust_strategy: LEVENBERG_MARQUARDT
```

约束惩罚权重：

```yaml
distance_variance_penalty: 0.5   # 平移残差权重
angle_variance_penalty: 1.0      # 旋转残差权重
```

### 5.3 回环检测

采用**分支定界**（Branch and Bound）在历史关键帧中搜索回环：

1. 当前帧与 `loop_search_maximum_distance: 3.0` m 内的历史帧做粗匹配
2. 粗匹配响应 > `loop_match_minimum_response_coarse: 0.35` → 进入精匹配
3. 精匹配响应 > `loop_match_minimum_response_fine: 0.45` → 确认回环，添加约束
4. 触发全局优化，消除累积漂移

```yaml
do_loop_closing: true
loop_search_space_dimension: 8.0
loop_match_minimum_chain_size: 10
loop_match_maximum_variance_coarse: 3.0
```

### 5.4 地图表示与更新

- **表示**：占据栅格（OccupancyGrid），分辨率 `resolution: 0.05`（5 cm/格）
- **取值**：-1 未知，0 空闲，100 占据
- **更新**：`map_update_interval: 0.1` s 刷新一次；每个关键帧 scan 投影累积
- **关键帧触发**：位移 > `minimum_travel_distance: 0.01` m 或航向 > `minimum_travel_heading: 0.02` rad

---

## 6. 建图实施步骤

### 6.1 环境配置要求

```bash
source /opt/ros/humble/setup.bash
cd /home/iflytek/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

依赖：`slam_toolbox`、`robot_localization`、`robot_state_publisher`、COIN-D6 驱动。

### 6.2 启动

一键启动（含 bringup + 雷达 + slam_toolbox）：

```bash
ros2 launch car1_slam slam.launch.py
```

启动顺序（来自 `slam.launch.py`，2026-06 重构后）：
1. bringup（驱动 + **robot_state_publisher** + **joint_state_publisher** + 底盘控制器 + EKF）
2. 雷达（发布 `/scan`，frame `laser_link`）
3. slam_toolbox（**延迟 10 s** 等 odom/TF/scan 稳定）
4. RViz2（可选，`use_rviz:=false` 关闭）

> ⚠️ **重要变更**：robot_state_publisher / joint_state_publisher 已从 slam.launch
> 移入 bringup。原因：EKF 需要 `base_link → imu_link` 的静态 TF 才能把 IMU 角速度
> 变换到 base 坐标系；若 RSP 只在 slam.launch，单独跑 bringup 时 EKF 拿不到该 TF，
> IMU 数据被静默丢弃（航向恒不更新）。移入 bringup 后，RSP 在 EKF 之前启动，
> 且 robot_localization 对 TF 查询有重试机制，可容忍小的启动时序差。
> slam.launch 不再重复启动，避免两份 RSP 抢 `/robot_description` 与 `/tf_static`。

启动顺序设计要点（满足 slam_toolbox 要求）：
- slam_toolbox 用 `TimerAction period=10.0` 延迟启动，等 TF 树（map 除外）、odom、scan 全部就绪
- slam_toolbox 自身发布 `map → odom`，依赖下游已有 `odom → base_footprint`（EKF）和 `base_footprint → laser_link`（RSP）

### 6.3 关键启动参数

| 参数 | 值 | 作用 |
|------|----|----|
| `mode` | mapping | 建图模式（定位模式用 localization） |
| `map_update_interval` | 0.1 | 地图刷新周期 |
| `minimum_travel_distance` | 0.01 | 关键帧位移阈值（小→更新频繁，地图细） |
| `minimum_travel_heading` | 0.02 | 关键帧航向阈值 |
| `transform_publish_period` | 0.02 | map→odom 发布周期（50 Hz） |

### 6.4 数据采集规范

- 角速度 ≤ 1.0 rad/s，避免点云运动畸变
- 线速度 ≤ 0.3 m/s
- 拐角处放慢，确保回环帧充分重叠
- 尽量走闭环路径（回到起点触发回环优化）
- 避免长直走廊无特征（scan matcher 易退化）

### 6.5 建图过程监控

```bash
# 实时可视化
ros2 launch car1_slam rviz_slam.launch.py

# 监控频率
ros2 topic hz /scan
ros2 topic hz /map

# 监控 TF
ros2 run tf2_ros tf2_echo odom base_footprint
```

观察要点：墙体是否单线（无重影）、点云是否贴合、回环后地图是否突然校正。

---

## 7. 注意事项

### 7.1 硬件选型建议

| 项 | 建议 |
|----|------|
| 雷达 | 频率 ≥ 10 Hz，量程覆盖场景，本项目 COIN-D6 0.01–16 m |
| IMU | 陀螺零偏小、scale 准；麦轮平台 IMU 几乎必备 |
| 编码器 | 闭环反馈，分辨率足够 |
| 算力 | RK3576 可实时跑 2D SLAM |

### 7.2 环境适应性

| 场景 | 风险 | 对策 |
|------|------|------|
| 长走廊 | 特征退化，前端漂移 | 降速、依赖里程计、增加路标 |
| 玻璃/镜面 | 雷达穿透/反射 | 贴标记、避开 |
| 动态行人 | 残影入图 | 降速、事后清理 |
| 空旷大厅 | 超量程无回波 | 沿墙走 |

### 7.3 常见问题与规避

| 问题 | 原因 | 规避 |
|------|------|------|
| 地图旋转/双墙 | 里程计航向不准（IMU 2×、打滑） | 先修 IMU scale，启用 EKF vyaw |
| scan matcher 发散 | TF 跳变/时间不同步 | 检查 TF 连续性 |
| 回环不触发 | 阈值过严/路径不闭合 | 调低 response 阈值、走闭环 |
| RViz 无 scan | QoS 不匹配 | Reliability 设 Best Effort |
| 地图不更新 | travel 阈值过大 | 调小 minimum_travel_* |

### 7.4 性能优化

- `scan_buffer_size` 适中（本项目 100），过大耗内存
- `throttle_scans: 1` 全部处理；算力紧张可增大跳帧
- `max_laser_range` 截断到有效范围（12 m）减少计算
- `stack_size_to_use: 40000000` 防大地图栈溢出

---

## 8. 检验标准

### 8.1 地图精度量化

| 指标 | 方法 | 合格 |
|------|------|------|
| 墙体厚度 | 量地图双线间距 | < 0.1 m |
| 尺寸误差 | 地图测距 vs 卷尺实测 | < 2% |
| 直角度 | 量墙体夹角 | 90°±2° |
| 重影率 | 平行重墙占比 | < 5% |

### 8.2 轨迹一致性

| 方法 | 说明 |
|------|------|
| 闭环误差 | 回起点位置 < 0.1 m，航向 < 5° |
| 多圈重复 | 同路径多圈轨迹重合 |
| odom vs map | map→odom 校正量不应持续增大 |

### 8.3 计算资源基准

```bash
top -p $(pgrep -f slam_toolbox)
```

| 资源 | 参考（RK3576） |
|------|---------------|
| CPU | 单核 < 60% |
| 内存 | 中等场景 < 500 MB |
| map 发布 | 稳定 10 Hz |

### 8.4 系统稳定性

- 长时间（>10 min）运行无崩溃、无内存持续增长
- 回环后地图收敛，不反复跳变
- TF 无 `extrapolation` 警告刷屏

---

## 9. 输出结果说明

### 9.1 地图文件格式

保存地图：

```bash
ros2 run car1_slam map_save map_01
# 或
ros2 run nav2_map_server map_saver_cli -f map_01 \
  --ros-args -p map_subscribe_transient_local:=true
```

产物：

| 文件 | 内容 |
|------|------|
| `map_01.pgm` | 灰度位图：255 空闲，0 占据，205 未知 |
| `map_01.yaml` | 元数据：分辨率、原点、阈值 |

`*.yaml` 示例字段：

```yaml
image: map_01.pgm
resolution: 0.05
origin: [-x, -y, 0.0]      # 地图左下角在 map 坐标
occupied_thresh: 0.65
free_thresh: 0.25
negate: 0
```

### 9.2 轨迹/位姿图数据

- slam_toolbox 可序列化位姿图（`.posegraph` + `.data`），供后续续图/定位
- 序列化服务：`/slam_toolbox/serialize_map`

### 9.3 日志解读

| 日志 | 含义 |
|------|------|
| `Registering sensor` | 雷达注册成功 |
| `Loop closure ...` | 回环触发 |
| `Message Filter dropping message` | TF 滞后/时间不同步 |
| `extrapolation into the future/past` | 时间戳问题，检查 use_sim_time |

---

## 10. 结果验证方法

### 10.1 地图可视化验证

```bash
ros2 launch car1_slam rviz_slam.launch.py
```

- 加 Map 显示，看墙体闭合、无重影
- 加 LaserScan（Best Effort），实时点云应贴合地图墙体
- 加 TF，确认 map→odom→base_footprint 链完整

### 10.2 轨迹比对

- 走已知尺寸闭环（如 2 m×2 m 方形）
- 地图测量边长，对比实际
- 回起点位姿误差评估（位置 + 航向）

### 10.3 实际导航测试

用建好的地图跑导航，间接验证地图质量：

```bash
ros2 launch car1_navigation navigation.launch.py map:=map_01
```

- AMCL 能否在地图内正确定位（机器人模型与地图对齐）
- RViz 设目标点，能否规划并到达
- 麦轮全向运动是否正常

---

## 附录：本项目当前待办

| 优先级 | 事项 | 状态 |
|--------|------|------|
| P0 | IMU gyro scale 2× 修正（固件回读 0x0F + 30ms 延时） | ✅ **已完成** (2024) |
| P1 | EKF 启用 IMU vyaw（imu0_config 第 12 位） | 待启用 |
| P1 | IMU 采样率 10 Hz → 50 Hz（I2C burst 读取） | 待优化 |
| P1 | 编码器线性标定（走直线） | 待做 |
| P2 | 编码器角度标定（麦轮意义有限，靠 IMU） | 可选 |
| 跳过 | 加速度计六面法（2D SLAM 不用） | 不做 |
