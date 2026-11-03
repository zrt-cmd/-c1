# car1 机械臂 IK 测试指南

## 快速开始

### 1. 运行完整测试套件（推荐）

```bash
cd /home/iflytek/ros2_ws/src/xflz_robot_car1/arm/kinematics/kinematics
python3 test_fk_ik.py
```

**预期结果：**
- ✅ FK 测试通过
- ✅ IK 测试通过  
- ✅ 往返测试 4/4 通过（误差 < 1.5mm）
- ✅ PWM 转换测试通过

---

### 2. 自定义目标点测试

```bash
python3 custom_ik_test.py
```

**修改测试点：**
编辑 `custom_ik_test.py`，修改：
```python
test_cases = [
    # (x米, y米, z米, 俯仰角度)
    (0.25, 0.00, 0.15, 0),      # 你的目标点 1
    (0.22, 0.03, 0.12, -30),    # 你的目标点 2
]
```

---

### 3. 交互式测试（Python REPL）

```bash
python3
```

```python
from config import Point3D
from ik_analytical import solve_analytical_ik_auto_pitch, solution_to_dict
from fk import compute_fk_position_only
from transform import ik_angles_to_pwm
import math

# 定义目标点
target = Point3D(x=0.25, y=0.0, z=0.15)

# 求解 IK
solution = solve_analytical_ik_auto_pitch(target, preferred_pitch_rad=0.0)

if not solution.error:
    angles = solution_to_dict(solution)
    pwm = ik_angles_to_pwm(angles)
    
    print(f"关节角度 (度):")
    for name, angle in angles.items():
        print(f"  {name}: {math.degrees(angle):+.2f}°")
    
    print(f"\nPWM 值: {pwm}")
    
    # FK 验证
    pos = compute_fk_position_only(angles)
    error = math.sqrt((pos.x-target.x)**2 + (pos.y-target.y)**2 + (pos.z-target.z)**2)
    print(f"\nFK 验证误差: {error*1000:.2f} mm")
```

---

## ROS2 测试（需要先编译）

### 编译包

```bash
cd /home/iflytek/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car1_kinematics
source install/setup.bash
```

### 运行 IK 测试节点

**终端 1：启动节点**
```bash
ros2 run car1_kinematics ik_test_node
```

**终端 2：发送测试目标**
```bash
# 测试点 1：正前方 25cm，高度 15cm
ros2 topic pub /ik_test/target geometry_msgs/PoseStamped \
  "{pose: {position: {x: 0.25, y: 0.0, z: 0.15}}}" --once

# 测试点 2：右前方 22cm，右侧 3cm，高度 12cm
ros2 topic pub /ik_test/target geometry_msgs/PoseStamped \
  "{pose: {position: {x: 0.22, y: 0.03, z: 0.12}}}" --once

# 测试点 3：左前方 20cm，左侧 -2cm，高度 18cm
ros2 topic pub /ik_test/target geometry_msgs/PoseStamped \
  "{pose: {position: {x: 0.20, y: -0.02, z: 0.18}}}" --once
```

**查看输出：**
- 终端 1 会显示 IK 解、关节角度、PWM 值、FK 验证误差
- 关节角度发布到：`/ik_test/joint_angles`
- PWM 值发布到：`/ik_test/pwm_values`

---

## 可视化测试（RViz）

```bash
ros2 launch car1_description display.launch.py
```

在 RViz 中：
1. 手动拖动关节滑块
2. 观察 tcp_link（工具中心点）的位置
3. 验证机械臂运动范围

---

## 测试单个模块

### 配置参数
```bash
python3 config.py
```
输出：几何参数、工作空间、关节限位

### 正向运动学
```bash
python3 fk.py
```
输出：典型姿态的 TCP 位置

### 逆向运动学
```bash
python3 ik_analytical.py
```
输出：测试目标点的 IK 解

---

## 坐标系说明

**base_footprint 坐标系（原点在地面中心）：**
- +X 轴：向前（机器人正前方）
- +Y 轴：向左
- +Z 轴：向上

**工作空间：**
- 最大伸展：252.3 mm（从 J1 原点计算）
- 最小距离：0.0 mm
- J1 原点位置：base_footprint + (56.6, 0.0, 129.5) mm

**目标点说明：**
- 目标位置是 tcp_link（工具中心点）的位置
- tcp_link 位于 arm_link5 下方 76.2mm
- 指尖位于 tcp_link 下方 33mm
- 抓取时：tcp_link 应移动到物体顶部中心，指尖自然位于物体中部

---

## 常见问题

### Q1: "Out of reach" 错误
**原因：** 目标点超出工作空间（> 252.3mm）

**解决：**
- 减小 X、Y 坐标
- 确保 `sqrt(x² + y²) < 0.252` 米

### Q2: "Both elbow configurations exceed joint limits"
**原因：** 目标点在工作空间内，但所有关节配置都超出 ±180° 限位

**解决：**
- 调整 Z 高度（通常是 Z 太低或太高）
- 尝试不同的俯仰角

### Q3: "Target Y residual exceeds workspace constraint"
**原因：** Y 坐标偏移过大（> 10mm 容差）

**解决：**
- 减小 Y 坐标绝对值
- Y 坐标建议范围：-0.05 到 +0.05 米

### Q4: FK 验证误差较大
**原因：** 可能是 URDF 参数与实际硬件不匹配

**解决：**
- 检查 `config.py` 中的几何参数
- 用千分尺实测连杆长度
- 更新 `ROBOT_CALIBRATION` 参数

---

## 推荐测试顺序

1. ✅ **运行 `test_fk_ik.py`** - 验证基础功能
2. ✅ **运行 `custom_ik_test.py`** - 测试自定义目标点
3. ✅ **启动 RViz** - 可视化验证
4. ⏭️ **编译 + 运行 ROS2 节点** - 测试 ROS2 集成
5. ⏭️ **实机验证（Level 4）** - 连接硬件，真实测试

---

## 下一步：Level 4 实机集成

当前 Level 3 已完成数学基础和仿真验证。Level 4 需要：

1. **创建 `safe_arm_controller_node.py`**：
   - 订阅 `/arm_controller/target_pose`
   - 发布到 `/car1_base_driver/bus_servo/set_position`
   - 实现速度限制、轨迹插值、紧急停止

2. **实机验证工具**：
   - `known_point_test.py`：在桌面放置物理标记，验证到达精度
   - `ik_round_trip_test.py`：实机 IK→PWM→移动→读取→验证

3. **安全特性**：
   - 关节速度限制（避免突然移动）
   - 工作空间软限位（距离边界 2cm 时警告）
   - 碰撞检测（基础版：禁止负 Z 坐标）

查看 `LEVEL3_SUMMARY.md` 了解详细实现细节。
