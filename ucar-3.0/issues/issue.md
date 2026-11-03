# 机械臂视觉控制代码 Issue 清单

> 来源：`xflz_robot_eai_ros2_ws/src/xflz_eai_car1/` 视觉控制机械臂相关代码审查
> 涉及文件：`app/car1_app/`（visual_grasp.py、track_and_grab.py、object_tracking.py、common.py）、`arm/car1_arm_controller/`、`arm/kinematics/`、`driver/car1_servo_controller/`
> 严重度：P0=严重(导致错误/撞击/功能不可用) / P1=中等(影响鲁棒性) / P2=轻微(风格/文档)

---

## ISSUE-01  [P0] visual_grasp 硬编码 TF 变换，完全绕过 TF 系统

**文件**：`app/car1_app/car1_app/visual_grasp.py:359-380` (`_detect_and_localize`)

**现象**
```python
# Transform to base_link manually (don't use TF which may use stale joint_states)
cam_x, cam_y, cam_z = 0.256, -0.003, 0.244
pitch_rad = 0.0927  # 5.31° in radians
cos_p, sin_p = 0.9957, 0.0925
...
x_base = cam_x - y_cam
y_base = cam_y + cos_p * x_cam - sin_p * z_cam
z_base = cam_z - sin_p * x_cam - cos_p * z_cam
```

**问题**
- 相机在 base_link 下的位姿 `[0.256, -0.003, 0.244]` 和俯仰角 `5.31°` 是针对特定观察位姿 `[0.2, 0, 0.22, π]` 离线算出的常量，写死在代码中。
- 节点第 84-85 行已创建 `tf_buffer` / `tf_listener`，却完全未使用。注释称"TF 可能用过期 joint_states"——根因是 `servo_controller` 的 `publish_rate: 3.0` 过低（6 个舵机轮询，全量刷新约 2s），但正确做法是提升回读速率或等待 TF 新鲜，而非写死变换。

**影响**
只要 `observe_pose` 参数改变、机械臂装配/标定有偏差、或相机装夹位置变化，定位结果立即全错且无任何报错。`dry_run` 测出的坐标对得上纯属"恰好"对上固定姿态。

**建议修复**
改用 `tf_buffer.lookup_transform('base_link', camera_depth_optical_frame, time)` + `do_transform_point`（已 import `tf2_geometry_msgs.do_transform_point` 却未用）。若担心 TF 时延，检测前显式 `tf_buffer.can_transform` 校验并等待；同时将 `servo_controller` 的 `publish_rate` 从 3.0 提至 ≥20。

---

## ISSUE-02  [P0] target_pose 发布触发机械臂提前移动到物体表面（可能撞击）

**文件**：`app/car1_app/car1_app/visual_grasp.py:382-414` (`_handle_compute_target`) + `arm/car1_arm_controller/car1_arm_controller/arm_controller.py:142-171` (`pose_callback`)

**现象**
流程时序冲突：
1. `COMPUTE_TARGET` 状态把检测到的物体坐标 `(x,y,z)` 配 `pitch=π` 发布到 `/arm_controller/target_pose`。
2. `arm_controller.pose_callback` 收到后**立即**对该点做 IK 并 `_move_stepwise` 移动过去——直接移到物体表面点，**夹爪此时未张开**。
3. 紧接着 `visual_grasp` 进入 `PICK` 状态调用 `/arm_controller/pick`，`pick_callback` 再次执行 `_execute_pick`（开夹爪→移到上方→下降→闭夹爪→抬起）。

**问题**
- 第 2 步把未张开的夹爪直接送到物体表面，可能撞击/推走物体。
- `target_pose` topic 语义本应是"设定抓取目标点"，但 `pose_callback` 副作用是"立刻移动过去"。`pick` 服务又假设臂还在预备位置。两者语义冲突，导致重复且互相冲突的运动。
- `_execute_pick` 内部第 2 步本就是"移到 z+pre_h 上方"，故 `COMPUTE_TARGET` 阶段不应触发实际移动。

**建议修复**
`visual_grasp` 不在 `COMPUTE_TARGET` 发布 `target_pose`；改为只设置内部 `pick_target` 后直接调用 `pick` 服务。或在 `arm_controller` 区分"设目标"与"移动到目标"两种语义（如新增 `set_target` 服务，仅存目标不移动）。

---

## ISSUE-03  [P0] track_and_grab 舵机控制 topic 名错误，指令发不到舵机

**文件**：`app/car1_app/car1_app/track_and_grab.py:100-104`

**现象**
```python
self.servo_pub = self.create_publisher(
    ServosPosition, 'car1_servo_controller/servo_controller', 10
)
```

**问题**
`controller_manager_node.py:87-89` 订阅的是相对名 `servo_controller`（节点名 `servo_controller`，namespace `/`，实际解析为 `/servo_controller`）。`arm_controller.py:92` 也发到 `/servo_controller`。track_and_grab 多了一层 `car1_servo_controller/` 前缀，**无人订阅，指令全部丢弃**。

**建议修复**
将 topic 改为 `/servo_controller`，与 arm_controller 保持一致。

---

## ISSUE-04  [P0] track_and_grab ServosPosition 消息字段赋值错误 + 缺单位

**文件**：`app/car1_app/car1_app/track_and_grab.py:202-215` (`_publish_servo_command`)

**现象**
```python
msg = ServosPosition()
msg.servo_id = list(positions.keys())    # ServosPosition 无 servo_id 字段
msg.position = list(positions.values())  # position 期望 ServoPosition[]，不是数字列表
self.servo_pub.publish(msg)
```

**问题**
`ServosPosition.msg` 实际定义：
```
float64 duration
string position_unit
car1_servo_controller_msgs/ServoPosition[] position
```
- 没有 `servo_id` 字段 → 赋值抛 `AttributeError`。
- `position` 应是 `ServoPosition[]`（每个含 `id` + `position`），这里塞了裸数字列表。
- 未设置 `position_unit`（`'pulse'`/`'rad'`/`'deg'`），而 `controller_manager_node` 回调只在这三种 unit 下处理，否则忽略。

三者叠加：即便修好 topic 名，指令也发不出或被丢弃。

**建议修复**
```python
msg = ServosPosition()
msg.duration = 1.0
msg.position_unit = 'pulse'  # 或 'rad'
for sid, pos in positions.items():
    sp = ServoPosition()
    sp.id = sid
    sp.position = float(pos)
    msg.position.append(sp)
```

---

## ISSUE-05  [P0] track_and_grab 舵机 ID 映射错误，夹爪指令发到肩关节

**文件**：`app/car1_app/car1_app/track_and_grab.py:368-379`

**现象**
```python
self._publish_servo_command({1: 500})            # 注释 "Gripper open position"
self._publish_servo_command({2: 300, 3: 600})    # 注释 "Shoulder, Elbow"
```

**问题**
权威配置 `driver/car1_servo_controller/config/servo_controller.yaml`：

| 关节 | servo id |
|------|----------|
| arm_joint1（底座旋转） | 0 |
| arm_joint2（肩） | 1 |
| arm_joint3（肘） | 2 |
| arm_joint4（腕） | 3 |
| arm_joint5 | 4 |
| r_joint（夹爪） | 5 |

track_and_grab 把 `id=1` 当夹爪，实际是**肩关节**；夹爪应是 `id=5`。

**建议修复**
夹爪用 `id=5`，肩 `id=1`，肘 `id=2`，腕 `id=3`。并参考 `pick_and_place.yaml` 的脉冲值（夹爪开 1000、闭 1500）。

---

## ISSUE-06  [P1] visual_grasp 开环等待 3s，无到位反馈

**文件**：`app/car1_app/car1_app/visual_grasp.py:224-230` (`_handle_wait_stable`)

**问题**
固定 `elapsed > 3.0` 即进入检测，既不订阅 `/joint_states` 也不比较当前位姿与目标位姿。机械臂若因限位、堵转未到位，检测会在错误姿态下进行，配合 ISSUE-01 的硬编码 TF，定位直接错位。

**建议修复**
订阅 `/joint_states`，在 `WAIT_STABLE` 状态比较当前关节角与目标位姿的 IK 解，偏差小于容差（如 0.02 rad）且持续若干帧后再进入检测。

---

## ISSUE-07  [P1] visual_grasp depth_sample_size 参数声明却未使用

**文件**：`app/car1_app/car1_app/visual_grasp.py:55`（声明）、`:328`（实际用硬编码）

**问题**
第 55 行声明 `depth_sample_size`（默认 5），launch 里也传 5，但第 328 行实际用 `sample_size = 10` 硬编码。参数形同虚设。

**建议修复**
将第 328 行改为 `sample_size = self.depth_sample_size`。

---

## ISSUE-08  [P1] 深度单位启发判断边界脆弱

**文件**：`app/car1_app/car1_app/visual_grasp.py:346-347`、`track_and_grab.py:196`

**现象**
```python
if z_cam > 100:
    z_cam = z_cam / 1000.0
```

**问题**
用 `>100` 区分毫米(uint16)与米(float)。当真实距离恰为 0.1m 时：毫米模式下是 100，不满足 `>100`，会被当作米 → 深度放大 1000 倍。虽 10cm 通常小于相机最小工作距离，但属脆弱设计。

**建议修复**
直接根据图像 `encoding`（如 `16UC1` 即毫米，`32FC1` 即米）确定单位，而非猜测。

---

## ISSUE-09  [P1] track_and_grab 抓取动作无时序，连续发送位置目标

**文件**：`app/car1_app/car1_app/track_and_grab.py:360-385` (`_handle_grab`)

**问题**
在一次 tick 调用里连续发"开夹爪→降臂→闭夹爪"三条指令，中间无 `duration` 等待，舵机来不及执行，只有最后一条生效或运动混乱。对比 `arm_controller._execute_pick` 是有 `_wait()` 分步执行的。

**建议修复**
抓取动作分步执行，每步设置合理 `duration` 并等待完成；或直接弃用 track_and_grab 的抓取段，复用 `arm_controller` 的 `/arm_controller/pick` 服务。

---

## ISSUE-10  [P1] track_and_grab / object_tracking 相机 topic 与 visual_grasp 不一致

**文件**：`app/car1_app/car1_app/track_and_grab.py`、`object_tracking.py`

**问题**
`track_and_grab.py`/`object_tracking.py` 订阅 `/depth_cam/rgb/image_raw`、`/depth_cam/depth/image_raw`；`visual_grasp.py` 订阅 `/ascamera_hp60c/camera_publisher/rgb0/image`。visual_grasp 注释明确驱动靠 topic 名里 `rgb0`/`depth0` 子串门控流，不能 remap。两套 topic 名并存，需确认哪个是当前驱动真实发布的，否则 track_and_grab/object_tracking 收不到图。

**建议修复**
统一相机 topic 名为驱动实际发布的名称（`/ascamera_hp60c/camera_publisher/rgb0/image` 等），或在 launch 中正确 remap。

---

## ISSUE-11  [P1] visual_grasp timer 回调中 raise SystemExit 退出节点

**文件**：`app/car1_app/car1_app/visual_grasp.py:448,454`

**现象**
```python
def _handle_done(self):
    raise SystemExit(0)
```

**问题**
在 10Hz timer 回调里抛 `SystemExit` 终止节点，依赖 `main()` 里 `except SystemExit` 兜底。能跑但非常规，若 executor 配置不同可能抛到错误位置。

**建议修复**
改用 `self.timer.cancel()` + `rclpy.shutdown()`，或 `executor.shutdown()`。

---

## ISSUE-12  [P2] ik_analytical.py pitch 约定注释与代码矛盾

**文件**：`arm/kinematics/kinematics/ik_analytical.py:53`

**问题**
docstring 写 `0 = horizontal forward`，但代码几何 `toolZ = L_TCP * cos(pitch)` 表明 pitch=0 时 TCP 朝上（+Z），pitch=π 时朝下。与 `arm_controller._quat_to_pitch`（0=上, π/2=水平, π=下）实际一致，但注释错误。

**建议修复**
将第 53 行注释改为 `target_pitch_rad: Target pitch angle (radians), 0 = TCP pointing up (+Z), π/2 = horizontal, π = pointing down`。

---

## ISSUE-13  [P2] arm_controller 的 d1/a2/a3/a4 参数是死参数

**文件**：`arm/car1_arm_controller/car1_arm_controller/arm_controller.py:50-54,65-68`

**问题**
声明并读取 `d1/a2/a3/a4`，但实际 IK 调用 `solve_analytical_ik_auto_pitch`，用的是 `config.py` 的 `ROBOT_CALIBRATION`/`GEOM`（J1_origin、link1、link2、toolFromJ4），完全未用到 d1/a2/a3/a4。读出后从不参与计算，易让人误以为可调。

**建议修复**
删除 `d1/a2/a3/a4` 参数读取，或在 launch/yaml 中移除对应配置项，避免误导。

---

## ISSUE-14  [P2] arm_controller gripper_depth 缺夹爪几何补偿

**文件**：`arm/car1_arm_controller/car1_arm_controller/arm_controller.py`

**问题**
`gripper_depth` 默认 0，注释"tcp 停在检测到的表面点，夹爪跨过物体闭合，不下探"。visual_grasp 检测的是物体**顶面**中心深度，z 即顶面高度。gripper_depth=0 时下降到 z，夹爪指尖高度需与物体中心对齐才能夹住——当前无夹爪指尖到 TCP 的长度补偿，薄物体可能夹空。

**建议修复**
按夹爪指尖到 TCP 的偏移补偿 `gripper_depth`，或让下降目标点为物体中心高度（顶面 z - 物体半高 + 夹爪指尖偏移）。

---

## ISSUE-15  [P2] CLAUDE.md 舵机 ID 记载错误（已修复）

**文件**：`xflz_robot_eai_ros2_ws/CLAUDE.md:151`、`xflz_robot_eai_ros2_ws/src/xflz_eai_car1/CLAUDE.md:124`

**问题**
原记载 `5 (bus servo ID 1-5) + gripper (ID 10)`，与权威配置 `servo_controller.yaml`（臂 ID 0-4、夹爪 ID 5）不符，会误导维护者填错舵机 ID（如 ISSUE-05）。

**修复状态**
已于 2026-06-20 修正为 `5 (bus servo ID 0-4, arm_joint1-5) + gripper (ID 5, r_joint)`。

---

## 修复优先级建议

| 优先级 | Issue | 说明 |
|--------|-------|------|
| 立即修 | 01, 02 | visual_grasp 定位正确性与抓取安全 |
| 立即修 | 03, 04, 05 | track_and_grab 抓取链路完全不可用 |
| 尽快修 | 06, 08 | 鲁棒性隐患 |
| 尽快修 | 09, 10 | track_and_grab 功能完整性 |
| 排期修 | 07, 11, 13, 14 | 工程债 |
| 顺手修 | 12, 15 | 文档/注释（15 已修） |
