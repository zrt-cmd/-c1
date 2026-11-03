# 实验 11 — 体感控制（姿态动作 → 控制指令）

> 在 exp10 姿态骨架基础上加**动作分类 → 控制指令**（vendor 11_整体检测_体感 为纯可视化，
> 无控制逻辑；PLAN exp11 要求"动作分类→控制指令"，规则为本实验设计）。
> 双栏（左=原图+FPS，右=黑底骨架）+ 状态栏显示动作。

## 动作规则（actions.py，物理直觉优先）

| 动作 | 判定（归一化坐标） | 指令 | cmd_vel |
|---|---|---|---|
| 下蹲 squat | 双髋 y > 0.72（最高优先） | 后退 | linear −0.2 m/s |
| 双手举 both_up | 双腕 y < 双肩 y − 0.08 | 前进 | linear +0.2 m/s |
| 左手举 left_up | 仅左腕高 | 左转 | angular +0.8 rad/s |
| 右手举 right_up | 仅右腕高 | 右转 | angular −0.8 rad/s |
| 站立/消失 | 其余 | 停止 | 0 |

- 关键点 vis < 0.5 忽略；髋部不可见 → 停止（姿态不可信）
- 优先级：squat > both_up > left/right（下蹲时举手不算）
- 车 `/cmd_vel` 会 clamp ±0.3 m/s / ±1.0 rad/s（odom_publisher_node）

## 技术方案

| 环节 | 选型 |
|---|---|
| 模型 | `pose_landmark_full.tflite`（同 exp10；整图 256×256，33 点） |
| 转换 | `convert_mediapipe.py`（mean 127.5/std 128 + uint8 输入，对齐 mediapipe-rknn；fp16 默认） |
| 板上推理 | `RKNNLiteBackend`（core_mask 0=AUTO） |
| 取流 | `--ros` = ascamera 话题（标准）；`--camera N` = V4L2 回退 |
| 控制发布 | 节点模式可选（`control.publish_cmd_vel: false` 默认关，避免误动）；话题可配（车 `/cmd_vel` 或 turtlesim `/turtle1/cmd_vel`） |
| 动作打印 | `control.print_interval`（秒，默认 2.0；0=关）：终端周期打印当前动作 |
| 显示 | 双栏 + 状态栏 `FPS: N Action: Turn Left/...`（**英文**：cv2.putText 无中文字形，中文会显示 ????） |
| 流 | MJPEG `:8083/pose_control.mjpg` |

## 运行

```bash
# 转换（x86 转换环境，tensorflow≤2.14 用户安装）：
PYTHONPATH=car1_pose_control python3 -m pose_control_core.convert_mediapipe --all
PYTHONPATH=car1_pose_control python3 -m pose_control_core.convert_mediapipe --check
# 1) 单图：状态栏显示动作
PYTHONPATH=car1_pose_control python3 -m pose_control_core.demo_image \
  --config config/pose_control_config.yaml --display --save out.jpg
# 2) 相机实时（标准）：
ros2 launch ascamera hp60c.launch.py
PYTHONPATH=car1_pose_control:$PYTHONPATH python3 -m pose_control_core.camera_demo \
  --config config/pose_control_config.yaml --ros --display   # 或 :8083/pose_control.mjpg
# 3) ROS2 节点 + 速度指令（可选；改 config control.publish_cmd_vel: true 后）
cd ~/ros2_ws && colcon build --symlink-install --packages-select car1_pose_control
ros2 launch car1_pose_control pose_control.launch.py
# 3b) 控制 turtlesim 小海龟（把 config control.topic 改为 /turtle1/cmd_vel）
ros2 run turtlesim turtlesim_node &   # 另开终端
ros2 launch car1_pose_control pose_control.launch.py
#   ros2 topic echo /turtle1/cmd_vel 观察指令
```

> ⚠️ PYTHONPATH 必须保留 `$PYTHONPATH`（venv PyPI rclpy 陷阱）；`/cmd_vel` 有车侧 clamp。

## 测试（55 项，x86 全绿）

`pytest tests/`（含 10 项动作分类：各动作判定、优先级、vis 过滤、边界）。

## 板上验证清单

1. [ ] 单图：站姿 → `Action: 停止`；举手照片 → 对应转向/前进
2. [ ] 相机实时：站立停止、双手举=前进、左手举=左转、右手举=右转、下蹲=后退；状态栏同步
3. [ ] `:8083/pose_control.mjpg` 流畅；无人时 `Action: 停止`
4. [ ] （可选）节点模式 `publish_cmd_vel: true`：`ros2 topic echo /cmd_vel` 看到对应线/角速度；车上实机确认不超 clamp

## 已知差异（vs vendor）

- 后端 = RKNN BlazePose → RKNNLite；vendor = mediapipe solutions（板上不可装）
- vendor 11_整体检测_体感 是 holistic（face+pose+hands）**纯可视化**；本实验按 PLAN 用 pose 单模型 + 自设计动作规则，无 cmd_vel 发布逻辑可参照
- 省略 `/mediapipe/points`；新增 MJPEG 流 + 可选 cmd_vel
