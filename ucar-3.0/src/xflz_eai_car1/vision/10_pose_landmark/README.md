# 实验 10 — 骨骼关键点（BlazePose 33 点）

> 姿态关键点双栏显示 + 实时 FPS。**方案 = mediapipe `pose_landmark_full` tflite 转 RKNN
> （fp16/rk3576），板上 RKNNLite NPU 推理 33 点骨架**（vendor 10_姿态检测 视觉对齐：
> 左=原图+FPS 红字，右=黑底红点绿线骨架）。

## 技术方案

| 环节 | 选型 |
|---|---|
| 模型 | `pose_landmark_full.tflite`（mediapipe 官方，整图 256×256 输入，无独立检测器；33 关键点 [x,y,z,vis] + presence + 分割/world 输出） |
| 转换 | `convert_mediapipe.py`（**mean 127.5/std 128 + uint8 输入**（rknn 内部归一化，板端实测；手动 /127.5−1 破坏输入），对齐 mediapipe-rknn/models/pose_landmark/convert.py；fp16 默认） |
| 板上推理 | `RKNNLiteBackend`（rknn-toolkit-lite2 2.3.2，core_mask 默认 0=AUTO） |
| 取流 | `--ros` = ascamera 话题（标准）；`--camera N` = V4L2 回退（NOVATEK 相机强制 YUYV） |
| 显示 | 双栏：左=原图+FPS（红字），右=黑底 33 点骨架（红点 r6 + 绿线 2px，vis<0.5 跳过）+ 状态栏（FPS: N Pose: OK/--） |
| 流 | MJPEG `:8082/pose.mjpg`（09=8081 之后，避免冲突） |

## 环境与部署

```bash
# 依赖（板上）：rknn-toolkit-lite2 2.3.2 + mjpeg_streamer（runtime/board/requirements.txt）
# 转换（x86 转换环境，用户安装 tensorflow≤2.14 + ml-dtypes 0.2.0 后）：
PYTHONPATH=car1_pose_landmark python3 -m pose_landmark_core.convert_mediapipe --all   # fp16
PYTHONPATH=car1_pose_landmark python3 -m pose_landmark_core.convert_mediapipe --check  # 契约自检
# 产物 models/pose_landmark_full.rknn 入库
```

## 运行

```bash
# 1) 单图（x86/板上都可）：左原图 + 右骨架
PYTHONPATH=car1_pose_landmark python3 -m pose_landmark_core.demo_image \
  --config config/pose_config.yaml --display --save out.jpg
# 2) 板上相机实时（标准）：先起驱动
ros2 launch ascamera hp60c.launch.py
PYTHONPATH=car1_pose_landmark:$PYTHONPATH python3 -m pose_landmark_core.camera_demo \
  --config config/pose_config.yaml --ros --display   # 或浏览器 :8082/pose.mjpg
# 3) 无 ROS 回退
PYTHONPATH=car1_pose_landmark:$PYTHONPATH python3 -m pose_landmark_core.camera_demo \
  --config config/pose_config.yaml --camera 25 --display
# 4) ROS2 节点（可选，需先编译）
cd ~/ros2_ws && colcon build --symlink-install --packages-select car1_pose_landmark
ros2 launch car1_pose_landmark pose_landmark.launch.py
```

> ⚠️ PYTHONPATH 必须保留 `$PYTHONPATH`（覆盖会丢 Humble 路径 → venv 内 PyPI rclpy → ModuleNotFoundError）。

## 实现（src 逻辑，x86 全可测）

| 模块 | 职责 |
|---|---|
| `pose.py` | BlazePose 几何：`POSE_CONNECTIONS` 35 边拓扑、`resize_pad` letterbox、`decode_pose`（33×4 解析 + 逆映射 `px=(norm×256−pad)/scale`）、`prepare`（RGB [-1,1]） |
| `engine.py` | `PoseEngine` 单模型整图引擎（后端注入；presence<阈值 → 空结果）+ `draw_skeleton`（vis 过滤） |
| `config.py` | schema 校验 `load_config()`（单模型、core_mask、source.mode） |
| `view.py` | 双栏合成 + 状态栏（Pose 特化文本）+ FPS 红字 |
| `mjpeg_server.py` / `ros2_source.py` / `backend.py` | 09 字节一致拷贝（流/ROS 取流/RKNNLite 协议） |
| `camera_demo.py` | 实时循环（--ros/--camera/MJPEG/q 退出；cap/engine 注入） |
| `demo_image.py` | 单图模式（--save/--display；缺图 exit 2） |
| `convert_mediapipe.py` | tflite→rknn（127.5/128 + uint8 契约）+ `--check` 头部 JSON 自检 |
| `pose_landmark_node.py` | ROS2 节点（订阅 RGB → 双栏 → MJPEG；`remove_ros_args` 已修） |

## 测试（45 项，x86 全绿）

`pytest tests/`（用 ros2_venv 解释器；不得跨实验合跑——共享模块名）。

## 板上验证清单

1. [ ] 单图：`demo_image --display --save out.jpg` —— 右栏黑底骨架贴合人体（33 点红点 + 绿线，站姿可辨）
2. [ ] 相机实时：`camera_demo --ros` —— 骨架实时跟随，左栏 FPS 红字；浏览器 `:8082/pose.mjpg` 流畅
3. [ ] 无脸/无人时不崩，状态栏 `Pose: --`
4. [ ] `q` 退出干净；`convert_mediapipe.py --check` 契约通过

## 已知差异（vs vendor 10_姿态检测）

- 后端 = RKNN BlazePose 整图（33 点，mediapipe 模型）→ RKNNLite NPU；vendor = mediapipe solutions（板上不可装）
- 省略 `/mediapipe/points` 发布（板无 yahboomcar_msgs）；新增 MJPEG 流
- 单模型整图推理（mediapipe-rknn 参考即此形态；vendor 的 mpPose 内部同为整图管线）
- 动作分类/控制指令 → 实验 11（体感控制）
