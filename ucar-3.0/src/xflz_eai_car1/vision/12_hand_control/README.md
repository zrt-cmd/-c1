# 实验 12 — 手指控制/手势识别（CSDN 方案 4 模型管线）

> **手势识别完整管线**：手掌检测 → 21 点关键点 → 特征嵌入 → 8 类手势分类
> （mediapipe gesture_recognizer.task 解包 4 模型转 rknn），叠加拇指-食指夹角特效 + 手指计数。
> 双栏（左=特效帧+FPS 红字，右=黑底手骨架 + 手势名 + 仪表），'f' 键切特效。

## 技术方案

| 环节 | 选型 |
|---|---|
| 模型 | ① `hand_detector`（192×192 手掌检测，blazeface 2016 锚）② `hand_landmarks_detector`（224×224，21 点 + presence + handedness + world）③ `gesture_embedder`（21 点→128 维）④ `canned_gesture_classifier`（128 维→8 类） |
| 转换 | `gesture_recognizer/convert.py`（.task 解包 4 模型；①② mean 0/std 1 **float32 [0,1] 输入**（板端实测 uint8 直喂→输出爆炸）；③ `norm_tensor` 单输入（x/y/z 全部 /224）；④ float 无 mean/std；fp16 默认） |
| 管线 | 手掌检测（letterbox 192 float/255）→ NMS → union 合并（特写大图 palm 分裂多框）→ 21 点（224，**[x,y,z] 直读**）→ **官方检测-跟踪状态机**（上帧 presence≥0.5 跳过 palm，ROI 由上帧 21 点生成；丢失回退全图）→ embedder（norm_tensor）→ softmax 8 类手势 |
| 手势 | None / Closed_Fist / Open_Palm / Pointing_Up / Thumb_Down / Thumb_Up / Victory / ILoveYou |
| 板上推理 | `RKNNLiteBackend`（core_mask 0=AUTO） |
| 取流 | `--ros` = ascamera 话题（标准）；`--camera N` = V4L2 回退 |
| 控制 | 拇指-食指夹角 angle(4,0,8)（余弦定理）→ [0,70]° → value [0,255] → 5 种特效（color/thresh/blur/hue/enhance），'f' 循环切换 |
| 计数 | fingersUp（vendor 11_手势识别）：拇指双角 150° + 四指 tip.y<pip.y → 状态栏 Fingers: N |
| 显示 | 左=特效帧+FPS，右=黑底骨架（红点 r6 + 绿线 2px）+ 仪表（竖条 value 255→0）+ 状态栏 `FPS Hands Fingers [特效]` |
| 流 | MJPEG `:8084/hand.mjpg` |

## 运行

```bash
# 转换（x86 转换环境，tensorflow≤2.14 用户安装）：
PYTHONPATH=car1_hand_control python3 -m hand_control_core.convert_mediapipe --all
PYTHONPATH=car1_hand_control python3 -m hand_control_core.convert_mediapipe --check
# 1) 单图：状态栏显示手数/手指数
PYTHONPATH=car1_hand_control python3 -m hand_control_core.demo_image \
  --config config/hand_control_config.yaml --display --save out.jpg
# 2) 相机实时（标准）：
ros2 launch ascamera hp60c.launch.py
PYTHONPATH=car1_hand_control:$PYTHONPATH python3 -m hand_control_core.camera_demo \
  --config config/hand_control_config.yaml --ros --display   # 或 :8084/hand.mjpg；'f' 切特效
# 3) 无 ROS 回退
PYTHONPATH=car1_hand_control:$PYTHONPATH python3 -m hand_control_core.camera_demo \
  --config config/hand_control_config.yaml --camera 25 --display
# 4) ROS2 节点（可选）
cd ~/ros2_ws && colcon build --symlink-install --packages-select car1_hand_control
ros2 launch car1_hand_control hand_control.launch.py
```

> ⚠️ PYTHONPATH 必须保留 `$PYTHONPATH`（venv PyPI rclpy 陷阱）。

## 实现（src 逻辑，x86 全可测）

| 模块 | 职责 |
|---|---|
| `hand.py` | 几何：HAND_CONNECTIONS 21 边、palm 解码（blazeface 2016 锚/NMS）、union_bbox 合并、detection2roi（官方旋转方形 ROI）、roi_affine 仿射/逆映射、21 点解码 |
| `engine.py` | `HandEngine` 官方状态机（detection/tracking；presence 过滤）+ `draw_hands` |
| `fingers.py` | fingersUp 计数（vendor 语义）+ 拇指-食指夹角/控制值映射 |
| `effects.py` | 5 种特效（vendor 12：thresh/blur/hue/enhance/color） |
| `camera_demo.py` | 实时循环 + 'f' 切特效 + 仪表 + MJPEG |
| `convert_mediapipe.py` | 双模型转换 + `--check` 契约自检 |
| 共享 | config/view/mjpeg_server/ros2_source/backend（09 模式） |

## 测试（58 项，x86 全绿）

`pytest tests/`：拓扑/锚/解码/逆映射/两阶段回映射/手数截断/分数过滤/手指计数/夹角映射/5 特效/双栏/CLI/MJPEG。

## 板上验证清单（用户执行，结果反馈后按需修改）

1. [ ] 单图：伸手照片 → 右栏 21 点骨架 + 手势名（如 Open_Palm/Victory），状态栏 Hands/Fingers
2. [ ] 相机实时：手伸向镜头 → 骨架实时跟随 + 手势分类稳定；'f' 键切换 5 种特效
3. [ ] 拇指食指张开/捏合 → 仪表条升降（value 0~255）+ 特效强度变化
4. [ ] `:8084/hand.mjpg` 流畅；无手不崩；`q` 退出干净

> ⚠️ 参考（test_board.py）未经实机验证：若检测不到手/手势错，反馈现象，
> 重点核查输入契约（convert.py 配置 mean 127.5/std 128 期望 uint8 输入；预检显示
> uint8 输入板上输出可能饱和——实测 float/255 输入 cls 值域正常，可一键切换）。

## 已知差异（vs vendor 12_手指控制）

- 后端 = RKNN palm+hand → RKNNLite；vendor = mediapipe solutions（板上不可装）
- vendor 特效切换为 ROS1 脚本 + 窗口按键；本实验同按键语义（'f'）且 MJPEG 流同步推特效帧
- 省略 `/mediapipe/points`；vendor 无手指计数（来自 11_手势识别 fingersUp），本实验状态栏展示
- vendor 手骨架 r=15/线 10px（大屏）；本实验取 09 系 r6/2px 以适配 640 宽帧
