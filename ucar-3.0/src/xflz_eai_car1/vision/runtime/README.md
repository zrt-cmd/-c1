# Board Runtime (RK3576) — Verified Environment Manifest

> 目标机器：`iflytek@10.1.134.84`（RK3576 / Ubuntu 22.04.5 / ROS2 Humble）
> 环境来源：`~/.bashrc` 自动激活的 venv `~/ros2_venv`（Python 3.10.12，`include-system-site-packages=true`，叠加系统 apt 包）
> 状态：以下版本已在板上核实（2026-08-06 更新：新增 lite2 与 mjpeg_streamer；推理统一 RKNNLite，MJPEG 统一 mjpeg_streamer），**满足全部 12 个实验的部署侧需求**。

## 核心运行库（ros2_venv 内，均已安装）

```
rknn-toolkit2==2.3.2        # 完整版（aarch64），转换+调试用
rknn-toolkit-lite2==2.3.2   # lite 版（aarch64，cp310 wheel 见 board/rknn_toolkit_lite2-2.3.2-*.whl），板上推理标准 API（2026-08-06 起）
mjpeg-streamer==2024.2.8    # MJPEG 流服务器（2026-08-06 用户决策统一；清华镜像装；入口 require() 检测）
numpy==1.26.4
opencv-python==4.11.0.86
onnx==1.16.1
onnxruntime==1.23.2
onnxoptimizer==0.3.8
torch==2.2.0
scipy==1.15.3
pillow==12.2.0
pyserial==3.5
pyyaml==6.0.3
tqdm==4.68.4
evdev==1.9.3
sherpa-onnx==1.13.4        # 语音实验（L6）
flask==3.1.3
flask-socketio==5.6.1
python-socketio==5.16.3
pupil-apriltags==1.0.4.post11   # 手眼标定
pytest==6.2.5              # 系统 apt 包（TDD 验证用）
```

## 系统级（apt / 非 pip）

- ROS2 Humble 全套：rclpy 3.3.21、nav2_msgs、slam_toolbox 2.6.10、robot_localization 3.5.4、cv_bridge 3.2.1、tf2、rosbag2、launch_testing
- `~/ros2_ws/install`：xflz_eai_car1 全部 21 个包已构建（ascamera、car1_app、car1_navigation 等）
- `~/models/`：rkllm / tts / vad / zipformer / zipformer_kws（语音栈）
- `~/rknn_model_zoo-2.3.2.tar.gz`：转换参考工具包
- 环境变量：`ROS_DOMAIN_ID=2`；登录即 `cd ~/ros2_ws`

## 未安装（有意为之）

| 包 | 影响实验 | 处理 |
|---|---|---|
| `ultralytics` | 01/06 训练 | 训练在 PC 完成（环境你自理）；板上只需 rknn 推理，不需要 |
| `mediapipe` | 09-12 | 已废弃（实测板上 SIGILL，RK3576 ARMv8.0 无 LSE）：09-12 全部改用转换后的 rknn 模型 + `rknnlite.api.RKNNLite` 部署 |
| 板上摄像头注意 | 02-09 MJPEG/USB 模式 | 当前 USB 上是 NOVATEK ASJ ZNX_NVT（非 HP60C）：YUYV 1280×1040@8 可用，MJPG 流固件损坏（雪花）；HP60C 未接时 ascamera 无法启动 |

## 自检

```bash
source ~/ros2_venv/bin/activate
python3 course_examples/runtime/board/verify_runtime.py
# 期望：全部 PASS（或标注 WARN 的可选项）
```
