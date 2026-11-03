# 00 — 课程环境矩阵速查

> 详见 `course_examples/runtime/README.md`（板载已核实清单）与 `runtime/board/verify_runtime.py`（自检脚本）。

## 板载（10.1.134.84，RK3576 / Ubuntu 22.04.5 / Humble）

- **环境**：`~/ros2_venv`（py3.10.12，`.bashrc` 自动激活，含系统包）
- **入口**：`source ~/ros2_venv/bin/activate && cd ~/ros2_ws`
- **已装**（18 包，版本见 `runtime/board/requirements.txt`）：rknn-toolkit2 2.3.2（转换+推理）、numpy 1.26.4、opencv-python 4.11.0.86、onnx 1.16.1、onnxruntime 1.23.2、torch 2.2.0、scipy 1.15.3、pyyaml 6.0.3、sherpa-onnx 1.13.4、flask+socketio、pupil-apriltags、pytest 6.2.5
- **自检**：`python3 course_examples/runtime/board/verify_runtime.py`（期望全 PASS）

## 各实验环境归属

| 实验 | 运行环境 | 备注 |
|---|---|---|
| 01 缺陷检测 | 训练=PC（自理）／部署=板 | 板上零新增 |
| 02 垃圾分类 | 板 | 模型用户提供 |
| 03 Lab 颜色 / 04 二维码 | 板 | 零新增 |
| 05 巡线 / 08 3D 检测 | 板 + ROS2 | 复用已构建 car1 |
| 06 语义分割 | 训练=PC／部署=板 | fork segment 分支 |
| 07 DeepSORT | 板 | scipy 已装 |
| 09-12 人脸/骨骼/体感/手指 | 板 | ⚠️ mediapipe 决策点（PLAN §2） |
| 语音（L6） | 板 | sherpa-onnx + ~/models 已装 |

## 训练侧（PC，用户自理）

- 基线：仓库 fork `ultralytics_yolov8-main`（ultralytics 8.3.229，`pip install -e .`），numpy<2.0
- 你的 conda 环境现状（已核实，2026-08-04）：`yolov8`（py3.8 / torch 1.10.1 / ultralytics 8.0.20）**不满足** fork 基线；`rknn` 环境**未装** rknn-toolkit2；`yolov11`（py3.9 / torch 1.8.1 / ultralytics 8.3.105）接近但 torch 过旧；GPU GTX 1060 6GB 驱动 442.23 仅支持 CUDA≤11.0（torch 2.x 不可用）。**结论：环境需按 fork 要求重建/升级（由你处理），文档只给版本要求。**
