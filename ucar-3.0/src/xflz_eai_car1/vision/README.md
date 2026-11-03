# AIoT_B2 课程示例工程

《机器人视觉应用与开发》12 个实验的可部署示例代码与执行文档。**总规划见 `PLAN.md`（冻结基线）**；参考代码一律只读，新代码全部在本目录。

## 结构

```
PLAN.md                  总规划（12 实验映射、环境矩阵、六段式模板、确认门）
runtime/                 板载运行环境（已核实清单 + 自检脚本）
docs/                    执行文档：00 环境速查 + 01~12 各实验（六段式）
NN_<experiment>/         各实验代码：src/ + tests/ + config/ + scripts/ + docs/build/
```
| # | 实验 | 状态 | 环境 |
|---|---|---|---|
| 01 | 工业表面缺陷检测 | ✅ 关闭（含 RGB 修复，43 用例全绿），**板端验证通过**（2026-08-06：13fps） | 训练 PC（自理）/ 部署板 |
| 02 | 垃圾分类 | ✅ 交付（64 用例全绿），**解码 RKOPT 融合导出契约修复后用户板端确认** | 板（mjpeg_streamer，清华镜像装） |
| 03 | Lab 色彩空间颜色识别 | ✅ 交付（24 用例全绿），**板端验证通过**（~50fps） | 板（mjpeg_streamer） |
| 04 | 二维码生成与识别 | ✅ 交付（19 用例全绿）；板上需补装 qrcode/pyzbar（清华镜像，libzbar0 已有） | 板（qrcode + mjpeg_streamer） |
| 05 | 多边形轮廓（课程名：自主巡线） | ✅ 交付（21 用例全绿），**板端验证通过** | 板（mjpeg_streamer） |
| 06 | 语义分割背景分割（yolov8n-seg 抠图背景替换） | ✅ 交付（40 用例全绿），**板端验证通过**（3–7fps） | 板（mjpeg_streamer） |
| 07 | DeepSORT 多目标跟踪 | ✅ 交付（40 用例全绿），**板端验证通过**（12–15fps；deepSort ID 稳定判定待用户复测） | 板（mjpeg_streamer；模型复用 Deepsort/） |
| 08 | 深度 3D 物体检测（深度查看器双栏+点击读深度） | ✅ 交付（24 用例全绿），**板端验证通过**（深度合成） | 板（mjpeg_streamer；ROS 模式用 rclpy） |
| 09 | 人脸关键点特征检测（**RKNN Blaze 468 点**，mediapipe tflite→rknn；**板上 RKNNLite 推理**） | ✅ 重建完成（2026-08-06，101 用例全绿），**板端验证通过（用户确认）** | 板（rknn-toolkit-lite2 2.3.2 + mjpeg_streamer） |
| 10 | 骨骼关键点（**BlazePose 33 点**整图，RKNNLite） | ✅ 代码完成（45 用例全绿），**模型已转换入库**（用户，pose_landmark_full.rknn）；板上验证待用户 | 板（同 09；转换环境用户装 tensorflow≤2.14） |
| 11 | 体感控制（pose 动作分类→控制指令/可选 cmd_vel） | ✅ 代码完成（58 用例全绿），**模型已转换入库**（用户）；板上验证待用户 | 板（同 09） |
| 12 | 手势识别（**palm+21 点+embedding+8 类分类**，官方检测-跟踪状态机） | ✅ 重写完成（**85 用例全绿**），**板端验证通过**（demo_image：张开手掌→Open_Palm、比耶→Victory，含 tracking 连续帧稳定性）；实机 camera_demo 待用户 | 板（同 09；用户重转 4 模型 mean 0/std 1 + rk3576 已入库） |

## 入口

- 板上自检：`python3 course_examples/runtime/board/verify_runtime.py`
- 实验 01：`course_examples/01_industrial_defect_detection/README.md`
- 实验 02：`course_examples/02_garbage_classification/README.md`

## 测试运行方式

每个实验的 `src/` 模块名相同（config/preprocess/postprocess 等），**pytest 按实验目录分别运行**（勿跨实验合并调用，避免模块名解析冲突）：

```bash
pytest course_examples/01_industrial_defect_detection/tests/   # 43 用例
pytest course_examples/02_garbage_classification/tests/        # 62 用例
```
