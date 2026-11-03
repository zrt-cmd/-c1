# 实验 01 — 工业表面缺陷检测（YOLOv8 → RK3576）

基于 NEU-DET（6 类：crazing / inclusion / patches / pitted_surface / rolled-in_scale / scratches）
的完整流水线：**训练（PC）→ RKNN-ONNX 导出（PC）→ rknn 转换（板/PC）→ 板上推理与 mAP50 评测**。

## 目录

```
config/defect_config.yaml   训练/推理/转换参数（含 schema 校验）
src/config.py               配置加载 + 校验（ConfigError）
src/dataset.py              NEU-DET 数据校验与统计
src/preprocess.py           letterbox（pad 0，与 rknn_model_zoo 对齐）
src/postprocess.py          DFL 解码 + 逐类 NMS（与 model zoo 逐行对齐）
src/metrics.py              mAP50（纯 numpy）
src/infer.py                后端抽象（rknn/onnx/fake）+ Detector 编排
src/convert_rknn.py         onnx -> rknn（int8 量化，target rk3576）
src/eval_map.py             val 集 mAP50 评测（onnx/rknn 后端）
src/app.py                  推理 CLI（图片/视频/相机 + FPS）
scripts/train.sh            PC 训练入口
scripts/export_onnx.sh      fork RKOPT 导出入口
tests/                      pytest 套件（环境无关，全部可单机跑）
```

## 已包含的训练产物（train/）

本实验已随仓库交付一份完整训练产物（用户 2026-08-05 训练并入库）：
- **超参**：yolov8n、NEU-DET 6 类、imgsz=640、epochs=100、batch=8、seed=0、optimizer=auto
  （完整参数见 `train/args.yaml`）
- **权重**：`train/weights/best.pt`（+ last.pt）、`train/weights/best.onnx`（RKNN 优化，9 输出）
- **指标**（epoch 100）：P=0.657 / R=0.706 / **mAP50=0.732** / mAP50-95=0.409
  （数据源 `results.csv`；PR/F1 曲线、混淆矩阵等见 `train/*.png`）
- 板上转换/推理直接用 `train/weights/best.onnx`，无需重新训练；重训可 `scripts/train.sh`。
- 转换产物已随仓库交付：`models/defect_yolov8n_int8.rknn`（int8、单核+剪枝、200 张 val 校准），板上推理可直接用。

## 快速开始

```bash
# 1) 单元测试（任意装有 numpy/opencv/pyyaml 的环境，板上/PC 皆可）
cd course_examples/01_industrial_defect_detection
pytest -v            # 期望：全部 PASS（数据集真实路径用例在无 NEU-DET 的机器上自动跳过）

# 2) 重训（可选，PC）→ 导出（详见 docs/01_industrial_defect_detection.md §5）
bash scripts/train.sh            # yolov8n imgsz=640
bash scripts/export_onnx.sh train/weights/best.pt

# 3) 转换（板载 ros2_venv，或 PC rknn env；onnx 已随仓库交付）
python src/convert_rknn.py --onnx train/weights/best.onnx --output models/defect_yolov8n_int8.rknn \
    --calib-dir <NEU-DET>/valid/images --calib-count 200

# 4) 板上推理
python src/app.py --model models/defect_yolov8n_int8.rknn --image <缺陷图> --save out.jpg
python src/app.py --model models/defect_yolov8n_int8.rknn --video test.mp4

# 5) 相机实时（板上标准：HP60C 深度相机走 ascamera 话题，opencv 直连读不了）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
python src/app.py --model models/defect_yolov8n_int8.rknn --ros
#    MJPEG: http://<板IP>:8080/defect.mjpg（--display 开本地窗口，q 退出）
#    --camera N 仅作无 ROS 的 UVC 回退（本板 opencv 未编 libv4l，HP60C 直连失败）

# 5) 精度对比
python src/eval_map.py --backend onnx --model train/weights/best.onnx --data <NEU-DET>
python src/eval_map.py --backend rknn --model models/defect_yolov8n_int8.rknn --data <NEU-DET>
```

## 验收标准（确认门，见执行文档 §8）
- `pytest -v` 全绿（42 用例，已在 WSL rknn23 实测）
- 转换成功：`best.onnx`（9 输出）→ `defect_yolov8n_int8.rknn`
- mAP50：rknn(int8) 相对 onnx 掉点 ≤3%（参考基线：Steel_Plate 项目 imgsz=800 mAP50≈0.764，imgsz=640 以实测为准）
- 板上 fps：yolov8n/640 ≥ 15 FPS（NPU）
- `--ros` 相机实时可用（ascamera 驱动先起；帧率打印正常）+ MJPEG `:8080/defect.mjpg` 可见
