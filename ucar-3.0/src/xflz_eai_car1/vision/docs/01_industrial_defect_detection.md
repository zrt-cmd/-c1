# 实验 01 — 工业表面缺陷检测：执行文档

> 六段式：① 目标与环境自检 → ② 框架搭建 → ③ 代码编写 → ④ TDD → ⑤ Review → ⑥ 确认。
> 代码已就位（`course_examples/01_industrial_defect_detection/`）；本文件是你在对应环境逐条执行的验证手册。

## ① 实验目标与环境自检

**目标**：用 NEU-DET 训练 YOLOv8 缺陷检测模型，经 RKNN 优化导出与 int8 量化，部署到 RK3576 板（NPU 推理），并完成 torch/onnx/rknn 三方精度对比。

**环境归属**：
- **训练/导出**：PC（环境自理，版本要求见 §4-A）
- **转换/推理/评测**：板载 `ros2_venv`（已装 rknn-toolkit2 2.3.2，零新增）

**自检（板上）**：
```bash
source ~/ros2_venv/bin/activate
python3 course_examples/runtime/board/verify_runtime.py
# 期望输出：所有硬检查 PASS（numpy 1.26.4 / opencv 4.11.0.86 / rknn-toolkit2 2.3.2 / RK3576 / NPU / 相机 / 串口）
```
**自检（PC）**：确认 fork 基线可导入——
```bash
python -c "import ultralytics; print(ultralytics.__version__)"   # 期望 8.3.229（fork）
python -c "import numpy; print(numpy.__version__)"                # 期望 <2.0
```

## ② 框架搭建

目录（已建）与模块职责见 `README.md`；接口契约（TDD 断言目标）见 `PLAN.md §7`。核心数据流：

```
NEU-DET (6类, 1439/216/144)
  → [PC] yolo train (yolov8n, 640, 100ep)          → best.pt
  → [PC] fork exporter (format=rknn, 去DFL/后处理)  → best.onnx (9 输出)
  → [板] convert_rknn (int8, val 200 张校准, rk3576) → defect_yolov8n_int8.rknn
  → [板] app.py / eval_map.py (rknn.api.RKNN + CPU 后处理)
```

关键设计决定（勿改）：
1. `pad_color=(0,0,0)` —— rknn_model_zoo 要求与 RGA 管线一致（非训练侧 114）。
2. 后处理与 rknn_model_zoo `yolov8.py` **逐行对齐**：DFL 解码在 CPU（softmax@16 bins）、分数=按类最大 cls（cls_sum 分支忽略）、逐类 NMS。
3. 推理后端注入（rknn/onnx/fake），单测不打真模型。

## ③ 代码编写

已交付（8 个 src 模块 + 2 个脚本 + 6 个测试文件），全部接口按 §② 契约实现。关键点：
- `postprocess.decode_rknn`：输入 3×nl 个张量（box `(1,64,h,w)` / cls `(1,nc,h,w)` 已 sigmoid / cls_sum 忽略），DFL+grid+stride 解码 → 过滤 → NMS。
- `infer.Detector.detect`：letterbox → backend.predict → decode → unletterbox（返回原图坐标）。
- `convert_rknn`：mean/std=[0,0,0]/[255,255,255]（训练侧 /255 归一化一致），`target_platform=rk3576`。
## ④ TDD（在对应环境执行）
```bash
cd course_examples/01_industrial_defect_detection
pytest -v
```
期望：**42 个用例全 PASS**（已在 WSL rknn23 环境实测通过；NEU-DET 真实路径用例在无数据集的机器上自动 skip）。用例分布：
- test_config 5 / test_dataset 5 / test_preprocess 6 / test_postprocess 13 / test_metrics 8 / test_infer 5

无依赖环境注意：仅需 `numpy + opencv-python + pyyaml + pytest`；板上/PC 均已具备（板上 pytest 6.2.5 兼容）。

## ⑤ Review

自查清单（已在交付时执行）：
- [x] 后处理与 rknn_model_zoo 权威实现逐行核对（DFL 加权/网格/stride/阈值/NMS）
- [x] 无死代码、无占位符；`eval_map.py` 中 GT 解析在 main 内联（已删重复函数）
- [x] 接口契约与 PLAN §7 一致（模块名/签名/返回类型）
- [ ] 你运行 `pytest` 全绿后，将触发 `ecc-code-reviewer` + `ecc-python-reviewer` 二次审查（确认门前置）

## ⑥ 确认（用户验证勾选）

**A. PC 侧（训练 + 导出）**：
- [ ] fork 已装：`pip install -e ultralytics_yolov8-main`；`ultralytics.__version__ == 8.3.229`
- [ ] `bash scripts/train.sh` 完成，产物 `runs/train_neu_course/yolov8n_img640_e100/weights/best.pt`
- [ ] `yolo val model=best.pt data=NEU-DET data.yaml` → 记录 torch mAP50（基线）
- [ ] `bash scripts/export_onnx.sh <best.pt>` 完成；校验：`python -c "import onnx;m=onnx.load('<best.onnx>');print(len(m.graph.output))"` **输出 9**

**B. 板侧（转换 + 推理 + 评测）**：
- [ ] `verify_runtime.py` 全 PASS
- [ ] 同步：`scp best.onnx` 到板（或 rsync course_examples/）
- [ ] `python src/convert_rknn.py --onnx best.onnx --output models/defect_yolov8n_int8.rknn --calib-dir <NEU-DET>/valid/images --calib-count 200` → 生成 .rknn
- [ ] `python src/app.py --model models/defect_yolov8n_int8.rknn --image <缺陷图> --save out.jpg` → 正确类别框
- [ ] `python src/app.py --model models/defect_yolov8n_int8.rknn --video test.mp4` → avg_fps ≥ 15
- [ ] `python src/eval_map.py --backend onnx ... && python src/eval_map.py --backend rknn ...` → 记录两表

**C. 验收**：
- [ ] mAP50(rknn int8) ≥ mAP50(onnx) − 0.03
- [ ] 上述全部勾选 → 告知我，触发 Review 代理审查 → 实验 01 关闭，进入实验 02

## 常见问题

| 现象 | 处理 |
|---|---|
| `pytest` 中真实 NEU-DET 用例 skip | 正常——该用例在无数据集的机器自动跳过；训练 PC 上会执行 |
| 转换报算子不支持 | 确认 onnx 来自**本 fork** 导出（输出 9 个）；stock ultralytics 导出的图含 DFL/后处理，无法直接转 |
| `init_runtime` 失败 | 板需 root 权限写 `/sys/kernel/debug/rknpu`？不需要——init_runtime 仅需 `/dev/rknpu` 访问权限；确认 `ls /dev/rknpu*` |
| mAP 掉点 >3% | 先查 onnx 与 torch 差值（onnx 应为同值）；若 onnx 正常仅 rknn 差 → 增大校准集/改用 `--no-quantize` 定位量化损失 |
| 板上找不到 NEU-DET | 评测用数据集需同步到板（val 216 张，约 30MB）；或 PC 上跑 onnx 评测、板只跑 rknn 推理 |
