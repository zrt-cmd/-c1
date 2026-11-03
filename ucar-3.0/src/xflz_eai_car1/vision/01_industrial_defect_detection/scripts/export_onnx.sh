#!/usr/bin/env bash
# 实验 01 RKNN-ONNX 导出（fork 的 RKOPT 流程，RKOPT_README 第 20-33 行）
# 用法：bash course_examples/01_industrial_defect_detection/scripts/export_onnx.sh <best.pt>
# 注意：fork 的 exporter.py __main__ 不解析命令行参数（"model=xxx" 位置参数会被无视，
#       DEFAULT_CFG.model 默认 yolov8n.pt 才会触发下载）！正确方式是用 get_cfg overrides
#       （与 yolo CLI 同机制），如下。
set -euo pipefail

WS="$(cd "$(dirname "$0")/../../.." && pwd)"
FORK="$WS/ultralytics_yolov8-main"
WEIGHTS="${1:-$WS/course_examples/01_industrial_defect_detection/train/weights/best.pt}"

export PYTHONPATH="$FORK"
echo "==> 导出 RKNN 优化 ONNX（默认 format=rknn，输出 3×nl 个 raw 张量，DFL 留在 CPU 侧）"
cd "$FORK"
python -c "import sys; from ultralytics.cfg import get_cfg, DEFAULT_CFG; from ultralytics.engine.exporter import export; sys.argv=['export']; export(get_cfg(DEFAULT_CFG, overrides={'model': r'$WEIGHTS', 'format': 'rknn', 'imgsz': 640}))"

echo "==> 期望产物：${WEIGHTS%.pt}.onnx"
echo "==> 校验：onnx 输出数量 = 9（detect, 3 层 × (box, cls, cls_sum)）"
echo "    python -c \"import onnx;m=onnx.load('${WEIGHTS%.pt}.onnx');print(len(m.graph.output))\"  # 期望 9"
