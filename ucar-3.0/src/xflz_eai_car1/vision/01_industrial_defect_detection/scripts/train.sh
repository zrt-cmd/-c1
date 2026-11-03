#!/usr/bin/env bash
# 实验 01 训练（PC 侧，环境自理）
# 用法：bash course_examples/01_industrial_defect_detection/scripts/train.sh [device]
# 前置：conda 环境已激活；fork 基线 = ultralytics 8.3.229（pip install -e 见下），numpy<2.0
set -euo pipefail

WS="$(cd "$(dirname "$0")/../../.." && pwd)"
FORK="$WS/ultralytics_yolov8-main"
DATA="$WS/NEU-DET.v1i.yolov8/data.yaml"
DEVICE="${1:-0}"   # 无 CUDA 时传 cpu

echo "==> 安装 fork（RKNN 优化版 ultralytics）"
(cd "$FORK" && pip install -e . -q)

echo "==> 训练 yolov8n imgsz=640"
yolo train \
  data="$DATA" \
  model=yolov8n.pt \
  epochs=100 imgsz=640 batch=8 \
  device="$DEVICE" \
  project="$WS/runs/train_neu_course" \
  name=yolov8n_img640_e100

echo "==> 产物：$WS/runs/train_neu_course/yolov8n_img640_e100/weights/best.pt"
