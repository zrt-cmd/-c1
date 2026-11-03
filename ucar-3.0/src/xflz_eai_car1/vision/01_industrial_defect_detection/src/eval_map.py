"""mAP50 评测（onnx / rknn 后端，在 val 集上与 GT 对比）。

用法（板上或 PC，ros2_venv / 训练环境）：
    python src/eval_map.py --backend rknn  --model models/defect_yolov8n_int8.rknn \
        --data <NEU-DET 根目录> --config config/defect_config.yaml
    python src/eval_map.py --backend onnx --model runs/.../weights/best.onnx ...

输出：per-class AP + mAP50 表格（stdout）。
说明：torch 基线用 ultralytics `yolo val` 在 PC 侧得到（官方指标，见执行文档）。
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import cv2

from config import load_config
from dataset import class_names, IMG_EXTS
from infer import Detector, DetectorConfig, RKNNBackend, ONNXBackend
from metrics import map50
from postprocess import Detection




def main() -> int:
    ap = argparse.ArgumentParser(description="mAP50 eval for onnx/rknn backends")
    ap.add_argument("--backend", choices=["rknn", "onnx"], required=True)
    ap.add_argument("--model", required=True, help=".rknn 或 .onnx 权重")
    ap.add_argument("--data", required=True, help="NEU-DET 数据集根目录")
    ap.add_argument("--config", default="config/defect_config.yaml")
    args = ap.parse_args()

    cfg = load_config(args.config)
    inf = cfg["infer"]
    data_root = Path(args.data)
    names = class_names(data_root / cfg["dataset"]["data_yaml"])
    split = cfg["eval"]["split"]

    det_cfg = DetectorConfig(imgsz=inf["imgsz"], score_thr=inf["score_thr"],
                             iou_thr=inf["iou_thr"], pad_color=tuple(inf["pad_color"]))
    backend = RKNNBackend() if args.backend == "rknn" else ONNXBackend()
    detector = Detector(backend, len(names), det_cfg).load(args.model)

    img_dir = data_root / split / "images"
    lbl_dir = data_root / split / "labels"
    images = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMG_EXTS)
    if not images:
        print(f"no images under {img_dir}")
        return 2

    preds_all, gts_all = [], []
    t0 = time.perf_counter()
    for i, img_p in enumerate(images, 1):
        img = cv2.imread(str(img_p))
        if img is None:
            continue
        h, w = img.shape[:2]
        preds_all.append(detector.detect(img))
        lbl = lbl_dir / f"{img_p.stem}.txt"
        gts: list[Detection] = []
        if lbl.exists():
            for line in lbl.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) != 5:
                    continue
                cls, x, y, bw, bh = (float(p) for p in parts)
                gts.append(Detection((x - bw / 2) * w, (y - bh / 2) * h,
                                     (x + bw / 2) * w, (y + bh / 2) * h, 1.0, int(cls)))
        gts_all.append(gts)
        if i % 50 == 0:
            print(f"  eval {i}/{len(images)}")
    dt = time.perf_counter() - t0

    result = map50(preds_all, gts_all, len(names), iou_thr=cfg["eval"]["iou_thr"])
    print(f"\nbackend={args.backend} model={args.model} split={split} images={len(images)} "
          f"time={dt:.1f}s fps={len(images) / dt:.1f}")
    for c, ap in result.per_class.items():
        print(f"  AP50 {names[c]:<16s} {ap:.4f}")
    print(f"  mAP50 {result.map50:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
