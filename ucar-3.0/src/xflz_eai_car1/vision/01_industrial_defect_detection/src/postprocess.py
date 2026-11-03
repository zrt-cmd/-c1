"""RKNN 导出模型的后处理（CPU 侧，与 rknn_model_zoo yolov8.py 逐行对齐）。

本 fork 的 rknn 导出（ultralytics/nn/modules/head.py Detect.forward, format=='rknn'）
每层输出 3 个张量（共 nl*3 个）：
  [3i+0] box:   (1, 64, h, w)     原始框回归（DFL 留在 CPU 侧）
  [3i+1] cls:   (1, nc, h, w)     已 sigmoid 的类别分数
  [3i+2] cls_sum: (1, 1, h, w)    clamp(sum(cls)) 分支——本实现按 model zoo Python
                                  版本忽略（分数 = 按类最大 cls）
解码：DFL(softmax@16 bins 加权) → grid+0.5± → ×stride → xyxy(letterbox 空间)

契约（TDD 断言目标）：
- decode_rknn(outputs, num_classes, imgsz, score_thr, iou_thr) -> list[Detection]
- classwise_nms(boxes, scores, class_ids, iou_thr, score_thr) -> list[Detection]
- Detection: x1,y1,x2,y2,score,class_id（全部 float / int）
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int


def dfl_decode(position: np.ndarray) -> np.ndarray:
    """Distribution Focal Loss 解码：(n,4,16,h,w) -> (n,4,h,w)。"""
    n, c, h, w = position.shape
    mc = c // 4
    y = position.reshape(n, 4, mc, h, w).astype(np.float32)
    y = np.exp(y - y.max(axis=2, keepdims=True))
    y = y / y.sum(axis=2, keepdims=True)  # softmax over bins
    bins = np.arange(mc, dtype=np.float32).reshape(1, 1, mc, 1, 1)
    return (y * bins).sum(axis=2)


def box_process(position: np.ndarray, imgsz: int) -> np.ndarray:
    """单层框解码 -> xyxy（letterbox 像素空间）。(n,4,h,w) -> (h*w, 4)。"""
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
    grid = np.stack([col, row], axis=0).astype(np.float32)[None]  # (1,2,h,w)
    stride = np.array([imgsz / grid_h, imgsz / grid_w], dtype=np.float32).reshape(1, 2, 1, 1)

    pos = dfl_decode(position)
    box_xy = grid + 0.5 - pos[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + pos[:, 2:4, :, :]
    xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)  # (1,4,h,w)
    return xyxy.transpose(0, 2, 3, 1).reshape(-1, 4)


def classwise_nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_ids: np.ndarray,
    iou_thr: float = 0.45,
    score_thr: float = 0.25,
) -> list[Detection]:
    """逐类 NMS（与 model zoo nms_boxes 一致：纯 numpy，无框则返回空）。"""
    kept: list[Detection] = []
    if boxes.size == 0:
        return kept
    for c in np.unique(class_ids):
        idx = np.where(class_ids == c)[0]
        b, s = boxes[idx], scores[idx]
        order = s.argsort()[::-1]
        x1, y1 = b[:, 0], b[:, 1]
        x2, y2 = b[:, 2], b[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        while order.size > 0:
            i = order[0]
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
            order = order[np.where(iou <= iou_thr)[0] + 1]
            kept.append(Detection(float(b[i][0]), float(b[i][1]), float(b[i][2]),
                                   float(b[i][3]), float(s[i]), int(c)))
    return sorted(kept, key=lambda d: d.score, reverse=True)


def decode_rknn(
    outputs: Sequence[np.ndarray],
    num_classes: int,
    imgsz: int = 640,
    score_thr: float = 0.25,
    iou_thr: float = 0.45,
) -> list[Detection]:
    """解码全部层输出 -> 过滤 -> 逐类 NMS。返回 letterbox 空间的检测。"""
    n_levels = len(outputs) // 3
    assert len(outputs) % 3 == 0, f"expected 3 outputs per level, got {len(outputs)}"
    boxes_list: list[np.ndarray] = []
    scores_list: list[np.ndarray] = []
    classes_list: list[np.ndarray] = []

    for i in range(n_levels):
        box = outputs[3 * i]                      # (1,64,h,w)
        cls = outputs[3 * i + 1]                  # (1,nc,h,w)，已 sigmoid
        if cls.shape[1] != num_classes:
            raise ValueError(f"level {i}: cls channels {cls.shape[1]} != num_classes {num_classes}")
        boxes_list.append(box_process(box, imgsz))
        ch = cls.shape[1]
        flat = cls.transpose(0, 2, 3, 1).reshape(-1, ch)
        scores_list.append(flat.max(axis=1))
        classes_list.append(flat.argmax(axis=1))

    boxes = np.concatenate(boxes_list)
    scores = np.concatenate(scores_list)
    class_ids = np.concatenate(classes_list)

    mask = scores >= score_thr
    return classwise_nms(boxes[mask], scores[mask], class_ids[mask], iou_thr=iou_thr)
