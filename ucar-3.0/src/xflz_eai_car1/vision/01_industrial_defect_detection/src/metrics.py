"""mAP50 指标（纯 numpy，不依赖 torchmetrics）。

契约（TDD 断言目标）：
- iou_matrix(boxes1, boxes2) -> (N,M) IoU 矩阵
- map50(preds, gts, num_classes, iou_thr=0.5) -> MAPResult(map50, per_class)
  preds/gts 均为每图 list[Detection]；逐图贪心匹配（按置信度降序，
  同类别 + IoU>=thr，一张 GT 只匹配一次）+ all-point 插值 AP。
  mAP 均值遵循 COCO/ultralytics 约定：无 GT 的类不参与均值（与 yolo val 可比）。

约定：GT 与 pred 用同一坐标空间（原始图像坐标即可）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np

from postprocess import Detection


@dataclass(frozen=True)
class MAPResult:
    map50: float
    per_class: dict[int, float] = field(default_factory=dict)


def iou_matrix(boxes1: np.ndarray, boxes2: np.ndarray) -> np.ndarray:
    """(N,4) xyxy 与 (M,4) xyxy 的 IoU 矩阵。"""
    if boxes1.size == 0 or boxes2.size == 0:
        return np.zeros((len(boxes1), len(boxes2)), dtype=np.float32)
    x1 = np.maximum(boxes1[:, None, 0], boxes2[None, :, 0])
    y1 = np.maximum(boxes1[:, None, 1], boxes2[None, :, 1])
    x2 = np.minimum(boxes1[:, None, 2], boxes2[None, :, 2])
    y2 = np.minimum(boxes1[:, None, 3], boxes2[None, :, 3])
    inter = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    a1 = (boxes1[:, 2] - boxes1[:, 0]) * (boxes1[:, 3] - boxes1[:, 1])
    a2 = (boxes2[:, 2] - boxes2[:, 0]) * (boxes2[:, 3] - boxes2[:, 1])
    return inter / (a1[:, None] + a2[None, :] - inter + 1e-6)


def _ap_from_pr(precisions: np.ndarray, recalls: np.ndarray) -> float:
    """All-point 插值 AP（precision-recall 曲线下面积）。"""
    if recalls.size == 0:
        return 0.0
    prec = np.concatenate(([0.0], precisions, [0.0]))
    rec = np.concatenate(([0.0], recalls, [1.0]))
    for i in range(len(prec) - 2, -1, -1):  # 单调包络
        prec[i] = max(prec[i], prec[i + 1])
    idx = np.where(rec[1:] != rec[:-1])[0]
    return float(np.sum((rec[idx + 1] - rec[idx]) * prec[idx + 1]))


def _match_flags(preds: list[Detection], gts: list[Detection], iou_thr: float) -> list[bool]:
    """逐框 TP/FP 标志（preds 须已按置信度降序；一张 GT 只匹配一次）。"""
    flags: list[bool] = []
    matched: set[int] = set()
    if gts:
        gt_boxes = np.array([[g.x1, g.y1, g.x2, g.y2] for g in gts], dtype=np.float32)
        for p in preds:
            cand = [i for i, g in enumerate(gts) if g.class_id == p.class_id and i not in matched]
            hit = False
            if cand:
                ious = iou_matrix(np.array([[p.x1, p.y1, p.x2, p.y2]], dtype=np.float32),
                                  gt_boxes[cand])[0]
                best = int(np.argmax(ious))
                if ious[best] >= iou_thr:
                    hit = True
                    matched.add(cand[best])
            flags.append(hit)
    else:
        flags = [False] * len(preds)
    return flags


def map50(
    preds: Sequence[Iterable[Detection]],
    gts: Sequence[Iterable[Detection]],
    num_classes: int,
    iou_thr: float = 0.5,
) -> MAPResult:
    """计算 mAP@0.5。preds/gts: 每图一个 iterable[Detection]。"""
    preds = [list(p) for p in preds]
    gts = [list(g) for g in gts]
    per_class_ap: dict[int, float] = {}
    has_gt: dict[int, bool] = {}
    for c in range(num_classes):
        has_gt[c] = False
        scored: list[tuple[float, bool]] = []
        n_gt_total = 0
        for ps, gs in zip(preds, gts):
            ps_c = sorted((p for p in ps if p.class_id == c), key=lambda d: d.score, reverse=True)
            gs_c = [g for g in gs if g.class_id == c]
            n_gt_total += len(gs_c)
            if gs_c:
                has_gt[c] = True
            flags = _match_flags(ps_c, gs_c, iou_thr)
            scored.extend((p.score, f) for p, f in zip(ps_c, flags))
        if not scored or n_gt_total == 0:
            per_class_ap[c] = 0.0
            continue
        scored.sort(key=lambda t: t[0], reverse=True)
        tp = np.cumsum([1.0 if f else 0.0 for _, f in scored])
        fp = np.cumsum([0.0 if f else 1.0 for _, f in scored])
        recalls = tp / n_gt_total
        precisions = tp / np.maximum(tp + fp, 1e-6)
        per_class_ap[c] = _ap_from_pr(precisions, recalls)

    # COCO/ultralytics 约定：无 GT 的类不参与均值（与 yolo val 可比）
    mean_classes = [c for c in range(num_classes) if has_gt[c]]
    overall = float(np.mean([per_class_ap[c] for c in mean_classes])) if mean_classes else 0.0
    return MAPResult(map50=overall, per_class=per_class_ap)
