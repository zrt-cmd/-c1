"""YOLOv5 式解码 + NMS + 4 大类映射（与参考 func.py 算法一致，可单测化）。

模型输出：3 尺度 (1,75,h,w)，75 = 3 anchors × (4 box + 1 obj + 20 cls)。
与参考实现的差异：sigmoid 应用可配置（`infer.sigmoid`，默认 true——若导出图
已含 sigmoid 只是分数单调压缩，结果不受影响）；网格由张量形状推导。

契约（TDD 断言目标）：
- decode_scale(tensor, anchors_3, stride, score_thr, sigmoid) -> (boxes, scores, cls_ids)
  boxes 为 xywh（letterbox 像素空间）
- classwise_nms(boxes_xyxy, scores, cls_ids, iou_thr) -> list[Detection]
- decode_rknn_outputs(outputs, anchors, imgsz, score_thr, nms_thr, sigmoid)
  -> list[Detection]（letterbox 空间 xyxy）；输出数≠3/通道≠75 -> ValueError
- map_categories(dets, cat_map, cat_names) -> list[CategoryDetection]
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


@dataclass(frozen=True)
class CategoryDetection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int
    category: int
    category_name: str

    @classmethod
    def from_detection(cls, d: Detection, category: int, category_name: str) -> "CategoryDetection":
        return cls(d.x1, d.y1, d.x2, d.y2, d.score, d.class_id, category, category_name)


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def decode_scale(
    tensor: np.ndarray,
    anchors_3: Sequence[tuple[float, float]],
    stride: float,
    score_thr: float,
    sigmoid: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """解码单尺度 (1,75,h,w) -> (xywh_boxes, scores, class_ids)（letterbox 空间）。"""
    if tensor.shape[1] != 75:
        raise ValueError(f"expected 75 channels, got {tensor.shape[1]}")
    nc = 20  # 75 = 3 × (5 + 20)
    t = tensor[0].transpose(1, 2, 0).reshape(tensor.shape[2], tensor.shape[3], 3, 5 + nc)
    if sigmoid:
        t = _sigmoid(t.astype(np.float32))

    grid_h, grid_w = t.shape[0], t.shape[1]
    col, row = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
    grid = np.stack([col, row], axis=-1).astype(np.float32)[..., None, :]  # (h,w,1,2)

    box_conf = t[..., 4]                                     # (h,w,3)
    box_cls = t[..., 5:]                                     # (h,w,3,nc)
    box_xy = t[..., :2] * 2 - 0.5                            # (h,w,3,2)
    box_xy = (box_xy + grid) * stride
    anchors = np.asarray(anchors_3, dtype=np.float32)        # (3,2)
    box_wh = np.square(t[..., 2:4] * 2) * anchors[None, None, :, :]
    boxes = np.concatenate([box_xy, box_wh], axis=-1).reshape(-1, 4)  # xywh
    conf = box_conf.reshape(-1)
    cls = box_cls.reshape(-1, nc)

    cls_max = cls.max(axis=-1)
    cls_id = cls.argmax(axis=-1)
    keep = np.where((conf >= score_thr) & (cls_max >= score_thr))[0]
    return boxes[keep], (cls_max * conf)[keep], cls_id[keep]


def classwise_nms(
    boxes_xyxy: np.ndarray,
    scores: np.ndarray,
    cls_ids: np.ndarray,
    iou_thr: float = 0.45,
) -> list[Detection]:
    """逐类 NMS（与参考 nms_boxes 一致，含 +0.00001 容差）。"""
    kept: list[Detection] = []
    if boxes_xyxy.size == 0:
        return kept
    for c in np.unique(cls_ids):
        idx = np.where(cls_ids == c)[0]
        b, s = boxes_xyxy[idx], scores[idx]
        x, y = b[:, 0], b[:, 1]
        w, h = b[:, 2] - b[:, 0], b[:, 3] - b[:, 1]
        areas = w * h
        order = s.argsort()[::-1]
        while order.size > 0:
            i = order[0]
            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
            inter = np.maximum(0.0, xx2 - xx1 + 0.00001) * np.maximum(0.0, yy2 - yy1 + 0.00001)
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[np.where(ovr <= iou_thr)[0] + 1]
            kept.append(Detection(float(b[i][0]), float(b[i][1]),
                                  float(b[i][2]), float(b[i][3]), float(s[i]), int(c)))
    return sorted(kept, key=lambda d: d.score, reverse=True)


def decode_rknn_outputs(
    outputs: Sequence[np.ndarray],
    anchors: Sequence[tuple[float, float]],
    imgsz: int = 640,
    score_thr: float = 0.25,
    nms_thr: float = 0.45,
    sigmoid: bool = True,
) -> list[Detection]:
    """解码全部 3 个尺度 -> 过滤 -> 逐类 NMS（letterbox 空间 xyxy）。"""
    if len(outputs) != 3:
        raise ValueError(f"expected 3 scale outputs, got {len(outputs)}")
    if len(anchors) != 9:
        raise ValueError(f"expected 9 anchors, got {len(anchors)}")

    boxes_list, scores_list, cls_list = [], [], []
    for i, tensor in enumerate(outputs):
        stride = imgsz / tensor.shape[2]
        b, s, c = decode_scale(tensor, anchors[3 * i:3 * i + 3], stride, score_thr, sigmoid)
        boxes_list.append(b)
        scores_list.append(s)
        cls_list.append(c)

    boxes = np.concatenate(boxes_list) if boxes_list else np.zeros((0, 4), dtype=np.float32)
    scores = np.concatenate(scores_list) if scores_list else np.zeros(0, dtype=np.float32)
    cls = np.concatenate(cls_list) if cls_list else np.zeros(0, dtype=np.int64)

    if boxes.size == 0:
        return []
    # xywh -> xyxy（参考 xywh2xyxy）
    xyxy = np.copy(boxes)
    xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    return classwise_nms(xyxy, scores, cls, nms_thr)


def map_categories(
    dets: Sequence[Detection],
    cat_map: Sequence[int],
    cat_names: dict[str, str],
) -> list[CategoryDetection]:
    """细类 -> 4 大类映射。"""
    out = []
    for d in dets:
        cat = cat_map[d.class_id]
        out.append(CategoryDetection.from_detection(d, cat, cat_names[str(cat)]))
    return out
