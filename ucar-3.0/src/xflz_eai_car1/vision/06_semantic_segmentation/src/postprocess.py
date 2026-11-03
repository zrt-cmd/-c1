"""分割解码：13 输出 -> 检测 + 掩码（构建文档 03）。

纯 numpy；解码语义逐行对照 rknn_model_zoo yolov8_seg.py
（post_process / filter_boxes / box_process / dfl）。
输出布局（fork Segment rknn 导出）：每层 [box(64), cls(80), cls_sum(1), mc(32)]x3 + proto(32)。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import cv2

DFL_BINS = 16
NM = 32  # mask 系数维度


@dataclass(frozen=True)
class SegResult:
    boxes: np.ndarray   # [N,4] float32 xyxy（imgsz 系）
    classes: np.ndarray  # [N] int32
    scores: np.ndarray   # [N] float32
    masks: np.ndarray    # [N, imgsz, imgsz] bool


def dfl_decode(position: np.ndarray) -> np.ndarray:
    """(n,64,h,w) -> (n,4,h,w)：16 bins softmax 的期望值。"""
    n, c, h, w = position.shape
    x = position.reshape(n, 4, c // 4, h, w)
    x = x - x.max(axis=2, keepdims=True)          # 数值稳定
    exp = np.exp(x)
    probs = exp / exp.sum(axis=2, keepdims=True)
    bins = np.arange(c // 4, dtype=np.float32).reshape(1, 1, c // 4, 1, 1)
    return (probs * bins).sum(axis=2)


def box_process(position: np.ndarray, imgsz: int) -> np.ndarray:
    """(1,64,h,w) -> (1,4,h,w) xyxy（imgsz 系）。"""
    n, _, gh, gw = position.shape
    col = np.arange(gw, dtype=np.float32).reshape(1, 1, 1, gw)
    row = np.arange(gh, dtype=np.float32).reshape(1, 1, gh, 1)
    grid = np.concatenate(np.broadcast_arrays(col, np.broadcast_to(row, (1, 1, gh, gw))), axis=1)
    stride = np.array([imgsz // gh, imgsz // gw], dtype=np.float32).reshape(1, 2, 1, 1)
    dist = dfl_decode(position)
    xy1 = grid + 0.5 - dist[:, 0:2]
    xy2 = grid + 0.5 + dist[:, 2:4]
    return np.concatenate((xy1 * stride, xy2 * stride), axis=1)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thr: float,
        class_ids: np.ndarray | None = None) -> np.ndarray:
    """逐类 NMS（max_wh 偏移），返回保留索引。"""
    boxes = boxes.astype(np.float32)
    if class_ids is not None:
        boxes = boxes + (class_ids * 7680)[:, None].astype(np.float32)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_r = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
        iou = inter / np.maximum(area_i + area_r - inter, 1e-9)
        order = rest[iou <= iou_thr]
    return np.array(keep, dtype=np.int64)


def _flatten(x: np.ndarray) -> np.ndarray:
    """(1,c,h,w) -> (h*w, c)。"""
    return x[0].transpose(1, 2, 0).reshape(-1, x.shape[1])


def decode_outputs(outputs: list[np.ndarray], cfg: dict, imgsz: int = 640) -> SegResult | None:
    """13 输出 -> SegResult（过滤 + NMS + 掩码解码）。"""
    score_thr = float(cfg["model"]["score_thr"])
    iou_thr = float(cfg["model"]["iou_thr"])
    mask_thr = float(cfg["model"]["mask_thr"])
    proto = outputs[-1][0]  # (32, 160, 160)

    boxes_l, cls_l, sum_l, mc_l = [], [], [], []
    pair = len(outputs) // 3  # 4
    for i in range(3):
        boxes_l.append(_flatten(box_process(outputs[pair * i], imgsz)))
        cls_l.append(_flatten(outputs[pair * i + 1]))
        sum_l.append(_flatten(outputs[pair * i + 2]))
        mc_l.append(_flatten(outputs[pair * i + 3]))
    boxes = np.concatenate(boxes_l).astype(np.float32)
    class_probs = np.concatenate(cls_l)
    cls_sum = np.concatenate(sum_l)[:, 0]
    seg_part = np.concatenate(mc_l).astype(np.float32)

    class_max = class_probs.max(axis=-1)
    classes = class_probs.argmax(axis=-1)
    pos = np.where(class_max * cls_sum >= score_thr)[0]
    if pos.size == 0:
        return None
    scores = (class_max * cls_sum)[pos]
    boxes = boxes[pos]
    classes = classes[pos]
    seg_part = (seg_part * cls_sum[:, None])[pos]

    keep = nms(boxes, scores, iou_thr, class_ids=classes.astype(np.int32))
    boxes = boxes[keep]
    classes = classes[keep].astype(np.int32)
    scores = scores[keep]
    seg_part = seg_part[keep]

    ph, pw = proto.shape[-2:]
    proto_m = proto.reshape(seg_part.shape[-1], -1)  # (32, 160*160)
    seg_img = seg_part @ proto_m                      # (N, 160*160)
    seg_img = 1.0 / (1.0 + np.exp(-seg_img))          # sigmoid
    seg_img = seg_img.reshape(-1, ph, pw)
    seg_img = np.stack([cv2.resize(m, (imgsz, imgsz), interpolation=cv2.INTER_LINEAR)
                        for m in seg_img])
    # 框外掩码清零（_crop_mask 语义）
    for k in range(len(boxes)):
        x1, y1, x2, y2 = np.clip(boxes[k], 0, imgsz - 1).astype(int)
        m = np.zeros((imgsz, imgsz), dtype=np.uint8)
        m[y1:y2 + 1, x1:x2 + 1] = 1
        seg_img[k] *= m
    return SegResult(boxes=boxes, classes=classes, scores=scores,
                     masks=seg_img > mask_thr)


def foreground_mask(result: SegResult | None, keep_classes: list[int] | None) -> np.ndarray:
    """保留类掩码并集（空/None = 全部）。返回 (imgsz, imgsz) bool。"""
    if result is None or len(result.classes) == 0:
        return np.zeros(result.masks.shape[1:], dtype=bool) if result else None
    sel = np.ones(len(result.classes), dtype=bool) if not keep_classes else \
        np.isin(result.classes, np.asarray(keep_classes))
    if not sel.any():
        return np.zeros(result.masks.shape[1:], dtype=bool)
    return result.masks[sel].any(axis=0)
