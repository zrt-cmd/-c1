"""YOLOv8 检测解码（9 输出）→ Detection 列表（构建文档 02/03）。

对照 vendor Deepsort/YOlOv8/func.py（与 exp06 postprocess 同构、nc 通用）。
输出布局（fork rknn 导出）：每层 [box(64), cls(80), cls_sum(1)]×3。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import cv2


@dataclass(frozen=True)
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int


def dfl_decode(position: np.ndarray) -> np.ndarray:
    """(n,64,h,w) -> (n,4,h,w)：16 bins softmax 期望值。"""
    n, c, h, w = position.shape
    x = position.reshape(n, 4, c // 4, h, w)
    x = x - x.max(axis=2, keepdims=True)
    exp = np.exp(x)
    probs = exp / exp.sum(axis=2, keepdims=True)
    bins = np.arange(c // 4, dtype=np.float32).reshape(1, 1, c // 4, 1, 1)
    return (probs * bins).sum(axis=2)


def box_process(position: np.ndarray, imgsz: int) -> np.ndarray:
    """(1,64,h,w) -> (1,4,h,w) xyxy（imgsz 系）。"""
    _, _, gh, gw = position.shape
    col = np.arange(gw, dtype=np.float32).reshape(1, 1, 1, gw)
    row = np.arange(gh, dtype=np.float32).reshape(1, 1, gh, 1)
    grid = np.concatenate((np.broadcast_to(col, (1, 1, gh, gw)),
                           np.broadcast_to(row, (1, 1, gh, gw))), axis=1)
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
    return x[0].transpose(1, 2, 0).reshape(-1, x.shape[1])


def decode_outputs(outputs: list[np.ndarray], score_thr: float, iou_thr: float,
                   imgsz: int = 640) -> list[Detection]:
    """9 输出 -> Detection 列表（xyxy 为 imgsz 模型系）。"""
    boxes_l, cls_l, sum_l = [], [], []
    pair = len(outputs) // 3  # 3
    for i in range(3):
        boxes_l.append(_flatten(box_process(outputs[pair * i], imgsz)))
        cls_l.append(_flatten(outputs[pair * i + 1]))
        sum_l.append(_flatten(outputs[pair * i + 2]))
    boxes = np.concatenate(boxes_l).astype(np.float32)
    class_probs = np.concatenate(cls_l)
    cls_sum = np.concatenate(sum_l)[:, 0]

    class_max = class_probs.max(axis=-1)
    classes = class_probs.argmax(axis=-1)
    pos = np.where(class_max * cls_sum >= score_thr)[0]
    if pos.size == 0:
        return []
    scores = (class_max * cls_sum)[pos]
    boxes = boxes[pos]
    classes = classes[pos].astype(np.int32)

    keep = nms(boxes, scores, iou_thr, class_ids=classes)
    return [
        Detection(float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                  float(s), int(c))
        for b, s, c in zip(boxes[keep], scores[keep], classes[keep])
    ]


def decode_outputs_prefilter(outputs: list[np.ndarray], score_thr: float,
                             iou_thr: float, imgsz: int = 640) -> list[Detection]:
    """decode_outputs 的性能版：先按 class_max*cls_sum 预筛高分锚点，只解码这些。

    与 decode_outputs 逐帧一致（板端 40 帧实测全等）：低分锚点本就被
    score_thr 过滤，跳过 DFL 解码不改变 NMS 输入集合。逐点解码复用
    dfl_decode + 显式 grid/stride，语义与 box_process 一致。
    """
    pair = len(outputs) // 3
    boxes_l, scores_l, classes_l = [], [], []
    for i in range(3):
        box = outputs[pair * i]
        cls = outputs[pair * i + 1]
        sm = outputs[pair * i + 2]
        grid_h, grid_w = cls.shape[2], cls.shape[3]
        flat_cls = cls[0].transpose(1, 2, 0).reshape(-1, cls.shape[1])
        flat_sum = sm[0].reshape(-1)
        class_max = flat_cls.max(axis=1)
        classes = flat_cls.argmax(axis=1)
        pos = np.where(class_max * flat_sum >= score_thr)[0]
        if pos.size == 0:
            continue
        rows, cols = np.unravel_index(pos, (grid_h, grid_w))
        stride = np.array([imgsz / grid_h, imgsz / grid_w], dtype=np.float32)
        xy = np.zeros((pos.size, 4), dtype=np.float32)
        for k, (r, c) in enumerate(zip(rows.tolist(), cols.tolist())):
            cell = box[0, :, r, c]                       # (64,)
            d = dfl_decode(cell[None, :, None, None])    # (1,4,1,1)
            xy[k] = (
                (c + 0.5 - d[0, 0, 0, 0]) * stride[1],
                (r + 0.5 - d[0, 1, 0, 0]) * stride[0],
                (c + 0.5 + d[0, 2, 0, 0]) * stride[1],
                (r + 0.5 + d[0, 3, 0, 0]) * stride[0],
            )
        boxes_l.append(xy)
        scores_l.append((class_max * flat_sum)[pos])
        classes_l.append(classes[pos])
    if not boxes_l:
        return []
    boxes = np.concatenate(boxes_l).astype(np.float32)
    scores = np.concatenate(scores_l)
    classes = np.concatenate(classes_l).astype(np.int32)
    keep = nms(boxes, scores, iou_thr, class_ids=classes)
    return [
        Detection(float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                  float(s), int(c))
        for b, s, c in zip(boxes[keep], scores[keep], classes[keep])
    ]


def draw_detections(img: np.ndarray, dets: list[Detection],
                    class_names: tuple[str, ...] | None = None) -> np.ndarray:
    """绘制检测框 + 类别/分数。"""
    out = img.copy()
    for d in dets:
        name = class_names[d.class_id] if class_names else str(d.class_id)
        cv2.rectangle(out, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)),
                      (0, 255, 0), 2)
        cv2.putText(out, f"{name} {d.score:.2f}", (int(d.x1), max(16, int(d.y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return out
