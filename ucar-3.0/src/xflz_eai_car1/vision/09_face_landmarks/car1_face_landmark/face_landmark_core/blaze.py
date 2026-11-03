"""Blaze 两阶段检测算法（纯 numpy/cv2，无 rknn 依赖）。

算法移植自 mediapipe（tflite_tensors_to_detections / detections_to_rects /
rect 仿射变换），参数与 rknn-mediapipe-master 案例 C++/Python 实现一致
（Apache-2.0）：锚生成（SsdAnchors）、框/关键点解码、分数 sigmoid、
NMS、detection2roi（box 方法）、ROI 仿射 warp、landmark 逆映射。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class AnchorOptions:
    num_layers: int = 4
    min_scale: float = 0.1484375
    max_scale: float = 0.75
    input_size: int = 128
    anchor_offset_x: float = 0.5
    anchor_offset_y: float = 0.5
    strides: tuple = (8, 16, 16, 16)
    aspect_ratios: tuple = (1.0,)
    reduce_boxes_in_lowest_layer: bool = False
    interpolated_scale_aspect_ratio: float = 1.0
    fixed_anchor_size: bool = True


@dataclass
class DetectConfig:
    """face_detection_short_range v0.10 配置（案例 Config.cpp 核实）。"""
    num_anchors: int = 896
    num_coords: int = 16
    score_clipping_thresh: float = 100.0
    x_scale: float = 128.0
    y_scale: float = 128.0
    h_scale: float = 128.0
    w_scale: float = 128.0
    min_score_thresh: float = 0.5
    min_suppression_threshold: float = 0.3
    num_keypoints: int = 6
    detection2roi_method: str = "box"
    kp1: int = 1
    kp2: int = 0
    theta0: float = 0.0
    dscale: float = 1.5
    dy: float = 0.0


@dataclass
class Detection:
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    score: float
    keypoints: list = field(default_factory=list)  # [(x, y), ...]


@dataclass
class ROI:
    xc: float
    yc: float
    scale: float
    theta: float = 0.0


def generate_anchors(options: AnchorOptions) -> np.ndarray:
    """SsdAnchors 锚生成，返回 (N,4) 归一化 [xc, yc, w, h]（fixed → w=h=1）。"""
    anchors = []
    layer_id = 0
    n = len(options.strides)
    while layer_id < n:
        aspect_ratios, scales = [], []
        last_same = layer_id
        while last_same < n and options.strides[last_same] == options.strides[layer_id]:
            scale = _calc_scale(options, last_same, n)
            if last_same == 0 and options.reduce_boxes_in_lowest_layer:
                aspect_ratios.extend([1.0, 2.0, 0.5])
                scales.extend([0.1, scale, scale])
            else:
                for ar in options.aspect_ratios:
                    aspect_ratios.append(ar)
                    scales.append(scale)
                if options.interpolated_scale_aspect_ratio > 0.0:
                    scale_next = (1.0 if last_same == n - 1
                                  else _calc_scale(options, last_same + 1, n))
                    scales.append(np.sqrt(scale * scale_next))
                    aspect_ratios.append(options.interpolated_scale_aspect_ratio)
            last_same += 1

        heights = [sc / np.sqrt(ar) for ar, sc in zip(aspect_ratios, scales)]
        widths = [sc * np.sqrt(ar) for ar, sc in zip(aspect_ratios, scales)]
        stride = options.strides[layer_id]
        fh = int(np.ceil(options.input_size / stride))
        fw = int(np.ceil(options.input_size / stride))
        for y in range(fh):
            for x in range(fw):
                xc = (x + options.anchor_offset_x) / fw
                yc = (y + options.anchor_offset_y) / fh
                for aw, ah in zip(widths, heights):
                    anchors.append((xc, yc,
                                    1.0 if options.fixed_anchor_size else aw,
                                    1.0 if options.fixed_anchor_size else ah))
        layer_id = last_same
    return np.asarray(anchors, dtype=np.float32).reshape(-1, 4)


def _calc_scale(options: AnchorOptions, index: int, num_strides: int) -> float:
    if num_strides == 1:
        return (options.max_scale + options.min_scale) * 0.5
    return options.min_scale + (options.max_scale - options.min_scale) * index / (num_strides - 1.0)


def decode_boxes(raw_boxes: np.ndarray, anchors: np.ndarray, cfg: DetectConfig) -> np.ndarray:
    """框 + 关键点解码（raw (N,16) → (N,4) ymin,xmin,ymax,xmax，输入空间归一化）。"""
    n = raw_boxes.shape[0]
    boxes = np.zeros((n, 4), dtype=np.float32)
    boxes[:, 0] = raw_boxes[:, 1] / cfg.y_scale * anchors[:, 3] + anchors[:, 1]  # yc
    boxes[:, 1] = raw_boxes[:, 0] / cfg.x_scale * anchors[:, 2] + anchors[:, 0]  # xc
    h = raw_boxes[:, 3] / cfg.h_scale * anchors[:, 3]
    w = raw_boxes[:, 2] / cfg.w_scale * anchors[:, 2]
    ymin = boxes[:, 0] - h / 2.0
    xmin = boxes[:, 1] - w / 2.0
    ymax = boxes[:, 0] + h / 2.0
    xmax = boxes[:, 1] + w / 2.0
    return np.stack([ymin, xmin, ymax, xmax], axis=1)


def decode_keypoints(raw_boxes: np.ndarray, anchors: np.ndarray, cfg: DetectConfig) -> np.ndarray:
    """关键点解码 (N,16) → (N, num_keypoints, 2) 归一化（输入空间）。"""
    n = raw_boxes.shape[0]
    kps = np.zeros((n, cfg.num_keypoints, 2), dtype=np.float32)
    for k in range(cfg.num_keypoints):
        off = 4 + k * 2
        kps[:, k, 0] = raw_boxes[:, off] / cfg.x_scale * anchors[:, 2] + anchors[:, 0]
        kps[:, k, 1] = raw_boxes[:, off + 1] / cfg.y_scale * anchors[:, 3] + anchors[:, 1]
    return kps


def tensors_to_detections(raw_boxes: np.ndarray, raw_scores: np.ndarray,
                          cfg: DetectConfig, anchors: np.ndarray | None = None) -> list[Detection]:
    if anchors is None:
        anchors = generate_anchors(AnchorOptions(input_size=int(cfg.x_scale)))
    scores = np.clip(raw_scores, -cfg.score_clipping_thresh, cfg.score_clipping_thresh)
    x = np.clip(scores.reshape(-1), -88.0, 88.0)  # exp 不溢出范围
    # 稳定 sigmoid
    scores = np.where(x >= 0, 1.0 / (1.0 + np.exp(-x)),
                      np.exp(x) / (1.0 + np.exp(x)))
    boxes = decode_boxes(raw_boxes, anchors, cfg)
    kps = decode_keypoints(raw_boxes, anchors, cfg)
    dets = []
    for i in range(raw_boxes.shape[0]):
        if scores[i] < cfg.min_score_thresh:
            continue
        dets.append(Detection(
            xmin=float(boxes[i, 1]), ymin=float(boxes[i, 0]),
            xmax=float(boxes[i, 3]), ymax=float(boxes[i, 2]),
            score=float(scores[i]),
            keypoints=[(float(kps[i, k, 0]), float(kps[i, k, 1]))
                       for k in range(cfg.num_keypoints)],
        ))
    return dets


def nms(dets: list[Detection], iou_thresh: float = 0.3) -> list[Detection]:
    """按分数降序贪心 NMS，返回过滤后列表。"""
    order = sorted(range(len(dets)), key=lambda i: dets[i].score, reverse=True)
    keep = []
    while order:
        i = order.pop(0)
        keep.append(dets[i])
        order = [j for j in order
                 if _iou(dets[i], dets[j]) <= iou_thresh]
    return keep


def _iou(a: Detection, b: Detection) -> float:
    ix1 = max(a.xmin, b.xmin)
    iy1 = max(a.ymin, b.ymin)
    ix2 = min(a.xmax, b.xmax)
    iy2 = min(a.ymax, b.ymax)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, a.xmax - a.xmin) * max(0.0, a.ymax - a.ymin)
    area_b = max(0.0, b.xmax - b.xmin) * max(0.0, b.ymax - b.ymin)
    return inter / max(area_a + area_b - inter, 1e-9)


def resize_pad(img: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
    """letterbox 缩放（保持纵横比 + 灰边），返回 (padded, scale, (pad_x, pad_y))。"""
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded = np.full((size, size, 3), 0, dtype=np.uint8)
    pad_x = (size - nw) // 2
    pad_y = (size - nh) // 2
    padded[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return padded, scale, (pad_x, pad_y)


def denormalize_detections(dets: list[Detection], scale: float,
                           pad: tuple[int, int]) -> list[Detection]:
    """输入空间归一化 → 原图像素。

    原图 → letterbox（×scale，pad）→ 输入图（归一化 = 像素/128）。
    逆：px = (norm × 128 − pad) / scale。
    """
    pad_x, pad_y = pad
    out = []
    for d in dets:
        out.append(Detection(
            xmin=(d.xmin * 128.0 - pad_x) / scale,
            ymin=(d.ymin * 128.0 - pad_y) / scale,
            xmax=(d.xmax * 128.0 - pad_x) / scale,
            ymax=(d.ymax * 128.0 - pad_y) / scale,
            score=d.score,
            keypoints=[((kx * 128.0 - pad_x) / scale,
                        (ky * 128.0 - pad_y) / scale)
                       for kx, ky in d.keypoints],
        ))
    return out


def detection2roi(dets: list[Detection], cfg: DetectConfig) -> list[ROI]:
    """detections_to_rects（box 方法）：中心/尺度/旋转。"""
    rois = []
    for d in dets:
        if cfg.detection2roi_method == "box":
            xc = (d.xmin + d.xmax) / 2.0
            yc = (d.ymin + d.ymax) / 2.0
            scale = d.xmax - d.xmin  # 假设方形框
        elif cfg.detection2roi_method == "alignment" and len(d.keypoints) > max(cfg.kp1, cfg.kp2):
            xc, yc = d.keypoints[cfg.kp1]
            x1, y1 = d.keypoints[cfg.kp2]
            scale = np.hypot(xc - x1, yc - y1) * 2.0
        else:
            xc = (d.xmin + d.xmax) / 2.0
            yc = (d.ymin + d.ymax) / 2.0
            scale = d.xmax - d.xmin
        yc += cfg.dy * scale
        scale *= cfg.dscale
        theta = 0.0
        if len(d.keypoints) > max(cfg.kp1, cfg.kp2):
            x0, y0 = d.keypoints[cfg.kp1]
            x1, y1 = d.keypoints[cfg.kp2]
            theta = np.arctan2(y0 - y1, x0 - x1) - cfg.theta0
        rois.append(ROI(xc=float(xc), yc=float(yc), scale=float(scale), theta=float(theta)))
    return rois


def roi_affine(roi: ROI, target_size: int) -> np.ndarray:
    """ROI → 仿射矩阵（2x3）：中心平移 → 旋转 -θ → 缩放 target/scale。"""
    theta = -roi.theta
    cos, sin = np.cos(theta), np.sin(theta)
    # 源点：ROI 四角（正方形边=scale，中心 xc,yc，旋转 θ）
    half = roi.scale / 2.0
    corners = np.array([[-half, -half], [half, -half], [-half, half]], dtype=np.float32)
    rot = np.array([[cos, -sin], [sin, cos]], dtype=np.float32)
    src = corners @ rot.T + np.array([[roi.xc, roi.yc]], dtype=np.float32)
    dst = np.array([[0, 0], [target_size, 0], [0, target_size]], dtype=np.float32)
    return cv2.getAffineTransform(src, dst)


def warp_roi(img: np.ndarray, m: np.ndarray, target_size: int) -> np.ndarray:
    return cv2.warpAffine(img, m, (target_size, target_size),
                          flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def denormalize_landmarks(landmarks: np.ndarray, m: np.ndarray,
                          target_size: int, img_h: int, img_w: int) -> np.ndarray:
    """ROI 关键点（模型输出 = ROI 像素坐标 0..target_size）→ 原图归一化 (N,3)。

    face_landmark 模型输出 x,y 是 192×192 ROI 的**像素坐标**（mediapipe-rknn
    face_mesh.py 核实：直接 reshape 后按 letterbox 逆映射，不乘/不除输入尺寸），
    不是归一化 [0,1] —— 此处不再乘 target_size，直接逆仿射回原图像素再归一化；
    z 与 x/y 同尺度，按 target_size 归一化（相对深度，mediapipe 语义）。
    """
    pts = landmarks[:, :2]  # ROI 图像素
    inv = np.vstack([m, np.array([[0.0, 0.0, 1.0]], dtype=np.float32)])
    inv = np.linalg.inv(inv)[:2]
    px = pts @ inv[:, :2].T + inv[:, 2]
    out = np.zeros_like(landmarks)
    out[:, 0] = np.clip(px[:, 0] / max(img_w, 1), 0.0, 1.0)
    out[:, 1] = np.clip(px[:, 1] / max(img_h, 1), 0.0, 1.0)
    out[:, 2] = landmarks[:, 2] / float(target_size)
    return out
