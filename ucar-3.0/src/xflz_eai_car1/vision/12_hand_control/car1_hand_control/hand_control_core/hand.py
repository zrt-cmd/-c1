"""手部几何：HAND_CONNECTIONS、手掌检测解码（blazeface 192）、ROI 合并、21 点解码。

模型契约（mediapipe-rknn/models/gesture_recognizer/convert.py MODELS 表，板端实测）：
  hand_detector.rknn            —— 192×192，mean 0 / std 1（float32 [0,1] RGB）
  hand_landmarks_detector.rknn  —— 224×224，mean 0 / std 1（float32 [0,1] RGB）
  gesture_embedder.rknn         —— 无 mean/std（float 输入：norm_tensor 归一化 21 点）
  canned_gesture_classifier.rknn—— 无 mean/std（float 输入：128 维 embedding）

管线：手掌检测（letterbox 192 float/255）→ NMS → union_bbox 合并 →
关键点（letterbox 224 float/255）→ 21 点 → embedder → 分类器。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from itertools import product
from math import ceil

import cv2
import numpy as np

PALM_SIZE = 192
HAND_SIZE = 224

# 官方 HAND_CONNECTIONS（mediapipe 21 点拓扑，Apache-2.0）
HAND_CONNECTIONS: frozenset = frozenset([
    (0, 1), (1, 2), (2, 3), (3, 4),          # 拇指
    (0, 5), (5, 6), (6, 7), (7, 8),          # 食指
    (5, 9), (9, 10), (10, 11), (11, 12),     # 中指
    (9, 13), (13, 14), (14, 15), (15, 16),   # 无名指
    (13, 17), (17, 18), (18, 19), (19, 20),  # 小指
    (0, 17),                                 # 掌
])


@dataclass
class Hand:
    """单只手：landmarks = 21×3 原图像素 [x,y,z] + 手掌框 + 手势结果。"""
    landmarks: np.ndarray
    score: float
    handedness: float
    box: tuple[int, int, int, int] | None = None   # 手掌框原图像素 (x1,y1,x2,y2)
    world: np.ndarray = field(default_factory=lambda: np.zeros((21, 3), dtype=np.float32))
    gesture_label: str = "None"
    gesture_score: float = 0.0

    @property
    def is_right(self) -> bool:
        return self.handedness > 0.5


@dataclass
class HandResult:
    """一帧的手部结果（0~2 只手）。"""
    hands: list[Hand] = field(default_factory=list)

    @property
    def num_hands(self) -> int:
        return len(self.hands)


def resize_pad(img: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
    """letterbox 缩放（保持纵横比 + pad 0），返回 (padded, scale, (pad_x, pad_y))。"""
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded = np.full((size, size, 3), 0, dtype=np.uint8)
    pad_x = (size - nw) // 2
    pad_y = (size - nh) // 2
    padded[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return padded, scale, (pad_x, pad_y)


def prepare(img: np.ndarray, size: int) -> np.ndarray:
    """BGR → RGB float32 [0,1] NHWC（mean 0/std 1 转换契约，板端实测 2026-08-07）。

    用户差分实验 + 板端验证：palm/hand 模型输入层为 float32，转换配 mean 0/std 1
    （127.5/128 会把 [0,1] 输入打成 [-1,1] → reg 回归头爆炸 ±5000/NaN；
    uint8 直接喂同样爆炸）。正确输入 = float32 像素 /255。
    """
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return (rgb.astype(np.float32) / 255.0).reshape(1, size, size, 3)


def palm_anchors(input_size: int = PALM_SIZE) -> np.ndarray:
    """手掌检测 2016 锚（192）：spec [(8,2),(16,2),(16,2),(16,2)]。"""
    spec = [(8, 2), (16, 2), (16, 2), (16, 2)]
    anchors = []
    for stride, per_loc in spec:
        feat = ceil(input_size / stride)
        for y, x in product(range(feat), range(feat)):
            for _ in range(per_loc):
                anchors.append([(x + 0.5) / feat, (y + 0.5) / feat,
                                1.0 / feat, 1.0 / feat])
    return np.asarray(anchors, dtype=np.float32)


def decode_palm(reg: np.ndarray, cls: np.ndarray, anchors: np.ndarray,
                input_size: int = PALM_SIZE,
                score_thresh: float = 0.5, nms_thresh: float = 0.3) -> list[dict]:
    """手掌检测输出 → 检测列表（输入空间像素）：bbox [x1,y1,x2,y2] + 7 keypoints + score。"""
    reg = np.asarray(reg).reshape(-1, 18)
    s_raw = np.asarray(cls).reshape(-1)[: reg.shape[0]]
    s = np.clip(s_raw, -88.0, 88.0)
    probs = np.where(s >= 0, 1.0 / (1.0 + np.exp(-s)),
                     np.exp(s) / (1.0 + np.exp(s)))
    mask = probs > score_thresh
    if not np.any(mask):
        return []
    r, a, p = reg[mask], anchors[mask], probs[mask]

    cx = a[:, 0] + r[:, 0] / input_size
    cy = a[:, 1] + r[:, 1] / input_size
    w = r[:, 2] / input_size
    h = r[:, 3] / input_size
    x1 = (cx - w / 2) * input_size
    y1 = (cy - h / 2) * input_size
    x2 = (cx + w / 2) * input_size
    y2 = (cy + h / 2) * input_size
    kps = [np.stack([(a[:, 0] + r[:, 4 + i * 2] / input_size) * input_size,
                     (a[:, 1] + r[:, 4 + i * 2 + 1] / input_size) * input_size],
                    axis=-1) for i in range(7)]

    keep = _nms(np.stack([x1, y1, x2, y2], axis=-1), p, nms_thresh)
    return [{"bbox": [float(x1[k]), float(y1[k]), float(x2[k]), float(y2[k])],
             "keypoints": [kps[i][k].tolist() for i in range(7)],
             "score": float(p[k])} for k in keep]


def _nms(boxes: np.ndarray, scores: np.ndarray, thresh: float) -> list[int]:
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        rest = order[1:]
        if rest.size == 0:
            break
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (areas[i] + areas[rest] - inter + 1e-6)
        order = rest[iou <= thresh]
    return keep


def scale_detections(dets: list[dict], scale: float, pad: tuple[int, int],
                     orig_shape: tuple[int, int]) -> list[dict]:
    """手掌检测：letterbox 输入空间 → 原图像素（裁剪到帧内）。"""
    pad_w, pad_h = pad
    oh, ow = orig_shape[:2]
    out = []
    for d in dets:
        x1, y1, x2, y2 = d["bbox"]
        out.append({
            "bbox": [max(0.0, (x1 - pad_w) / scale), max(0.0, (y1 - pad_h) / scale),
                     min(float(ow), (x2 - pad_w) / scale), min(float(oh), (y2 - pad_h) / scale)],
            "keypoints": [((kx - pad_w) / scale, (ky - pad_h) / scale)
                          for kx, ky in d["keypoints"]],
            "score": d["score"],
        })
    return out



def union_bbox(dets: list[dict], frame_shape: tuple[int, int]) -> tuple[int, int, int, int]:
    """合并全部检测框 → 单一 ROI（裁剪到帧内）。

    板端实测 2026-08-07：hand_demo/ye 特写大图（手占画面 80%+）时 palm 检测器
    把一只手分裂成 4 个重叠小框（NMS 不合并）——逐框 crop 只能看到手掌局部，
    presence 低、手势分类错；union 后 ROI 覆盖整只手 → presence 0.99+、
    Open_Palm/Victory 分类正确。正常尺度单手（1 框）退化为单框，语义不变。
    返回 (x1, y1, x2, y2) 原图像素。
    """
    if not dets:
        return (0, 0, 0, 0)
    oh, ow = frame_shape[:2]
    x1 = max(0, int(min(d["bbox"][0] for d in dets)))
    y1 = max(0, int(min(d["bbox"][1] for d in dets)))
    x2 = min(ow, int(max(d["bbox"][2] for d in dets)))
    y2 = min(oh, int(max(d["bbox"][3] for d in dets)))
    return (x1, y1, x2, y2)


def detection2roi(landmarks: np.ndarray, kp1: int = 0, kp2: int = 9,
                  dscale: float = 2.6, dy: float = -0.5,
                  theta0: float = 0.0) -> tuple[float, float, float, float]:
    """21 点 → 旋转方形 ROI 参数（mediapipe detection2roi，官方语义）。

    alignment 模式（跟踪）：中心 = kp1（腕 0），边长 = 2×dist(kp1, kp2)（中指根 9），
    中心沿 y 偏移 dy×scale，边长 ×dscale，旋转角 = atan2(kp2-kp1) - theta0。
    板端实测 2026-08-07：theta 用 kp2-kp1（手方向沿 ROI 高度轴，垂直手正立）；
    反向（kp1-kp2）会把垂直手旋成水平（手型 90° 翻转，分类全 None）。
    返回 (xc, yc, scale, theta)。
    """
    p1, p2 = landmarks[kp1], landmarks[kp2]
    dx, d_y = float(p2[0] - p1[0]), float(p2[1] - p1[1])
    length = float(np.hypot(dx, d_y))
    theta = float(np.arctan2(d_y, dx)) - theta0
    xc, yc = float(p1[0]), float(p1[1])
    scale = length * 2.0
    yc += dy * scale
    scale *= dscale
    return xc, yc, scale, theta


def detection_box2roi(bbox: tuple, keypoints: list | None = None,
                      kp1: int = 0, kp2: int = 2,
                      dscale: float = 2.6, dy: float = -0.5,
                      theta0: float = float(np.pi / 2)) -> tuple[float, float, float, float]:
    """palm 检测框 → 旋转方形 ROI（mediapipe detection2roi box 模式）。

    中心 = 框中心，边长 = 框宽；旋转角来自检测器关键点 kp1/kp2（palm 7 点中
    腕 0 与食指根 2，官方 palm v0.10 配置 theta0=π/2）。
    返回 (xc, yc, scale, theta)。
    """
    x1, y1, x2, y2 = bbox
    xc, yc = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    scale = float(x2 - x1)
    yc += dy * scale
    scale *= dscale
    theta = 0.0
    if keypoints is not None and len(keypoints) > max(kp1, kp2):
        x0, y0 = keypoints[kp1]
        x1_, y1_ = keypoints[kp2]
        theta = float(np.arctan2(y0 - y1_, x0 - x1_)) - theta0
    return xc, yc, scale, theta


def roi_affine(xc: float, yc: float, scale: float, theta: float,
               resolution: int = 224) -> tuple[np.ndarray, np.ndarray]:
    """旋转方形 ROI → 仿射矩阵 M（ROI→224²）与逆矩阵 M_inv（224²→原图）。

    mediapipe 官方（LandmarkBase::extract_roi）：单位方形模板点 (-1,-1)(-1,1)(1,-1)
    ×scale/2 → 旋转 theta + 平移 → 映射到 (0,0)(0,H-1)(W-1,0)。
    """
    c, s = np.cos(theta), np.sin(theta)
    half = scale / 2.0
    src = []
    for px, py in ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0)):
        src.append([c * (px * half) - s * (py * half) + xc,
                    s * (px * half) + c * (py * half) + yc])
    dst = [[0.0, 0.0], [0.0, resolution - 1.0], [resolution - 1.0, 0.0]]
    m = cv2.getAffineTransform(np.asarray(src, np.float32),
                               np.asarray(dst, np.float32))
    return m, cv2.invertAffineTransform(m)


def warp_roi(frame: np.ndarray, m: np.ndarray,
             resolution: int = 224) -> np.ndarray:
    """仿射裁剪：旋转方形 ROI → resolution² 图像。"""
    return cv2.warpAffine(frame, m, (resolution, resolution))


def unproject_landmarks(joints_224: np.ndarray, m_inv: np.ndarray,
                        resolution: int = 224) -> np.ndarray:
    """224 空间关键点 → 原图像素（逆仿射，mediapipe denormalize_landmarks）。

    joints_224: (N,3) 模型输出像素（[x,y,z]）；z 通道直通（不参与仿射）。
    """
    pts = joints_224[:, :2].astype(np.float64)
    ones = np.ones((len(pts), 1))
    out = pts @ m_inv[:, :2].T + m_inv[:, 2]
    res = np.empty_like(joints_224, dtype=np.float32)
    res[:, :2] = out
    res[:, 2] = joints_224[:, 2]
    return res


def crop_hand(img: np.ndarray, bbox: list[float], pad_ratio: float = 0.2,
              ) -> tuple[np.ndarray, tuple[int, int]]:
    """裁剪手部 ROI（各边 20% padding，test_board.py 语义）；返回 (roi, (ox, oy))。"""
    x1, y1, x2, y2 = [int(v) for v in bbox]
    h, w = img.shape[:2]
    bw, bh = x2 - x1, y2 - y1
    pad_x = int(bw * pad_ratio)
    pad_y = int(bh * pad_ratio)
    x1 = max(0, x1 - pad_x)
    y1 = max(0, y1 - pad_y)
    x2 = min(w, x2 + pad_x)
    y2 = min(h, y2 + pad_y)
    return img[y1:y2, x1:x2], (x1, y1)


def decode_hand(outputs: list[np.ndarray], scale: float = 1.0,
                pad: tuple[int, int] = (0, 0),
                orig_shape: tuple[int, int] | None = None) -> Hand:
    """关键点输出 → Hand（landmarks/world 为 ROI 空间，未加 ROI 偏移）。

    outputs: [21×3 像素, presence, handedness, world 21×3]（test_board 语义）。
    板端实测 2026-08-07（用户重转模型）：输出布局 **[x, y, z]** 标准顺序，
    无需交换（腕下、五指朝上拓扑正常；旧 [y,x,z] 假设作废）。
    """
    joints = np.asarray(outputs[0]).reshape(21, 3).astype(np.float32)
    score = float(np.asarray(outputs[1]).reshape(-1)[0])
    handedness = float(np.asarray(outputs[2]).reshape(-1)[0])
    world = np.asarray(outputs[3]).reshape(21, 3).astype(np.float32)
    if orig_shape is not None:
        pad_w, pad_h = pad
        joints[:, 0] = (joints[:, 0] - pad_w) / scale
        joints[:, 1] = (joints[:, 1] - pad_h) / scale
        joints[:, 2] = joints[:, 2] / scale
    return Hand(landmarks=joints, score=score, handedness=handedness, world=world)
