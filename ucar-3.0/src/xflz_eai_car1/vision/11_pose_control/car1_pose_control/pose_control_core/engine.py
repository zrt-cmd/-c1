"""姿态引擎（RKNN BlazePose 单模型整图）与骨架绘制。

管线：letterbox 256 → RGB [-1,1] → pose_landmark_full → 33×4 像素关键点
+ presence（vendor mp.solutions.pose 语义）。后端注入（backend.py 协议）可测。
"""
from __future__ import annotations

from typing import Callable

import cv2
import numpy as np

from .backend import Backend, RKNNLiteBackend
from .pose import (
    INPUT_SIZE,
    POSE_CONNECTIONS,
    PoseResult,
    decode_pose,
    prepare,
    resize_pad,
)

POINT_COLOR = (0, 0, 255)      # 红：关键点（vendor DrawingSpec）
LINE_COLOR = (0, 255, 0)       # 绿：骨架连线（vendor）
POINT_RADIUS = 6
LINE_THICKNESS = 2
VIS_THRESHOLD = 0.5            # vendor：可见度低于 0.5 不画点/线


class PoseEngine:
    """整图姿态引擎（单模型；create_fn 注入后端，x86 测试免 rknn）。"""

    def __init__(
        self,
        pose_model: str,
        min_detection_confidence: float = 0.5,
        create_fn: Callable | None = None,
        core_mask: int = 0,
        input_size: int = INPUT_SIZE,
    ):
        self.pose_model = pose_model
        self.min_detection_confidence = min_detection_confidence
        self.input_size = input_size
        self._create_fn = create_fn
        self._core_mask = core_mask
        self._backend: Backend | None = None

    def _ensure_created(self):
        if self._backend is not None:
            return
        if self._create_fn is not None:
            self._backend = self._create_fn()
            return
        backend = RKNNLiteBackend(core_mask=self._core_mask)
        backend.load(self.pose_model)
        self._backend = backend

    def process(self, frame_bgr: np.ndarray) -> PoseResult:
        """整图推理 → PoseResult（presence < 阈值 → 空结果，不崩）。"""
        self._ensure_created()
        h, w = frame_bgr.shape[:2]
        det_img, scale, pad = resize_pad(frame_bgr, self.input_size)
        outs = self._backend.predict(prepare(det_img, self.input_size))
        result = decode_pose(outs, scale, pad, (h, w))
        if result.score < self.min_detection_confidence:
            return PoseResult()
        return result

    def close(self):
        if self._backend is not None:
            self._backend.close()
            self._backend = None


def draw_skeleton(
    img: np.ndarray,
    result: PoseResult,
    connections: frozenset = POSE_CONNECTIONS,
    point_color=POINT_COLOR,
    line_color=LINE_COLOR,
    point_radius: int = POINT_RADIUS,
    line_thickness: int = LINE_THICKNESS,
    vis_threshold: float = VIS_THRESHOLD,
) -> np.ndarray:
    """在给定图上绘制 33 点骨架（vis < 阈值跳过）；返回副本。

    右栏黑底画布：调用方传 np.zeros_like(frame)。
    """
    out = img.copy()
    if not result.detected:
        return out
    lm = result.landmarks
    h, w = out.shape[:2]
    x = np.clip(lm[:, 0], 0, w - 1).astype(np.int32)
    y = np.clip(lm[:, 1], 0, h - 1).astype(np.int32)
    vis = lm[:, 3]
    for a, b in connections:
        if a < len(x) and b < len(x) and vis[a] >= vis_threshold and vis[b] >= vis_threshold:
            cv2.line(out, (int(x[a]), int(y[a])), (int(x[b]), int(y[b])),
                     line_color, line_thickness)
    for i, (px, py) in enumerate(zip(x, y)):
        if vis[i] >= vis_threshold:
            cv2.circle(out, (int(px), int(py)), point_radius, point_color, -1)
    return out
