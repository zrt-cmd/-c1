"""人脸关键点引擎（RKNN Blaze 双模型：检测 + 468 点）与网格绘制。

两阶段管线（mediapipe 语义，算法见 blaze.py）：
  检测（face_detection_short_range 128x128）→ 锚解码/NMS → detection2roi
  → 仿射 warp（192x192）→ 关键点（face_landmark）→ 逆映射回原图归一化。
后端注入（backend.py 协议）可测；真实后端 = rknnlite.api.RKNNLite（板 NPU，
2026-08-06 起板上推理统一走 RKNNLite，不再使用完整版 rknn.api.RKNN）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import cv2
import numpy as np

from .backend import Backend, RKNNLiteBackend
from .blaze import (
    DetectConfig,
    denormalize_detections,
    denormalize_landmarks,
    detection2roi,
    nms,
    resize_pad,
    roi_affine,
    tensors_to_detections,
    warp_roi,
)

DET_SIZE = 128      # 检测器输入
LM_SIZE = 192       # 关键点输入
DET_CFG = DetectConfig()  # face_detection_short_range v0.10

# 官方 FACEMESH_CONTOURS 拓扑（唇/眼/眉/脸廓），提取自 mediapipe 0.10.11 wheel
# mediapipe/python/solutions/face_mesh_connections.py（Apache-2.0）
FACEMESH_CONTOURS: frozenset = frozenset([
    (61, 146), (146, 91), (91, 181), (181, 84), (84, 17), (17, 314), (314, 405),
    (405, 321), (321, 375), (375, 291), (61, 185), (185, 40), (40, 39), (39, 37),
    (37, 0), (0, 267), (267, 269), (269, 270), (270, 409), (409, 291), (78, 95),
    (95, 88), (88, 178), (178, 87), (87, 14), (14, 317), (317, 402), (402, 318),
    (318, 324), (324, 308), (78, 191), (191, 80), (80, 81), (81, 82), (82, 13),
    (13, 312), (312, 311), (311, 310), (310, 415), (415, 308),
    (263, 249), (249, 390), (390, 373), (373, 374), (374, 380), (380, 381),
    (381, 382), (382, 362), (263, 466), (466, 388), (388, 387), (387, 386),
    (386, 385), (385, 384), (384, 398), (398, 362),
    (276, 283), (283, 282), (282, 295), (295, 285), (300, 293), (293, 334),
    (334, 296), (296, 336),
    (33, 7), (7, 163), (163, 144), (144, 145), (145, 153), (153, 154), (154, 155),
    (155, 133), (33, 246), (246, 161), (161, 160), (160, 159), (159, 158),
    (158, 157), (157, 173), (173, 133),
    (46, 53), (53, 52), (52, 65), (65, 55), (70, 63), (63, 105), (105, 66),
    (66, 107),
    (10, 338), (338, 297), (297, 332), (332, 284), (284, 251), (251, 389),
    (389, 356), (356, 454), (454, 323), (323, 361), (361, 288), (288, 397),
    (397, 365), (365, 379), (379, 378), (378, 400), (400, 377), (377, 152),
    (152, 148), (148, 176), (176, 149), (149, 150), (150, 136), (136, 172),
    (172, 58), (58, 132), (132, 93), (93, 234), (234, 127), (127, 162),
    (162, 21), (21, 54), (54, 103), (103, 67), (67, 109), (109, 10),
])
@dataclass
class LandmarkResult:
    """一帧的检测结果：每张脸的归一化关键点 (N,3)（x,y 原图归一化，z 相对尺度）。

    boxes/scores 与 faces 平行：box = 检测框原图像素 (xmin, ymin, xmax, ymax)（YOLO 风格绘制用），
    score = BlazeFace 检测置信度（sigmoid 后）。
    """
    faces: list[np.ndarray] = field(default_factory=list)
    boxes: list[tuple[int, int, int, int]] = field(default_factory=list)
    scores: list[float] = field(default_factory=list)

    @property
    def num_faces(self) -> int:
        return len(self.faces)


class FaceLandmarkerEngine:
    """RKNN BlazeFace 检测 + FaceLandmark 468 点。

    create_fn 注入 (detector_backend, landmark_backend)（backend.py 协议）；
    真实路径懒加载 rknnlite.api.RKNNLite（RKNNLiteBackend，core_mask 可配）。
    """

    def __init__(
        self,
        detector_model: str,
        landmark_model: str,
        num_faces: int = 2,
        min_detection_confidence: float = 0.5,
        create_fn: Callable | None = None,
        core_mask: int = 0,
    ):
        self.detector_model = detector_model
        self.landmark_model = landmark_model
        self.num_faces = num_faces
        self.min_detection_confidence = min_detection_confidence
        self._create_fn = create_fn
        self._core_mask = core_mask
        self._det_backend: Backend | None = None
        self._lm_backend: Backend | None = None

    def _ensure_created(self):
        if self._det_backend is not None:
            return
        if self._create_fn is not None:
            self._det_backend, self._lm_backend = self._create_fn()
            return
        det = RKNNLiteBackend(core_mask=self._core_mask)
        det.load(self.detector_model)
        lm = RKNNLiteBackend(core_mask=self._core_mask)
        lm.load(self.landmark_model)
        self._det_backend, self._lm_backend = det, lm

    def _prepare(self, img: np.ndarray, size: int) -> np.ndarray:
        """BGR → RGB float32 [0,1] NHWC（mediapipe 输入契约）。"""
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return (rgb.astype(np.float32) / 255.0).reshape(1, size, size, 3)

    def process(self, frame_bgr: np.ndarray, timestamp_ms: int | None = None) -> LandmarkResult:
        """输入 BGR 帧，返回每脸 468 归一化关键点（无脸 → faces=[]）。"""
        del timestamp_ms
        self._ensure_created()
        h, w = frame_bgr.shape[:2]

        det_img, scale, pad = resize_pad(frame_bgr, DET_SIZE)
        reg, cls = self._det_backend.predict(self._prepare(det_img, DET_SIZE))
        dets = tensors_to_detections(np.asarray(reg).reshape(-1, DET_CFG.num_coords),
                                     np.asarray(cls).reshape(-1, 1), DET_CFG)
        dets = denormalize_detections(dets, scale, pad)
        dets = nms(dets, DET_CFG.min_suppression_threshold)[: self.num_faces]

        faces = []
        result_boxes: list[tuple[int, int, int, int]] = []
        result_scores: list[float] = []
        for det in dets:
            roi = detection2roi([det], DET_CFG)[0]
            m = roi_affine(roi, LM_SIZE)
            crop = warp_roi(frame_bgr, m, LM_SIZE)
            outs = self._lm_backend.predict(self._prepare(crop, LM_SIZE))
            # presence 是 logit：先过稳定 sigmoid 再与阈值比（mediapipe 语义）
            logit = float(np.asarray(outs[1]).reshape(-1)[0])
            x = np.clip(logit, -88.0, 88.0)
            presence = (1.0 / (1.0 + np.exp(-x)) if x >= 0
                        else np.exp(x) / (1.0 + np.exp(x)))
            if presence < self.min_detection_confidence:
                continue
            pts = np.asarray(outs[0]).reshape(468, 3).astype(np.float32)
            faces.append(denormalize_landmarks(pts, m, LM_SIZE, h, w))
            # 检测框：原图像素、裁剪到帧内；与 faces 平行（YOLO 风格绘制）
            box = (int(round(det.xmin)), int(round(det.ymin)),
                   int(round(det.xmax)), int(round(det.ymax)))
            box = (max(0, box[0]), max(0, box[1]),
                   min(w, box[2]), min(h, box[3]))
            result_boxes.append(box)
            result_scores.append(det.score)
        return LandmarkResult(faces=faces, boxes=result_boxes, scores=result_scores)

    def close(self):
        for b in (self._det_backend, self._lm_backend):
            if b is not None:
                b.close()
        self._det_backend = self._lm_backend = None


POINT_COLOR = (0, 255, 255)    # 黄：关键点
LINE_COLOR = (0, 255, 0)       # 绿：轮廓线
BOX_COLOR = (0, 255, 0)        # 绿：检测框（YOLO 风格）
POINT_RADIUS = 2
LINE_THICKNESS = 1
BOX_THICKNESS = 2


def draw_boxes(
    img: np.ndarray,
    result: LandmarkResult,
    color=BOX_COLOR,
    thickness: int = BOX_THICKNESS,
    label: bool = True,
) -> np.ndarray:
    """在原图上绘制人脸检测框（YOLO 风格：矩形 + `face <score>` 标签）；返回副本。"""
    out = img.copy()
    for box, score in zip(result.boxes, result.scores):
        xmin, ymin, xmax, ymax = box
        cv2.rectangle(out, (xmin, ymin), (xmax, ymax), color, thickness)
        if not label:
            continue
        text = f"face {score:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ly0 = max(0, ymin - th - 6)
        cv2.rectangle(out, (xmin, ly0), (xmin + tw + 4, ymin), color, -1)
        cv2.putText(out, text, (xmin + 2, ymin - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out


def draw_mesh(
    img: np.ndarray,
    result: LandmarkResult,
    connections: frozenset = FACEMESH_CONTOURS,
    point_color=POINT_COLOR,
    line_color=LINE_COLOR,
    point_radius: int = POINT_RADIUS,
    line_thickness: int = LINE_THICKNESS,
) -> np.ndarray:
    """在原图上绘制关键点网格；返回副本。"""
    out = img.copy()
    h, w = out.shape[:2]
    for pts in result.faces:
        px = np.clip(pts[:, 0] * w, 0, w - 1).astype(np.int32)
        py = np.clip(pts[:, 1] * h, 0, h - 1).astype(np.int32)
        for a, b in connections:
            if a < len(px) and b < len(px):
                cv2.line(out, (int(px[a]), int(py[a])),
                         (int(px[b]), int(py[b])), line_color, line_thickness)
        for x, y in zip(px, py):
            cv2.circle(out, (int(x), int(y)), point_radius, point_color, -1)
    return out
