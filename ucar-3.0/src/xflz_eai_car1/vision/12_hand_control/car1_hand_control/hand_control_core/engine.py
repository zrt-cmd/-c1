"""手势引擎（CSDN 方案 4 模型管线 + mediapipe 官方跟踪状态机）与骨架绘制。

管线（板端实测 2026-08-07，用户重转模型 + 参考仓库 gesture_recognizer 语义）：
  LANDMARK_DETECTION（检测模式）：hand_detector（192 letterbox float32/255）
    → blazeface 解码/NMS → union_bbox 合并（特写大图 palm 分裂多框）
    → detection2roi box 模式（旋转方形，dscale 2.6）→ 224 仿射裁剪
  LANDMARK_TRACKING（跟踪模式，跳过 palm）：上帧 21 点 → detection2roi
    alignment 模式（腕 0 + 中指根 9）→ 旋转方形 ROI → 仿射裁剪
  两模式共同：hand_landmarks_detector（224 float32/255）→ 21 点 [x,y,z] + presence
    → gesture_embedder（norm_tensor 单输入：x/y/z 全部 /224）→ 128 维
    → canned_gesture_classifier（128 维）→ 8 类手势 softmax
  丢失判定：presence < min_tracking_confidence → 下帧回退全图检测（官方状态机）
后端注入（backend.py 协议）可测；真实后端 = rknnlite.api.RKNNLite。
"""
from __future__ import annotations

from typing import Callable

import cv2
import numpy as np

from .backend import Backend, RKNNLiteBackend
from .gesture import decode_gesture, prepare_embedder_inputs
from .hand import (
    HAND_CONNECTIONS,
    HAND_SIZE,
    PALM_SIZE,
    Hand,
    HandResult,
    decode_hand,
    decode_palm,
    detection2roi,
    detection_box2roi,
    palm_anchors,
    prepare,
    resize_pad,
    roi_affine,
    scale_detections,
    union_bbox,
    unproject_landmarks,
    warp_roi,
)

POINT_COLOR = (0, 0, 255)      # 红：关键点（vendor）
LINE_COLOR = (0, 255, 0)       # 绿：连线（vendor）
POINT_RADIUS = 6
LINE_THICKNESS = 2
MIN_DET_SCORE = 0.5
MIN_HAND_SCORE = 0.5


class HandEngine:
    """4 模型手势引擎（create_fn 注入 4 后端，x86 测试免 rknn）。

    官方跟踪状态机：上帧 21 点 presence 达标 → tracking（跳过 palm）；
    presence 丢失 → 下帧 detection（全图 palm）。
    """

    def __init__(
        self,
        detector_model: str,
        landmark_model: str,
        embedder_model: str,
        classifier_model: str,
        max_hands: int = 2,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        create_fn: Callable | None = None,
        core_mask: int = 0,
        roi_dscale: float = 2.6,
        roi_dy: float = -0.5,
    ):
        self.detector_model = detector_model
        self.landmark_model = landmark_model
        self.embedder_model = embedder_model
        self.classifier_model = classifier_model
        self.max_hands = max_hands
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self._create_fn = create_fn
        self._core_mask = core_mask
        self._roi_dscale = roi_dscale
        self._roi_dy = roi_dy
        self._det: Backend | None = None
        self._lm: Backend | None = None
        self._emb: Backend | None = None
        self._clf: Backend | None = None
        self._last_landmarks: np.ndarray | None = None   # 上帧 21 点（原图像素）
        self._last_presence: float = 0.0

    def _ensure_created(self):
        if self._det is not None:
            return
        if self._create_fn is not None:
            (self._det, self._lm, self._emb, self._clf) = self._create_fn()
            return
        det = RKNNLiteBackend(core_mask=self._core_mask)
        det.load(self.detector_model)
        lm = RKNNLiteBackend(core_mask=self._core_mask)
        lm.load(self.landmark_model)
        emb = RKNNLiteBackend(core_mask=self._core_mask)
        emb.load(self.embedder_model)
        clf = RKNNLiteBackend(core_mask=self._core_mask)
        clf.load(self.classifier_model)
        self._det, self._lm, self._emb, self._clf = det, lm, emb, clf

    def _detect_roi(self, frame_bgr: np.ndarray):
        """检测模式：全图 palm → union 矩形 → letterbox 224；返回 (roi224, scale, pad, box) 或 None。

        特写大图 palm 分裂多框时 keypoints 不可靠（dets[0] 可能是手指碎片框），
        box 模式旋转 ROI 会旋歪 → 用 union 矩形 + letterbox（板端实测稳定）。
        """
        h, w = frame_bgr.shape[:2]
        det_img, scale, pad = resize_pad(frame_bgr, PALM_SIZE)
        reg, cls = self._det.predict(prepare(det_img, PALM_SIZE))
        anchors = palm_anchors(PALM_SIZE)
        dets = decode_palm(np.asarray(reg), np.asarray(cls), anchors, PALM_SIZE,
                           score_thresh=self.min_detection_confidence)
        dets = scale_detections(dets, scale, pad, (h, w))
        if not dets:
            return None
        box = union_bbox(dets, (h, w))
        x1, y1, x2, y2 = box
        if x2 - x1 < 4 or y2 - y1 < 4:
            return None
        roi = frame_bgr[y1:y2, x1:x2]
        if roi.size == 0 or min(roi.shape[:2]) < 4:
            return None
        roi_img, roi_scale, roi_pad = resize_pad(roi, HAND_SIZE)
        return roi_img, roi_scale, roi_pad, box

    def _track_roi(self, frame_bgr: np.ndarray):
        """跟踪模式：上帧 21 点 → ROI → 224。

        官方 detection2roi 旋转方形（腕+中指根对齐）在手适中时最稳；
        特写大图（手占画面 80%+）时 scale=2×L×dscale 超出帧短边，旋转 ROI
        里手占比异常 → landmark 输出振荡（板端实测）——改走矩形 bbox×1.5
        （与 detection 的 union 空间一致，稳定优先）。
        返回 (roi224, mode, extra)：mode='affine' 时 extra=(m_inv)；
        mode='rect' 时 extra=(roi_scale, roi_pad, x1, y1)。
        """
        h, w = frame_bgr.shape[:2]
        xc, yc, roi_scale, theta = detection2roi(
            self._last_landmarks, dscale=self._roi_dscale, dy=self._roi_dy)
        if roi_scale <= min(h, w):
            m, m_inv = roi_affine(xc, yc, roi_scale, theta, HAND_SIZE)
            return warp_roi(frame_bgr, m, HAND_SIZE), "affine", (m_inv,)
        # 特写：矩形 bbox × 1.5（中心扩展，裁剪到帧）
        lm = self._last_landmarks
        x1, y1 = float(lm[:, 0].min()), float(lm[:, 1].min())
        x2, y2 = float(lm[:, 0].max()), float(lm[:, 1].max())
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        side = max(x2 - x1, y2 - y1) * 1.5
        rx1, ry1 = max(0.0, cx - side / 2), max(0.0, cy - side / 2)
        rx2, ry2 = min(float(w), cx + side / 2), min(float(h), cy + side / 2)
        roi = frame_bgr[int(ry1):int(ry2), int(rx1):int(rx2)]
        if roi.size == 0 or min(roi.shape[:2]) < 4:
            return None, "none", ()
        roi_img, roi_scale, roi_pad = resize_pad(roi, HAND_SIZE)
        return roi_img, "rect", (roi_scale, roi_pad, int(rx1), int(ry1))

    def _classify(self, raw_joints: np.ndarray) -> tuple[str, float]:
        """embedding + 分类（norm_tensor 单输入，x/y/z 全部 /224）。"""
        emb_inputs = prepare_embedder_inputs(raw_joints, input_size=HAND_SIZE)
        emb = self._emb.predict(emb_inputs[0])
        clf_out = self._clf.predict([np.asarray(emb[0])])
        gesture = decode_gesture(clf_out[0])
        return gesture["label"], gesture["score"]

    def process(self, frame_bgr: np.ndarray) -> HandResult:
        """检测/跟踪状态机 → 21 点 → embedding → 分类；无手 → 空结果不崩。"""
        self._ensure_created()
        h, w = frame_bgr.shape[:2]

        # 模式选择：上帧 presence 达标 → 跟踪（跳过 palm）；否则全图检测
        tracking = (self._last_landmarks is not None
                    and self._last_presence >= self.min_tracking_confidence)
        if tracking:
            roi_img, mode, extra = self._track_roi(frame_bgr)
            if mode == "none":
                self._last_landmarks = None
                self._last_presence = 0.0
                return HandResult(hands=[])
            outs = self._lm.predict(prepare(roi_img, HAND_SIZE))
            raw_joints = np.asarray(outs[0]).reshape(21, 3)      # 224 空间 [x,y,z]
            score = float(np.asarray(outs[1]).reshape(-1)[0])
            if score < self.min_tracking_confidence:
                self._last_landmarks = None
                self._last_presence = 0.0
                return HandResult(hands=[])
            if mode == "affine":
                landmarks = unproject_landmarks(raw_joints, extra[0], HAND_SIZE)
            else:  # rect
                roi_scale, roi_pad, rx1, ry1 = extra
                pad_w, pad_h = roi_pad
                landmarks = raw_joints.copy().astype(np.float32)
                landmarks[:, 0] = (landmarks[:, 0] - pad_w) / roi_scale + rx1
                landmarks[:, 1] = (landmarks[:, 1] - pad_h) / roi_scale + ry1
                landmarks[:, 2] = landmarks[:, 2] / roi_scale
            handedness = float(np.asarray(outs[2]).reshape(-1)[0])
            world = np.asarray(outs[3]).reshape(21, 3).astype(np.float32)
            box = None
        else:
            found = self._detect_roi(frame_bgr)
            if found is None:
                self._last_landmarks = None
                self._last_presence = 0.0
                return HandResult(hands=[])
            roi_img, roi_scale, roi_pad, box = found
            x1, y1, x2, y2 = box
            outs = self._lm.predict(prepare(roi_img, HAND_SIZE))
            raw_joints = np.asarray(outs[0]).reshape(21, 3)      # 224 空间 [x,y,z]
            hand = decode_hand(outs, scale=roi_scale, pad=roi_pad,
                               orig_shape=(y2 - y1, x2 - x1))
            score = hand.score
            if score < self.min_tracking_confidence:
                self._last_landmarks = None
                self._last_presence = 0.0
                return HandResult(hands=[])
            landmarks = hand.landmarks                       # ROI 空间
            landmarks[:, 0] += x1                            # → 原图像素
            landmarks[:, 1] += y1
            world = hand.world
            handedness = hand.handedness

        hand = Hand(landmarks=landmarks, score=score,
                    handedness=handedness, world=world, box=box)
        label, gscore = self._classify(raw_joints)
        hand.gesture_label = label
        hand.gesture_score = gscore

        self._last_landmarks = landmarks.copy()
        self._last_presence = score
        return HandResult(hands=[hand])

    def close(self):
        for b in (self._det, self._lm, self._emb, self._clf):
            if b is not None:
                b.close()
        self._det = self._lm = self._emb = self._clf = None
        self._last_landmarks = None
        self._last_presence = 0.0


def draw_boxes(
    img: np.ndarray,
    result: HandResult,
    color=(0, 255, 0),
    thickness: int = 2,
) -> np.ndarray:
    """在原图绘制手掌检测框（YOLO 风格）；返回副本。"""
    out = img.copy()
    for hand in result.hands:
        if hand.box is None:
            continue
        x1, y1, x2, y2 = hand.box
        cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness)
        text = f"{hand.gesture_label} {hand.gesture_score:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ly0 = max(0, y1 - th - 6)
        cv2.rectangle(out, (x1, ly0), (x1 + tw + 4, y1), color, -1)
        cv2.putText(out, text, (x1 + 2, y1 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out


def draw_hands(
    img: np.ndarray,
    result: HandResult,
    connections: frozenset = HAND_CONNECTIONS,
    point_color=POINT_COLOR,
    line_color=LINE_COLOR,
    point_radius: int = POINT_RADIUS,
    line_thickness: int = LINE_THICKNESS,
) -> np.ndarray:
    """在给定图上绘制 21 点手骨架 + 手势名；返回副本。"""
    out = img.copy()
    h, w = out.shape[:2]
    for hand in result.hands:
        lm = hand.landmarks
        x = np.clip(lm[:, 0], 0, w - 1).astype(np.int32)
        y = np.clip(lm[:, 1], 0, h - 1).astype(np.int32)
        for a, b in connections:
            if a < len(x) and b < len(x):
                cv2.line(out, (int(x[a]), int(y[a])),
                         (int(x[b]), int(y[b])), line_color, line_thickness)
        for px, py in zip(x, y):
            cv2.circle(out, (int(px), int(py)), point_radius, point_color, -1)
        if hand.gesture_label != "None":
            x0 = int(lm[:, 0].mean()) - 40
            y0 = max(20, int(lm[:, 1].min()) - 10)
            text = f"{hand.gesture_label} {hand.gesture_score:.2f}"
            cv2.putText(out, text, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 0, 0), 2, cv2.LINE_AA)
    return out
