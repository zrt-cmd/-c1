"""blaze 模块测试（锚/解码/NMS/ROI/仿射/逆映射，纯 numpy/cv2）。"""
from __future__ import annotations

import numpy as np
import pytest

from face_landmark_core.blaze import (
    AnchorOptions,
    DetectConfig,
    Detection,
    ROI,
    decode_boxes,
    decode_keypoints,
    denormalize_detections,
    denormalize_landmarks,
    detection2roi,
    generate_anchors,
    nms,
    resize_pad,
    roi_affine,
    tensors_to_detections,
    warp_roi,
)

CFG = DetectConfig()


def test_anchor_count_and_range():
    anchors = generate_anchors(AnchorOptions(input_size=128))
    assert anchors.shape == (896, 4)  # mediapipe 契约
    assert np.all((anchors[:, 0] >= 0) & (anchors[:, 0] <= 1))
    assert np.all(anchors[:, 2] == 1.0)  # fixed_anchor_size


def test_anchor_small_input():
    anchors = generate_anchors(AnchorOptions(input_size=64))
    # 4 层：8/16/16/16 → fmap 8/4/4/4 → 64+16+16+16=112 锚 × 2（interpolated）= 224
    assert anchors.shape[0] == 224


def test_decode_boxes_identity():
    # 零回归 → 框中心 = 锚中心，尺寸 ≈ 0（退化框）
    anchors = np.array([[0.5, 0.5, 1.0, 1.0]], dtype=np.float32)
    raw = np.zeros((1, 16), dtype=np.float32)
    boxes = decode_boxes(raw, anchors, CFG)
    assert boxes.shape == (1, 4)
    assert abs((boxes[0, 1] + boxes[0, 3]) / 2 - 0.5) < 1e-5   # x 中心
    assert abs((boxes[0, 0] + boxes[0, 2]) / 2 - 0.5) < 1e-5   # y 中心
    assert abs(boxes[0, 3] - boxes[0, 1]) < 1e-5               # 宽≈0


def test_decode_boxes_offset():
    # 回归 +32/128 → x 中心偏移 +0.25（验证解码方向与尺度）
    anchors = np.array([[0.5, 0.5, 1.0, 1.0]], dtype=np.float32)
    raw = np.zeros((1, 16), dtype=np.float32)
    raw[0, 0] = 32.0
    boxes = decode_boxes(raw, anchors, CFG)
    assert abs((boxes[0, 1] + boxes[0, 3]) / 2 - 0.75) < 1e-4


def test_decode_keypoints():
    anchors = np.array([[0.5, 0.5, 1.0, 1.0]], dtype=np.float32)
    raw = np.zeros((1, 16), dtype=np.float32)
    raw[0, 4] = 64.0  # kp0.x → 0.5 + 64/128 = 1.0
    kps = decode_keypoints(raw, anchors, CFG)
    assert kps.shape == (1, 6, 2)
    assert abs(kps[0, 0, 0] - 1.0) < 1e-4


def test_tensors_to_detections_filter_and_sigmoid():
    anchors = generate_anchors(AnchorOptions(input_size=128))
    reg = np.zeros((896, 16), dtype=np.float32)
    cls = np.full((896, 1), 5.0, dtype=np.float32)   # sigmoid(5)≈0.993
    cls[0] = -100.0                                   # sigmoid≈0 → 过滤
    dets = tensors_to_detections(reg, cls, CFG, anchors)
    assert len(dets) == 895
    assert dets[0].score > 0.99


def test_nms_removes_overlap_keeps_distinct():
    a = Detection(0.1, 0.1, 0.5, 0.5, 0.9, [])
    b = Detection(0.15, 0.15, 0.55, 0.55, 0.8, [])  # 与 a 高重叠
    c = Detection(0.6, 0.6, 0.9, 0.9, 0.7, [])
    out = nms([a, b, c], 0.3)
    assert out == [a, c]


def test_resize_pad_letterbox():
    img = np.zeros((240, 480, 3), dtype=np.uint8)
    padded, scale, (px, py) = resize_pad(img, 128)
    assert padded.shape == (128, 128, 3)
    assert abs(scale - 128 / 480) < 1e-5
    assert px == 0  # 宽边占满 → 左右无 pad
    assert py == (128 - 240 * scale) // 2


def test_denormalize_detections():
    # 逆 letterbox：px = (norm*128 - pad) / scale
    det = Detection(0.5, 0.5, 0.6, 0.6, 0.9, [(0.5, 0.5)])
    out = denormalize_detections([det], 0.5, (4, 8))
    assert abs(out[0].xmin - (0.5 * 128 - 4) / 0.5) < 1e-3
    assert abs(out[0].keypoints[0][1] - (0.5 * 128 - 8) / 0.5) < 1e-3


def test_detection2roi_box_method():
    d = Detection(0.2, 0.3, 0.6, 0.5, 0.9, [])
    roi = detection2roi([d], CFG)[0]
    assert abs(roi.xc - 0.4) < 1e-5
    assert abs(roi.yc - 0.4) < 1e-5
    assert abs(roi.scale - (0.6 - 0.2) * CFG.dscale) < 1e-5  # dscale 1.5
    assert roi.theta == 0.0


def test_detection2roi_rotation_from_keypoints():
    # kp1=1, kp2=0（CFG）：theta = atan2(y1-y0, x1-x0)
    # 令 kp[1]=(0,0)、kp[0]=(1,0) → atan2(0, -1) = π
    d = Detection(0.1, 0.1, 0.5, 0.5, 0.9,
                  keypoints=[(1.0, 0.0), (0.0, 0.0), (0, 0), (0, 0), (0, 0), (0, 0)])
    roi = detection2roi([d], CFG)[0]
    assert abs(abs(roi.theta) - np.pi) < 1e-4

def test_roi_affine_centers_roi():
    roi = ROI(xc=100.0, yc=50.0, scale=20.0, theta=0.0)
    m = roi_affine(roi, 192)
    # ROI 中心 (100,50) 应映射到 (96,96)
    src = np.array([[100.0, 50.0]], dtype=np.float32)
    dst = src @ m[:, :2].T + m[:, 2]
    assert abs(dst[0, 0] - 96.0) < 1.0
    assert abs(dst[0, 1] - 96.0) < 1.0


def test_warp_roi_shape():
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    m = roi_affine(ROI(100, 100, 100, 0.0), 192)
    crop = warp_roi(img, m, 192)
    assert crop.shape == (192, 192, 3)


def test_denormalize_landmarks_roundtrip():
    # 恒等仿射（无旋转缩放，中心对齐 192）：ROI 中心像素 (96,96) → 原图中心
    img_w, img_h = 640, 480
    m = roi_affine(ROI(xc=320, yc=240, scale=192, theta=0.0), 192)
    lm = np.array([[96.0, 96.0, 0.0]], dtype=np.float32)  # 模型输出 = ROI 像素坐标
    out = denormalize_landmarks(lm, m, 192, img_h, img_w)
    assert abs(out[0, 0] - 0.5) < 0.01   # (320/640)
    assert abs(out[0, 1] - 0.5) < 0.01   # (240/480)


def test_denormalize_landmarks_normalizes_z():
    m = roi_affine(ROI(100, 100, 100, 0.0), 192)
    lm = np.array([[0.1, 0.2, -0.5]], dtype=np.float32)  # z 与 x/y 同尺度（像素）
    out = denormalize_landmarks(lm, m, 192, 480, 640)
    assert abs(out[0, 2] - (-0.5 / 192.0)) < 1e-6
