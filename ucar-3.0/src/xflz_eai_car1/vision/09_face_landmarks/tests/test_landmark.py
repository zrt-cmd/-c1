"""landmark 模块测试（RKNN 引擎，fake 双后端注入，无 rknn）。"""
from __future__ import annotations

import numpy as np
import pytest

from face_landmark_core.backend import FakeBackend
from face_landmark_core.blaze import DetectConfig, generate_anchors, AnchorOptions
from face_landmark_core.landmark import (
    BOX_COLOR,
    FACEMESH_CONTOURS,
    FaceLandmarkerEngine,
    LandmarkResult,
    draw_boxes,
    draw_mesh,
)


def det_outputs(center_norm=(0.5, 0.5), size_norm=0.2, score=5.0, num=1):
    """构造检测器输出：把锚 0 的回归设为已知框（128 空间归一化）。"""
    cfg = DetectConfig()
    anchors = generate_anchors(AnchorOptions(input_size=128))
    reg = np.zeros((896, 16), dtype=np.float32)
    cls = np.full((896, 1), -100.0, dtype=np.float32)
    # 锚 0 在 (0.03125, 0.03125)（fmap 16x16 网格角）
    a = anchors[0]
    cx, cy = center_norm
    hw = size_norm / 2.0
    reg[0, 0] = (cx - a[0]) * cfg.x_scale        # dx
    reg[0, 1] = (cy - a[1]) * cfg.y_scale        # dy
    reg[0, 2] = size_norm * cfg.w_scale          # w
    reg[0, 3] = size_norm * cfg.h_scale          # h
    cls[0] = score
    return reg, cls


def lm_outputs(point=(96.0, 96.0, 0.0), presence=15.0):
    """构造关键点输出：468 点全在 (x,y,z) —— x,y 为 ROI 192 像素坐标（模型语义），
    presence 为 logit（引擎内部过 sigmoid；15 → 0.9999997）。"""
    pts = np.zeros((1, 1, 1, 1404), dtype=np.float32)
    pts[0, 0, 0, 0::3] = point[0]
    pts[0, 0, 0, 1::3] = point[1]
    pts[0, 0, 0, 2::3] = point[2]
    return [pts, np.array([[[[presence]]]], dtype=np.float32)]


def make_engine(face_count=1, det_kwargs=None, lm_kwargs=None):
    det = FakeBackend(fn=lambda img: det_outputs(**(det_kwargs or {})))
    lm = FakeBackend(fn=lambda img: lm_outputs(**(lm_kwargs or {})))
    eng = FaceLandmarkerEngine(
        "models/face_detection_short_range.rknn",
        "models/face_landmark.rknn",
        num_faces=2,
        create_fn=lambda: (det, lm),
    )
    return eng, det, lm


def test_topology_indices_within_478():
    for a, b in FACEMESH_CONTOURS:
        assert 0 <= a < 478
        assert 0 <= b < 478
    # 唇 40 + 眼 32 + 眉 16 + 脸廓 36 = 124
    assert len(FACEMESH_CONTOURS) == 124


def test_topology_known_edges():
    assert (61, 146) in FACEMESH_CONTOURS  # 唇
    assert (263, 249) in FACEMESH_CONTOURS  # 右眼
    assert (276, 283) in FACEMESH_CONTOURS  # 右眉
    assert (10, 338) in FACEMESH_CONTOURS   # 脸廓


def test_result_num_faces():
    r = LandmarkResult(faces=[np.zeros((468, 3)), np.zeros((468, 3))])
    assert r.num_faces == 2
    assert LandmarkResult().num_faces == 0


def test_engine_lazy_and_process():
    eng, det, lm = make_engine()
    assert eng._det_backend is None  # 懒加载
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.num_faces == 1
    assert res.faces[0].shape == (468, 3)
    assert np.all((res.faces[0][:, 0] >= 0) & (res.faces[0][:, 0] <= 1))
    assert np.all((res.faces[0][:, 1] >= 0) & (res.faces[0][:, 1] <= 1))

def test_engine_center_landmark_maps_back():
    # 人脸在画面中心 (0.5,0.5)；ROI 中心像素 (96,96) → 关键点应回到 (0.5,0.5)
    eng, _, _ = make_engine(det_kwargs=dict(center_norm=(0.5, 0.5)),
                            lm_kwargs=dict(point=(96.0, 96.0, 0.0)))
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert abs(res.faces[0][0, 0] - 0.5) < 0.05
    assert abs(res.faces[0][0, 1] - 0.5) < 0.05

def test_engine_low_presence_filtered():
    eng, _, _ = make_engine(lm_kwargs=dict(presence=-5.0))  # sigmoid(-5)≈0.0067
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.num_faces == 0


def test_engine_no_face_empty():
    eng, _, _ = make_engine(det_kwargs=dict(score=-100.0))
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.num_faces == 0


def test_engine_num_faces_cap():
    # 两个不同锚高分 → 2 张脸；num_faces=2 不截断（构造单锚即可验证 ≤2）
    eng, _, _ = make_engine()
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.num_faces <= 2

def test_engine_z_normalized():
    eng, _, _ = make_engine(lm_kwargs=dict(point=(96.0, 96.0, -0.4)))
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert abs(res.faces[0][0, 2] - (-0.4 / 192.0)) < 1e-5


def test_engine_close():
    eng, det, lm = make_engine()
    eng.process(np.zeros((10, 10, 3), dtype=np.uint8))
    eng.close()
    assert det.loaded is None
    assert lm.loaded is None
    assert eng._det_backend is None


def test_engine_timestamp_ignored():
    eng, _, _ = make_engine()
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8), timestamp_ms=42)
    assert res.num_faces == 1


def test_engine_stores_core_mask():
    eng = FaceLandmarkerEngine("det.rknn", "lm.rknn", core_mask=7)
    assert eng._core_mask == 7
    assert eng._create_fn is None


def test_draw_mesh_blank_without_faces():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    out = draw_mesh(img, LandmarkResult())
    assert np.array_equal(out, img)


def test_draw_mesh_paints_points_and_lines():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    pts = np.zeros((478, 3), dtype=np.float32)
    pts[0] = (0.1, 0.5, 0.0)
    pts[1] = (0.9, 0.5, 0.0)
    out = draw_mesh(img, LandmarkResult(faces=[pts]), connections=frozenset([(0, 1)]))
    yellow = np.any((out[:, :, 2] == 255) & (out[:, :, 1] == 255) & (out[:, :, 0] == 0))
    green = np.any((out[:, :, 1] == 255) & (out[:, :, 2] == 0))
    assert yellow
    assert green
    assert not np.array_equal(out, img)


def test_draw_mesh_returns_copy_and_clips():
    img = np.zeros((50, 50, 3), dtype=np.uint8)
    pts = np.zeros((478, 3), dtype=np.float32)
    pts[:, 0] = 2.0
    pts[:, 1] = -1.0
    out = draw_mesh(img, LandmarkResult(faces=[pts]))
    assert out is not img
    assert np.array_equal(img, np.zeros_like(img))


def test_process_returns_box_and_score():
    # 320x240 帧、中心 (0.5,0.5) 尺寸 0.2：letterbox scale=0.4 pad_y=16
    # → 原图像素框 (128, 88, 192, 152)，score = sigmoid(5.0)
    eng, _, _ = make_engine()
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.num_faces == 1
    assert res.boxes[0] == (128, 88, 192, 152)
    s = 1.0 / (1.0 + np.exp(-5.0))
    assert abs(res.scores[0] - s) < 1e-4


def test_process_box_clipped_and_aligned():
    # 框与 faces 平行（同一长度）；框裁剪到帧内
    eng, _, _ = make_engine(det_kwargs=dict(center_norm=(0.25, 0.5)))
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert len(res.boxes) == len(res.faces) == res.num_faces
    for box in res.boxes:
        assert 0 <= box[0] < box[2] <= 320
        assert 0 <= box[1] < box[3] <= 240


def test_draw_boxes_blank_without_faces():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    out = draw_boxes(img, LandmarkResult())
    assert np.array_equal(out, img)


def test_draw_boxes_paints_rect_and_label():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    r = LandmarkResult(faces=[np.zeros((468, 3))], boxes=[(10, 20, 90, 80)],
                       scores=[0.9])
    out = draw_boxes(img, r)
    assert not np.array_equal(out, img)
    # 框线像素为绿色（线宽 2 → 边框含 ymin-1 行）
    assert tuple(out[19, 50]) == BOX_COLOR
    # 标签底色块在框上方（(30,3) 在底色内、字形上方，非原图）
    assert tuple(out[3, 30]) == BOX_COLOR


def test_draw_boxes_no_label():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    r = LandmarkResult(faces=[np.zeros((468, 3))], boxes=[(10, 20, 90, 80)],
                       scores=[0.9])
    out = draw_boxes(img, r, label=False)
    assert tuple(out[19, 50]) == BOX_COLOR  # 仍有框
    assert np.array_equal(out[3, 30], np.zeros(3, dtype=np.uint8))  # 无标签底色
