"""引擎与绘制测试（fake 后端注入，无 rknn）。"""
from __future__ import annotations

import numpy as np
import pytest

from pose_landmark_core.backend import FakeBackend
from pose_landmark_core.engine import PoseEngine, draw_skeleton
from pose_landmark_core.pose import PoseResult

from test_pose import make_outputs


def make_engine(score=5.0, **pose_kwargs):
    backend = FakeBackend(fn=lambda img: make_outputs(score=score))
    eng = PoseEngine("pose.rknn", create_fn=lambda: backend, **pose_kwargs)
    return eng, backend


def test_engine_lazy_and_process():
    eng, backend = make_engine()
    assert eng._backend is None          # 懒加载
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert backend.loaded is None        # fake 不走 load
    assert res.detected
    assert res.landmarks.shape == (33, 4)


def test_engine_center_maps_back():
    # 256 空间中心 (128,128) → 原图像素 (160,120)
    eng, _ = make_engine()
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert abs(res.landmarks[0, 0] - 160.0) < 1e-3
    assert abs(res.landmarks[0, 1] - 120.0) < 1e-3


def test_engine_low_score_filtered():
    eng, _ = make_engine(score=0.3)      # < min_detection_confidence 0.5
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert not res.detected
    assert res.landmarks is None


def test_engine_close():
    eng, backend = make_engine()
    eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    eng.close()
    assert eng._backend is None


def test_engine_stores_core_mask():
    eng = PoseEngine("pose.rknn", core_mask=7)
    assert eng._core_mask == 7
    assert eng._create_fn is None


def test_draw_skeleton_blank_when_no_pose():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    out = draw_skeleton(img, PoseResult())
    assert np.array_equal(out, img)


def test_draw_skeleton_paints_points_and_lines():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    lm = np.zeros((33, 4), dtype=np.float32)
    lm[11] = (20, 30, 0.0, 0.9)          # 肩
    lm[12] = (80, 30, 0.0, 0.9)
    lm[0] = (50, 10, 0.0, 0.2)           # 鼻（vis 低，不画）
    out = draw_skeleton(img, PoseResult(landmarks=lm))
    assert not np.array_equal(out, img)
    assert tuple(out[30, 20]) == (0, 0, 255)   # 红点
    assert tuple(out[30, 50]) == (0, 255, 0)   # 绿线（肩连线中点）


def test_draw_skeleton_skips_low_vis():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    lm = np.zeros((33, 4), dtype=np.float32)
    lm[11] = (20, 30, 0.0, 0.9)
    lm[12] = (80, 30, 0.0, 0.9)
    lm[13] = (20, 60, 0.0, 0.1)          # 低 vis
    out = draw_skeleton(img, PoseResult(landmarks=lm))
    assert tuple(out[60, 20]) == (0, 0, 0)     # 低 vis 点不画


def test_draw_skeleton_returns_copy_and_clips():
    img = np.zeros((50, 50, 3), dtype=np.uint8)
    lm = np.zeros((33, 4), dtype=np.float32)
    lm[11] = (200, 200, 0.0, 0.9)          # 越界 → 裁剪
    out = draw_skeleton(img, PoseResult(landmarks=lm))
    assert out is not img
    assert np.array_equal(img, np.zeros_like(img))
