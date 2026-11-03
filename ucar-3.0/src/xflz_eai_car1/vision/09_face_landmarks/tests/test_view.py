"""view 模块测试。"""
from __future__ import annotations

import numpy as np
import pytest

from face_landmark_core.view import add_fps, compose, put_status


def test_compose_shapes():
    a = np.zeros((100, 120, 3), dtype=np.uint8)
    b = np.zeros((80, 160, 3), dtype=np.uint8)
    out = compose(a, b)
    assert out.shape == (100, 280, 3)


def test_compose_left_right_content():
    a = np.full((50, 40, 3), 10, dtype=np.uint8)
    b = np.full((50, 40, 3), 200, dtype=np.uint8)
    out = compose(a, b)
    assert np.all(out[:, :40] == 10)
    assert np.all(out[:, 40:] == 200)


def test_compose_unequal_height_pads():
    a = np.full((60, 40, 3), 1, dtype=np.uint8)
    b = np.full((30, 40, 3), 2, dtype=np.uint8)
    out = compose(a, b)
    assert out.shape == (60, 80, 3)
    assert np.all(out[30:60, 40:] == 0)  # 右栏下半为黑


def test_compose_rejects_gray():
    a = np.zeros((10, 10), dtype=np.uint8)
    b = np.zeros((10, 10, 3), dtype=np.uint8)
    with pytest.raises(ValueError):
        compose(a, b)


def test_put_status_draws_text():
    img = np.zeros((60, 200, 3), dtype=np.uint8)
    out = put_status(img, 24.7, 2)
    assert out.shape == img.shape
    bottom = out[36:, :]
    assert np.any(bottom > 0)  # 黑条上有白字
    assert np.all(img[36:, :] == 0)  # 原图未动（返回副本）


def test_put_status_tiny_image():
    img = np.zeros((10, 100, 3), dtype=np.uint8)
    out = put_status(img, 5.0, 0)
    assert out.shape == img.shape


def test_add_fps_draws_red_text():
    img = np.zeros((80, 200, 3), dtype=np.uint8)
    out = add_fps(img, 30)
    assert np.any((out[:, :, 2] == 255))  # 红字
    assert np.array_equal(img, np.zeros_like(img))
