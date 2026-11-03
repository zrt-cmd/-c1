"""图像特效测试。"""
from __future__ import annotations

import numpy as np

from hand_control_core.effects import EFFECTS, apply_effect


def _img():
    rng = np.random.default_rng(7)
    return rng.integers(0, 256, size=(60, 80, 3), dtype=np.uint8)


def test_effects_list():
    assert EFFECTS == ["color", "thresh", "blur", "hue", "enhance"]


def test_color_is_identity():
    img = _img()
    out = apply_effect(img, "color", 100)
    assert np.array_equal(out, img)


def test_thresh_binary():
    img = _img()
    out = apply_effect(img, "thresh", 128)
    assert out.dtype == np.uint8 and out.shape == img.shape
    assert set(np.unique(out[:, :, 0])).issubset({0, 255})


def test_blur_changes_image():
    img = _img()
    out = apply_effect(img, "blur", 200)
    assert not np.array_equal(out, img)


def test_hue_shifts():
    import cv2
    img = _img()
    out = apply_effect(img, "hue", 90)
    assert not np.array_equal(out, img)
    h1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[:, :, 0].astype(int)
    h2 = cv2.cvtColor(out, cv2.COLOR_BGR2HSV)[:, :, 0].astype(int)
    diff = (h2 - h1) % 180
    # 大部分像素色相位移 ≈ 90（±2，除黑/灰等色相无意义像素）
    assert np.median(diff) in (90, 89, 91)


def test_enhance_runs():
    img = _img()
    out = apply_effect(img, "enhance", 120)
    assert out.shape == img.shape


def test_unknown_effect_identity():
    img = _img()
    assert np.array_equal(apply_effect(img, "nope", 10), img)
