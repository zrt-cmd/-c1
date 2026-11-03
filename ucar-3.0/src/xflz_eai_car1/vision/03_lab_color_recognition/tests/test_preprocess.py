"""preprocess 模块 TDD 测试（构建文档 02 的 6 条计划；HSV 契约）。"""
import numpy as np
import pytest

from config import ColorSpec
from preprocess import (auto_hsv_range, bgr_to_hsv, color_mask, detect_colors,
                        draw_detection, largest_circle, mask_view, morph_close,
                        morph_open, range_mask)
from preprocess import ColorDetect

RED = ColorSpec("红色", (0, 15), (100, 255), (80, 255), h2=(160, 180))
GREEN = ColorSpec("绿色", (35, 85), (80, 255), (80, 255))
BLUE = ColorSpec("蓝色", (90, 130), (80, 255), (80, 255))


def _cfg():
    return {
        "morph": {"open_kernel": 3, "close_kernel": 3, "open_iter": 1, "close_iter": 1},
        "contour": {"min_area": 500, "min_radius": 10, "max_radius": 400},
    }


def _circle_img(color_bgr, size=200, radius=40):
    """BGR 图像：中心实心圆。"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    cv2 = pytest.importorskip("cv2")
    cv2.circle(img, (size // 2, size // 2), radius, color_bgr, -1)
    return img


def test_bgr_to_hsv_shape_and_range():
    cv2 = pytest.importorskip("cv2")
    img = _circle_img((0, 0, 255))
    hsv = bgr_to_hsv(img)
    assert hsv.shape == img.shape
    assert hsv[..., 0].max() <= 179  # H∈0-179
    assert hsv[..., 1:].max() <= 255


def test_range_mask_selects_red():
    cv2 = pytest.importorskip("cv2")
    img = _circle_img((0, 0, 255))  # 纯红 BGR -> HSV H=0
    mask = range_mask(bgr_to_hsv(img), RED)
    assert mask[mask.shape[0] // 2, mask.shape[1] // 2] == 255
    assert mask[0, 0] == 0


def test_range_mask_h2_catches_red_wrap():
    cv2 = pytest.importorskip("cv2")
    # H=170 的"红"（品红偏红）只能被 h2=[160,180] 捕获
    img = np.zeros((20, 20, 3), dtype=np.uint8)
    img[5:15, 5:15] = (180, 60, 255)  # BGR -> HSV H≈165
    mask = range_mask(bgr_to_hsv(img), RED)
    assert mask[10, 10] == 255


def test_morph_open_removes_speck():
    cv2 = pytest.importorskip("cv2")
    mask = np.zeros((40, 40), dtype=np.uint8)
    mask[10, 10] = 255          # 单点噪点
    mask[15:25, 15:25] = 255    # 大块
    out = morph_open(mask, kernel=3)
    assert out[10, 10] == 0     # 噪点去除
    assert out[20, 20] == 255   # 大块保留


def test_morph_close_fills_hole():
    cv2 = pytest.importorskip("cv2")
    mask = np.zeros((40, 40), dtype=np.uint8)
    mask[10:30, 10:30] = 255
    mask[18:22, 18:22] = 0      # 孔洞
    out = morph_close(mask, kernel=5)
    assert out[20, 20] == 255


def test_largest_circle_synthetic():
    cv2 = pytest.importorskip("cv2")
    mask = np.zeros((200, 200), dtype=np.uint8)
    cv2.circle(mask, (100, 100), 40, 255, -1)
    c = largest_circle(mask, min_area=500)
    assert c is not None
    assert c.radius == pytest.approx(40, abs=3)


def test_largest_circle_empty_and_small():
    assert largest_circle(np.zeros((50, 50), dtype=np.uint8)) is None
    mask = np.zeros((50, 50), dtype=np.uint8)
    mask[10, 10] = 255
    assert largest_circle(mask, min_area=100) is None


def test_detect_colors_finds_red():
    img = _circle_img((0, 0, 255))
    dets = detect_colors(img, _cfg(), [RED])
    assert len(dets) == 1
    assert dets[0].color_name == "红色"
    assert dets[0].cx == pytest.approx(100, abs=6)
    assert dets[0].cy == pytest.approx(100, abs=6)


def test_detect_colors_three_colors():
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2 = pytest.importorskip("cv2")
    cv2.circle(img, (50, 100), 30, (0, 0, 255), -1)    # 红
    cv2.circle(img, (100, 100), 30, (0, 255, 0), -1)   # 绿
    cv2.circle(img, (150, 100), 30, (255, 0, 0), -1)   # 蓝
    dets = detect_colors(img, _cfg(), [RED, GREEN, BLUE])
    assert {d.color_name for d in dets} == {"红色", "绿色", "蓝色"}


def test_detect_colors_noise_empty():
    rng = np.random.default_rng(0)
    img = rng.integers(0, 255, (200, 200, 3), dtype=np.uint8)
    assert detect_colors(img, _cfg(), [RED]) == []


def test_auto_hsv_range_finds_red():
    cv2 = pytest.importorskip("cv2")
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(img, (100, 100), 60, (0, 0, 255), -1)  # 纯红 -> HSV H=0
    rng = auto_hsv_range(img, RED)
    assert rng is not None
    assert rng["h"][0] <= 0 <= rng["h"][1]   # 主段含 H=0
    assert rng["s"][0] <= 255 and rng["v"][0] <= 255


def test_auto_hsv_range_blue():
    cv2 = pytest.importorskip("cv2")
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(img, (100, 100), 60, (255, 0, 0), -1)  # 纯蓝 -> HSV H=120
    rng = auto_hsv_range(img, BLUE)
    assert rng is not None
    assert rng["h"][0] <= 120 <= rng["h"][1]


def test_auto_hsv_range_none_when_color_absent():
    img = np.zeros((200, 200, 3), dtype=np.uint8)  # 全黑，无红色
    assert auto_hsv_range(img, RED) is None


def test_draw_detection_visual_grasp_style():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    mask = np.zeros((100, 100), dtype=np.uint8)
    cv2 = pytest.importorskip("cv2")
    cv2.circle(mask, (50, 50), 20, 255, -1)
    dets = [ColorDetect("红色", 50.0, 50.0, 20.0, 1200.0, mask)]
    out = draw_detection(img, dets)
    assert not np.array_equal(out, img)  # 轮廓/框/十字/标签均有绘制
    assert out[50, 50].tolist() != [0, 0, 0]  # 质心十字处有颜色


def test_mask_view_white_on_black_with_marker():
    cv2 = pytest.importorskip("cv2")
    mask = np.zeros((100, 100), dtype=np.uint8)
    cv2.circle(mask, (50, 50), 20, 255, -1)
    dets = [ColorDetect("红色", 50.0, 50.0, 20.0, 1200.0, mask)]
    out = mask_view(mask, dets)
    assert out[55, 55].tolist() == [255, 255, 255]  # 匹配区域白色（避开中心十字）
    assert out[5, 5].tolist() == [0, 0, 0]          # 背景黑
    assert out[30, 50].tolist() != [0, 0, 0]        # 质心十字有绘制
