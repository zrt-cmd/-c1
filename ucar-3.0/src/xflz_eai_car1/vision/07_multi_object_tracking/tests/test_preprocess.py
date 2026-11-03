"""preprocess 模块 TDD 测试（构建文档 02 的 5 条计划）。"""
import pytest
import numpy as np

from preprocess import LetterboxResult, letterbox, prepare_input, to_rgb


def test_letterbox_square_pad_zero():
    img = np.zeros((320, 480, 3), dtype=np.uint8)
    img[:, :, 0] = 255  # 蓝色
    lb = letterbox(img, imgsz=640)
    assert lb.padded.shape == (640, 640, 3)
    # 宽图 480x320 -> 640x427（ratio 640/480=1.333，new_h=427）
    assert lb.ratio == pytest.approx(1.3333, abs=0.01)
    assert lb.pad_x == 0
    assert lb.pad_y == (640 - 427) // 2
    # pad 区域为 0
    assert lb.padded[0, 0].tolist() == [0, 0, 0]


def test_letterbox_tall_image_pad_x():
    img = np.zeros((480, 320, 3), dtype=np.uint8)
    lb = letterbox(img, imgsz=640)
    assert lb.pad_y == 0
    assert lb.pad_x == (640 - 427) // 2
    assert lb.padded.shape == (640, 640, 3)


def test_to_rgb_swaps_channels():
    bgr = np.zeros((10, 10, 3), dtype=np.uint8)
    bgr[..., 0] = 10   # B
    bgr[..., 1] = 20   # G
    bgr[..., 2] = 30   # R
    rgb = to_rgb(bgr)
    assert rgb[0, 0].tolist() == [30, 20, 10]
    assert rgb.dtype == np.uint8


def test_prepare_input_shape_and_dtype():

    img = np.zeros((300, 500, 3), dtype=np.uint8)
    inp = prepare_input(img, imgsz=640)
    assert inp.shape == (1, 640, 640, 3)
    assert inp.dtype == np.uint8


def test_unletterbox_inverse():
    img = np.zeros((320, 480, 3), dtype=np.uint8)
    lb = letterbox(img, imgsz=640)
    boxes = np.array([[0.0, 0.0, 480.0, 320.0, 0.9]])
    orig = lb.unletterbox(boxes)
    assert orig[0, 0] == pytest.approx(-lb.pad_x / lb.ratio, abs=0.5)
    assert orig[0, 2] == pytest.approx((480 - lb.pad_x) / lb.ratio, abs=0.5)
    assert isinstance(lb, LetterboxResult)


