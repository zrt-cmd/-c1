"""preprocess 模块 TDD 测试（构建文档 02 的 6 条计划）。"""
import numpy as np
import pytest

from preprocess import letterbox, prepare_input, to_rgb, unletterbox


def test_letterbox_small_to_640():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    lb = letterbox(img, 640)
    assert lb.padded.shape == (640, 640, 3)
    assert lb.scale == 1.0
    assert lb.pad_x == 0 and lb.pad_y == 80


def test_letterbox_portrait_scale():
    tall = np.zeros((960, 480, 3), dtype=np.uint8)
    lb = letterbox(tall, 640)
    assert lb.padded.shape == (640, 640, 3)
    assert lb.scale == 640 / 960
    assert lb.pad_x == 160 and lb.pad_y == 0


def test_letterbox_pad_color_zero():
    img = np.full((50, 100, 3), 255, dtype=np.uint8)
    lb = letterbox(img, 200)
    assert lb.pad_x == 0 and lb.pad_y == 50
    assert lb.padded[0, 0].tolist() == [0, 0, 0]          # 填充区 0（模型动物园约定）
    assert lb.padded[60, 10].tolist() == [255, 255, 255]  # 内容区原像素


def test_to_rgb_swaps_channels():
    img = np.zeros((4, 4, 3), dtype=np.uint8)
    img[..., 0] = 10   # B
    img[..., 1] = 20   # G
    img[..., 2] = 30   # R
    rgb = to_rgb(img)
    assert (rgb[..., 0] == img[..., 2]).all()   # R = 原 BGR 的 R
    assert (rgb[..., 2] == img[..., 0]).all()   # B = 原 BGR 的 B
    assert not np.shares_memory(rgb, img)       # 拷贝


def test_prepare_input_returns_rgb_with_meta():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[..., 2] = 200                              # 原图 R 通道亮
    inp, lb = prepare_input(img, 640)
    assert inp.shape == (640, 640, 3)
    assert inp.dtype == np.uint8
    assert (inp[80, 0, 0] == img[0, 0, 2]).all()   # 输入 R = 原 BGR 的 R（RGB 序）
    assert lb.scale == 1.0 and lb.pad_y == 80


def test_unletterbox_roundtrip():
    lb = letterbox(np.zeros((480, 640, 3), dtype=np.uint8), 640)
    boxes = np.array([[0.0, 80.0, 640.0, 560.0]])
    assert np.allclose(unletterbox(boxes, lb), [[0.0, 0.0, 640.0, 480.0]])


def test_unletterbox_scaled_with_pad():
    lb = letterbox(np.zeros((960, 480, 3), dtype=np.uint8), 640)
    assert lb.scale == 640 / 960 and lb.pad_x == 160
    orig = unletterbox(np.array([[160.0, 0.0, 426.0, 640.0]]), lb)
    assert orig[0, 0] == 0.0
    assert orig[0, 2] == pytest.approx((426.0 - 160.0) / lb.scale)
