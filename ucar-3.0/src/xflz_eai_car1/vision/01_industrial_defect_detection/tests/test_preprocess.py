"""preprocess 模块 TDD 测试。"""
import numpy as np
import pytest

from preprocess import letterbox, to_rgb, unletterbox


def test_letterbox_small_to_640(sample_img):
    lb = letterbox(sample_img, 640)
    assert lb.padded.shape == (640, 640, 3)
    assert lb.scale == 1.0
    assert lb.pad_x == 0 and lb.pad_y == 80


def test_letterbox_portrait_scale():
    # 960x480 竖图 -> 640：宽向缩到 320，pad_x=160，pad_y=0
    tall = np.zeros((960, 480, 3), dtype=np.uint8)
    lb = letterbox(tall, 640)
    assert lb.padded.shape == (640, 640, 3)
    assert lb.scale == 640 / 960
    assert lb.pad_x == 160 and lb.pad_y == 0


def test_letterbox_pad_color_default_zero():
    # 100x50 非方形图 -> 200x200：scale=2 -> new 200x100，pad_x=0, pad_y=50
    img = np.full((50, 100, 3), 255, dtype=np.uint8)
    lb = letterbox(img, 200)
    assert lb.pad_x == 0 and lb.pad_y == 50
    assert lb.padded[0, 0].tolist() == [0, 0, 0]          # 填充区为 0（rknn_model_zoo 约定）
    assert lb.padded[60, 10].tolist() == [255, 255, 255]  # 内容区为原像素
def test_letterbox_content_preserved(sample_img):
    lb = letterbox(sample_img, 640)
    # 480x640 -> pad_y=80：内容完整落在填充区，未变形
    assert np.allclose(lb.padded[80, 0], sample_img[0, 0])
    assert np.allclose(lb.padded[559, 639], sample_img[479, 639])


def test_unletterbox_roundtrip():
    lb = letterbox(np.zeros((480, 640, 3), dtype=np.uint8), 640)
    boxes = np.array([[0.0, 80.0, 640.0, 560.0]])   # letterbox 空间全图框
    orig = unletterbox(boxes, lb)
    assert np.allclose(orig, [[0.0, 0.0, 640.0, 480.0]])


def test_unletterbox_scaled():
    lb = letterbox(np.zeros((960, 480, 3), dtype=np.uint8), 640)
    assert lb.scale == 640 / 960
    assert lb.pad_x == 160 and lb.pad_y == 0
    orig = unletterbox(np.array([[160.0, 0.0, 426.0, 640.0]]), lb)
    assert orig[0, 0] == 0.0
    assert orig[0, 2] == pytest.approx((426.0 - 160.0) / lb.scale)


def test_to_rgb_swaps_channels():
    img = np.zeros((4, 4, 3), dtype=np.uint8)
    img[..., 0] = 10   # B
    img[..., 1] = 20   # G
    img[..., 2] = 30   # R
    rgb = to_rgb(img)
    assert (rgb[..., 0] == img[..., 2]).all()   # R = 原 BGR 的 R
    assert (rgb[..., 2] == img[..., 0]).all()   # B = 原 BGR 的 B
    assert not np.shares_memory(rgb, img)       # 必须拷贝（不改原帧）
