"""reid 模块 TDD 测试（构建文档 04/05 的 4 条计划）。"""
import numpy as np
import pytest

from reid import FakeReID, ReIDBackend, preprocess_crop


def test_preprocess_crop_shape_and_norm():
    crop = np.full((80, 40, 3), 128, dtype=np.uint8)
    out = preprocess_crop(crop)
    assert out.shape == (1, 3, 128, 64)
    assert out.dtype == np.float32
    assert abs(out[0, 0, 10, 10] - (128 / 255 - 0.485) / 0.229) < 1e-4


def test_preprocess_crop_rgb_order():
    crop = np.zeros((80, 40, 3), dtype=np.uint8)
    crop[..., 0] = 255   # B 通道满值（R=G=0）
    out = preprocess_crop(crop)
    # BGR->RGB：out[0] = R = 0；out[2] = B = 1.0
    assert abs(out[0, 0, 0, 0] - (0.0 - 0.485) / 0.229) < 1e-4
    assert abs(out[0, 2, 0, 0] - (1.0 - 0.406) / 0.225) < 1e-4


def test_fake_reid_deterministic():
    r = FakeReID()
    crops = [np.full((80, 40, 3), 100, dtype=np.uint8),
             np.full((80, 40, 3), 200, dtype=np.uint8)]
    feats = r(crops)
    assert feats.shape == (2, 128)
    assert feats[0, 0] < feats[1, 0]
    assert np.array_equal(r([crops[0]])[0], r([crops[0]])[0])


def test_reid_backend_requires_load():
    r = ReIDBackend()
    with pytest.raises(RuntimeError, match="not loaded"):
        r([np.zeros((80, 40, 3), dtype=np.uint8)])
