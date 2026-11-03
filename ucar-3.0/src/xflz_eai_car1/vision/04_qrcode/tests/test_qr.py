"""qr 模块 TDD 测试（构建文档 02 的 6 条计划）。"""
import numpy as np
import pytest

from config import load_config
from qr import detect_qr, draw_qr, generate_qr

CFG_YAML = (
    "generate:\n"
    "  version: 1\n  error_correction: H\n  box_size: 5\n  border: 4\n"
    "  fill_color: [0, 0, 0]\n  back_color: [255, 255, 255]\n"
    "detect:\n  scale: 0.5\n"
    "camera:\n  index: 0\n  width: 640\n  height: 480\n"
    "display:\n  show: false\n  window_name: qrcode\n"
)


@pytest.fixture(scope="module")
def cfg(tmp_path_factory):
    p = tmp_path_factory.mktemp("cfg") / "qrcode_config.yaml"
    p.write_text(CFG_YAML, encoding="utf-8")
    return load_config(p)


def test_generate_qr_shape_and_type(cfg):
    img = generate_qr("hello", cfg)
    assert img.ndim == 3 and img.shape[2] == 3
    assert img.dtype == np.uint8
    # version=1(21 modules) + border*2=8 -> 29 格 × box_size 5 = 145
    assert img.shape[0] == img.shape[1] == 145


def test_roundtrip_generate_detect(cfg):
    text = "AIoT_B2 课程实验 04"
    img = generate_qr(text, cfg)
    results = detect_qr(img, scale=1.0)
    assert len(results) == 1
    assert results[0].text == text


def test_detect_noise_empty(cfg):
    rng = np.random.default_rng(0)
    noise = rng.integers(0, 255, (300, 300, 3), dtype=np.uint8)
    assert detect_qr(noise, scale=0.5) == []


def test_detect_scaled_coords_match(cfg):
    text = "scaled-test"
    img = generate_qr(text, cfg)
    # 放大 2 倍：框坐标也应约 2 倍
    big = np.zeros((img.shape[0] * 2, img.shape[1] * 2, 3), dtype=np.uint8)
    big[50:50 + img.shape[0], 50:50 + img.shape[1]] = img
    results = detect_qr(big, scale=0.5)
    assert len(results) == 1
    assert results[0].text == text
    assert results[0].x1 >= 45 and results[0].y1 >= 45  # 原图偏移 50 左右


def test_draw_qr_changes_pixels(cfg):
    img = generate_qr("draw", cfg)
    res = detect_qr(img, scale=1.0)
    out = draw_qr(img, res)
    assert out.shape == img.shape
    assert not np.array_equal(out, img)


def test_generate_empty_data_raises(cfg):
    with pytest.raises(ValueError, match="empty"):
        generate_qr("", cfg)
