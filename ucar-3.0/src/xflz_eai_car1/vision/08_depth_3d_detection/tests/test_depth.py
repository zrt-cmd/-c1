"""depth 模块 TDD 测试（构建文档 02 的 5 条计划）。"""
import numpy as np
import pytest

from depth import load_depth, normalize_depth, pseudocolor, read_depth

CAM = {"fx": 608.39958, "fy": 611.20714, "cx": 328.30096, "cy": 226.46483}


@pytest.fixture
def depth_map():
    d = np.zeros((480, 640), dtype=np.uint16)
    d[100:400, 100:500] = 1000
    d[240, 320] = 2000
    return d


def test_load_depth_roundtrip(tmp_path):
    import cv2

    d = np.zeros((64, 64), dtype=np.uint16)
    d[10:20, 10:20] = 1234
    p = tmp_path / "d.png"
    assert cv2.imwrite(str(p), d)
    loaded = load_depth(p)
    assert loaded.shape == (64, 64)
    assert loaded.dtype == np.uint16
    assert loaded[15, 15] == 1234


def test_load_depth_missing_raises():
    with pytest.raises(FileNotFoundError):
        load_depth("/nonexistent/depth.png")


def test_normalize_known_range():
    d = np.zeros((8, 8), dtype=np.uint16)
    d[0:2, 0:2] = 0      # 无效
    d[2:4, 2:4] = 500    # 最小值
    d[6:8, 6:8] = 1500   # 最大值
    norm = normalize_depth(d)
    assert norm[3, 3] == 0 and norm[7, 7] == 255
    assert norm[5, 5] == 0  # 无效区


def test_normalize_all_invalid_zero():
    norm = normalize_depth(np.zeros((4, 4), dtype=np.uint16))
    assert not norm.any()


def test_pseudocolor_shapes_and_modes():
    norm = np.zeros((64, 64), dtype=np.uint8)
    norm[10:50, 10:50] = 128
    jet = pseudocolor(norm, "jet")
    assert jet.shape == (64, 64, 3)
    gray = pseudocolor(norm, "gray")
    assert gray.shape == (64, 64, 3)
    assert np.array_equal(gray[..., 0], gray[..., 1])
    # 无效区(0)与有效区(128)颜色不同
    assert not np.array_equal(jet[0, 0], jet[20, 20])


def test_read_depth_valid_and_backproject():
    d = np.zeros((480, 640), dtype=np.uint16)
    d[240, 320] = 1000
    r = read_depth(d, 320, 240, scale=0.001, cam=CAM)
    assert r.z_mm == 1000 and r.z_m == pytest.approx(1.0)
    # x = (320-328.30096)*1.0/608.39958
    assert r.xyz[0] == pytest.approx((320 - 328.30096) / 608.39958, abs=1e-4)
    assert r.xyz[2] == pytest.approx(1.0)


def test_read_depth_invalid_and_out_of_bounds():
    d = np.zeros((480, 640), dtype=np.uint16)
    r = read_depth(d, 10, 10, cam=CAM)   # 无效 0
    assert r.z_mm == 0 and r.xyz is None
    r2 = read_depth(d, 9999, 9999, cam=CAM)  # 越界
    assert r2.z_mm == 0 and r2.xyz is None
