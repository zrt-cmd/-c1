"""background 模块 TDD 测试（构建文档 04 的 4 条计划）。"""
import numpy as np
import pytest

from background import compose, load_background


def _cfg(mode, color=(0, 0, 0), image=""):
    return {"background": {"mode": mode, "color": color, "image": image}}


def _img_and_mask(h=32, w=48):
    img = np.full((h, w, 3), 100, dtype=np.uint8)
    mask = np.zeros((h, w), dtype=bool)
    mask[8:24, 8:40] = True
    return img, mask


def test_black_mode():
    img, mask = _img_and_mask()
    out = compose(img, mask, _cfg("black"))
    assert out[10, 10].tolist() == [100, 100, 100]   # 掩码内保留
    assert out[2, 2].tolist() == [0, 0, 0]           # 掩码外置黑


def test_solid_mode():
    img, mask = _img_and_mask()
    out = compose(img, mask, _cfg("solid", color=[10, 20, 30]))
    assert out[10, 10].tolist() == [100, 100, 100]
    assert out[2, 2].tolist() == [10, 20, 30]


def test_image_mode(tmp_path):
    img, mask = _img_and_mask()
    bg = np.full((16, 24, 3), 200, dtype=np.uint8)  # 小背景图 -> resize 铺满
    bg_path = tmp_path / "bg.jpg"
    assert cv2_imwrite(str(bg_path), bg)
    out = compose(img, mask, _cfg("image", image=bg_path.name), base_dir=tmp_path)
    assert out[10, 10].tolist() == [100, 100, 100]   # 前景保留
    assert out[2, 2].tolist() == [200, 200, 200]     # 背景图缩放后铺满


def cv2_imwrite(path, img):
    import cv2
    return cv2.imwrite(path, img)


def test_mask_size_mismatch_raises():
    img = np.zeros((32, 32, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="mask shape"):
        compose(img, np.zeros((16, 16), dtype=bool), _cfg("black"))


def test_missing_background_raises(tmp_path):
    img, mask = _img_and_mask()
    with pytest.raises(ValueError, match="cannot read"):
        compose(img, mask, _cfg("image", image=str(tmp_path / "nope.jpg")))


def test_load_background_direct():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    import cv2
    p = "/tmp/exp06_bg_test.jpg"
    cv2.imwrite(p, img)
    out = load_background(p, (48, 32))
    assert out.shape == (32, 48, 3)
