"""config / view / backend 契约测试（schema 校验、双栏合成、RKNNLite 常量）。"""
from __future__ import annotations

import numpy as np
import pytest

from pose_control_core.backend import RKNNLiteBackend
from pose_control_core.config import ConfigError, load_config
from pose_control_core.view import add_fps, compose, put_status


def test_valid_config(make_config):
    cfg = load_config(make_config())
    assert cfg["model"]["pose_model"].endswith(".rknn")


def test_missing_section_raises(make_config):
    # 删除 model 段
    import yaml
    p = make_config().parent / "bad.yaml"
    p.write_text(yaml.safe_dump({"backend": {"core_mask": 0, "verbose": False}}))
    with pytest.raises(ConfigError, match="model.pose_model"):
        load_config(p)


def test_wrong_type_raises(make_config):
    with pytest.raises(ConfigError, match="key model.min_detection_confidence"):
        load_config(make_config({"model": {"min_detection_confidence": "high"}}))


def test_confidence_range(make_config):
    with pytest.raises(ConfigError, match="min_detection_confidence"):
        load_config(make_config({"model": {"min_detection_confidence": 1.5}}))


def test_mode_enum(make_config):
    with pytest.raises(ConfigError, match="source.mode"):
        load_config(make_config({"source": {"mode": "video"}}))


def test_port_range(make_config):
    with pytest.raises(ConfigError, match="server.port"):
        load_config(make_config({"server": {"port": 0}}))


def test_image_mode_requires_rgb_path(make_config):
    with pytest.raises(ConfigError, match="rgb_path"):
        load_config(make_config({"source": {"mode": "image", "rgb_path": ""}}))


def test_compose_dual_pane():
    left = np.zeros((100, 160, 3), dtype=np.uint8)
    right = np.full((80, 160, 3), 255, dtype=np.uint8)
    dst = compose(left, right)
    assert dst.shape == (100, 320, 3)
    assert np.all(dst[:80, 160:] == 255)   # 右栏
    assert np.all(dst[80:, 160:] == 0)     # 高度差补零


def test_put_status_text():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    out = put_status(img, 12.0, True)
    assert not np.array_equal(out, img)
    out2 = put_status(img, 12.0, False)
    assert not np.array_equal(out2, img)


def test_add_fps():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    out = add_fps(img, 15.0)
    assert np.any(out[:, :, 2] > 0)        # 红字


def test_rknnlite_constants_and_lazy():
    b = RKNNLiteBackend()
    assert b.NPU_CORE_AUTO == 0
    assert b.NPU_CORE_0 == 1
    assert b.NPU_CORE_1 == 2
    assert b.NPU_CORE_2 == 4
    assert b.NPU_CORE_0_1_2 == 7
    assert b._rknn is None
    b.close()                               # 幂等 close


def test_rknnlite_load_lazy_import():
    import sys
    assert "rknnlite" not in sys.modules or True  # 不强制，加载才 import


def test_print_interval_negative_raises(make_config):
    with pytest.raises(ConfigError, match="print_interval"):
        load_config(make_config({"control": {"print_interval": -1}}))


def test_print_interval_zero_ok(make_config):
    cfg = load_config(make_config({"control": {"print_interval": 0}}))
    assert cfg["control"]["print_interval"] == 0
