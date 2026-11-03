"""config / view / backend 契约测试。"""
from __future__ import annotations

import numpy as np
import pytest
import yaml

from hand_control_core.backend import RKNNLiteBackend
from hand_control_core.config import ConfigError, load_config
from hand_control_core.view import add_fps, compose, put_status


def test_valid_config(make_config):
    cfg = load_config(make_config())
    assert cfg["model"]["detector_model"].endswith(".rknn")


def test_missing_model_raises(make_config):
    p = make_config().parent / "bad.yaml"
    p.write_text(yaml.safe_dump({"backend": {"core_mask": 0, "verbose": False}}))
    with pytest.raises(ConfigError, match="detector_model"):
        load_config(p)


def test_wrong_type_raises(make_config):
    with pytest.raises(ConfigError, match="key model.max_hands"):
        load_config(make_config({"model": {"max_hands": "two"}}))


def test_max_hands_range(make_config):
    with pytest.raises(ConfigError, match="max_hands"):
        load_config(make_config({"model": {"max_hands": 3}}))


def test_confidence_range(make_config):
    with pytest.raises(ConfigError, match="min_detection_confidence"):
        load_config(make_config({"model": {"min_detection_confidence": 1.5}}))


def test_mode_enum(make_config):
    with pytest.raises(ConfigError, match="source.mode"):
        load_config(make_config({"source": {"mode": "video"}}))


def test_port_range(make_config):
    with pytest.raises(ConfigError, match="server.port"):
        load_config(make_config({"server": {"port": 0}}))


def test_compose_dual_pane():
    left = np.zeros((100, 160, 3), dtype=np.uint8)
    right = np.full((80, 160, 3), 255, dtype=np.uint8)
    dst = compose(left, right)
    assert dst.shape == (100, 320, 3)


def test_put_status_hand_text():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    out = put_status(img, 12.0, 1, 3, "thresh")
    assert not np.array_equal(out, img)


def test_add_fps_red():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    assert np.any(add_fps(img, 15.0)[:, :, 2] > 0)


def test_rknnlite_constants_and_lazy():
    b = RKNNLiteBackend()
    assert b.NPU_CORE_AUTO == 0 and b.NPU_CORE_0 == 1
    assert b.NPU_CORE_1 == 2 and b.NPU_CORE_2 == 4
    assert b.NPU_CORE_0_1_2 == 7
    b.close()
