"""demo_image / convert 契约测试（fake 引擎注入）。"""
from __future__ import annotations

import sys

import numpy as np
import pytest

from hand_control_core import convert_mediapipe
from hand_control_core.demo_image import demo_image, main as demo_main
from hand_control_core.hand import Hand, HandResult


class FakeEngine:
    def __init__(self, num=1):
        lm = np.zeros((21, 3), dtype=np.float32)
        lm[:, 1] = np.linspace(30, 90, 21)
        self.result = HandResult(
            hands=[Hand(landmarks=lm.copy(), score=0.9, handedness=0.8,
                        gesture_label="Victory", gesture_score=0.99)
                   for _ in range(num)])
        self.closed = False

    def process(self, frame):
        return self.result

    def close(self):
        self.closed = True


@pytest.fixture
def rgb_image(tmp_path):
    import cv2
    img = np.full((120, 160, 3), 60, dtype=np.uint8)
    p = tmp_path / "hand.jpg"
    cv2.imwrite(str(p), img)
    return p


def _cfg_with_rgb(make_config, rgb_path):
    return make_config({"source": {"rgb_path": str(rgb_path)}})


def test_demo_image_returns_composed(make_config, rgb_image):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    engine = FakeEngine()
    dst = demo_image(str(cfg_path), engine=engine)
    assert dst.ndim == 3 and dst.shape[1] == 2 * 160
    assert engine.closed


def test_demo_image_saves_file(make_config, rgb_image, tmp_path):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    out = tmp_path / "out.jpg"
    dst = demo_image(str(cfg_path), save=str(out), engine=FakeEngine())
    assert out.exists()
    import cv2
    assert cv2.imread(str(out)).shape == dst.shape


def test_demo_image_no_hand(make_config, rgb_image):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    dst = demo_image(str(cfg_path), engine=FakeEngine(num=0))
    assert dst.shape[1] == 2 * 160


def test_demo_image_missing_image_raises(make_config, tmp_path):
    cfg_path = make_config({"source": {"rgb_path": str(tmp_path / "nope.png")}})
    with pytest.raises(FileNotFoundError):
        demo_image(str(cfg_path), engine=FakeEngine())


def test_demo_image_missing_config_returns_2(make_config, tmp_path, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["demo_image", "--config",
                                      str(tmp_path / "nope.yaml")])
    assert demo_main() == 2


def test_convert_check_missing_model_returns_2(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["convert_mediapipe", "--check",
                                      "--workdir", str(tmp_path)])
    assert convert_mediapipe.main() == 2


def test_convert_expected_contract():
    det = convert_mediapipe.EXPECTED["hand_detector"]
    lm = convert_mediapipe.EXPECTED["hand_landmarks_detector"]
    assert det["input"] == [1, 192, 192, 3] and det["min_outputs"] == 2
    assert lm["input"] == [1, 224, 224, 3] and lm["min_outputs"] == 4
