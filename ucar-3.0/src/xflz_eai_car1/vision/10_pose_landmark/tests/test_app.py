"""demo_image / convert 契约测试（fake 引擎注入；转换检查无模型即报错）。"""
from __future__ import annotations

import sys

import numpy as np
import pytest

from pose_landmark_core.demo_image import demo_image, main as demo_main
from pose_landmark_core.pose import PoseResult
from pose_landmark_core import convert_mediapipe


class FakeEngine:
    """fake PoseEngine：固定结果，close 幂等。"""

    def __init__(self, detected=True):
        lm = np.zeros((33, 4), dtype=np.float32)
        if detected:
            lm[11] = (20, 30, 0.0, 0.9)
            lm[12] = (80, 30, 0.0, 0.9)
        self.result = PoseResult(landmarks=lm if detected else None, score=0.9)
        self.closed = False

    def process(self, frame):
        return self.result

    def close(self):
        self.closed = True


@pytest.fixture
def rgb_image(tmp_path):
    img = np.full((120, 160, 3), 60, dtype=np.uint8)
    p = tmp_path / "pose.jpg"
    import cv2
    cv2.imwrite(str(p), img)
    return p


def _cfg_with_rgb(make_config, rgb_path, **over):
    return make_config({"source": {"rgb_path": str(rgb_path)}, **over})


def test_demo_image_returns_composed(make_config, rgb_image):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    engine = FakeEngine()
    dst = demo_image(str(cfg_path), engine=engine)
    assert dst.ndim == 3 and dst.dtype == np.uint8
    assert dst.shape[1] == 2 * 160          # 双栏
    assert engine.closed


def test_demo_image_saves_file(make_config, rgb_image, tmp_path):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    out = tmp_path / "out.jpg"
    dst = demo_image(str(cfg_path), save=str(out), engine=FakeEngine())
    assert out.exists()
    import cv2
    reloaded = cv2.imread(str(out))
    assert reloaded.shape == dst.shape


def test_demo_image_no_pose(make_config, rgb_image):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    dst = demo_image(str(cfg_path), engine=FakeEngine(detected=False))
    assert dst.shape[1] == 2 * 160


def test_demo_image_missing_image_raises(make_config, tmp_path):
    cfg_path = make_config({"source": {"rgb_path": str(tmp_path / "nope.png")}})
    with pytest.raises(FileNotFoundError):
        demo_image(str(cfg_path), engine=FakeEngine())


def test_demo_image_missing_config_returns_2(make_config, tmp_path, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["demo_image", "--config",
                                      str(tmp_path / "nope.yaml")])
    assert demo_main() == 2


def test_demo_image_bad_image_returns_2(make_config, tmp_path, monkeypatch):
    cfg_path = make_config({"source": {"rgb_path": str(tmp_path / "nope.png")}})
    monkeypatch.setattr(sys, "argv", ["demo_image", "--config", str(cfg_path)])
    assert demo_main() == 2


def test_convert_check_missing_model_returns_2(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["convert_mediapipe", "--check",
                                      "--workdir", str(tmp_path)])
    assert convert_mediapipe.main() == 2


def test_convert_no_args_returns_2(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["convert_mediapipe"])
    assert convert_mediapipe.main() == 2


def test_convert_expected_contract():
    exp = convert_mediapipe.EXPECTED["pose_landmark_full"]
    assert exp["input"] == [1, 256, 256, 3]
    assert exp["min_outputs"] == 5
    assert exp["url"].endswith("pose_landmark_full.tflite")
