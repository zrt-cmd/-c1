"""camera_demo 测试（fake cap + fake engine 注入）。"""
from __future__ import annotations

import sys

import numpy as np
from face_landmark_core.blaze import AnchorOptions, DetectConfig, generate_anchors
from face_landmark_core.backend import FakeBackend
from face_landmark_core.config import load_config
from face_landmark_core.camera_demo import FakeCap, main as cam_main, run_camera
from face_landmark_core.landmark import FaceLandmarkerEngine


def _engine():
    def det(img):
        cfg = DetectConfig()
        anchors = generate_anchors(AnchorOptions(input_size=128))
        reg = np.zeros((896, 16), dtype=np.float32)
        cls = np.full((896, 1), -100.0, dtype=np.float32)
        a = anchors[0]
        reg[0, 0] = (0.5 - a[0]) * cfg.x_scale
        reg[0, 1] = (0.5 - a[1]) * cfg.y_scale
        reg[0, 2] = 0.2 * cfg.w_scale
        reg[0, 3] = 0.2 * cfg.h_scale
        cls[0] = 5.0
        return reg, cls

    def lm(img):
        pts = np.zeros((1, 1, 1, 1404), dtype=np.float32)
        pts[0, 0, 0, 0::3] = 96.0   # ROI 192 像素坐标（模型语义）
        pts[0, 0, 0, 1::3] = 96.0
        return [pts, np.array([[[[15.0]]]], dtype=np.float32)]

    return FaceLandmarkerEngine(
        "det.rknn", "lm.rknn", num_faces=2,
        create_fn=lambda: (FakeBackend(fn=det), FakeBackend(fn=lm)),
    )


def test_fake_cap_iterates():
    cap = FakeCap([np.zeros((10, 10, 3), np.uint8)] * 3)
    n = 0
    while True:
        ret, _ = cap.read()
        if not ret:
            break
        n += 1
    assert n == 3
    assert not cap.opened


def test_run_camera_processes_all_frames(make_config):
    cfg = load_config(make_config())
    cap = FakeCap([np.zeros((120, 160, 3), np.uint8)] * 5)
    n = run_camera(cfg, engine=_engine(), cap=cap, display=False, port=0)
    assert n == 5


def test_run_camera_zero_frames(make_config):
    cfg = load_config(make_config())
    cap = FakeCap([])
    n = run_camera(cfg, engine=_engine(), cap=cap, display=False, port=0)
    assert n == 0


def test_run_camera_with_mjpeg(make_config):
    cfg = load_config(make_config())
    cap = FakeCap([np.zeros((120, 160, 3), np.uint8)] * 3)
    n = run_camera(cfg, engine=_engine(), cap=cap, display=False, port=0)
    assert n == 3  # port=0 → MJPEG 关闭


def test_main_missing_config_returns_2(make_config, tmp_path, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["camera_demo", "--config",
                                      str(tmp_path / "nope.yaml")])
    assert cam_main() == 2

def test_main_camera_index_passthrough(make_config, monkeypatch):
    seen = {}
    monkeypatch.setattr("face_landmark_core.camera_demo.run_camera",
                        lambda cfg, **kw: seen.update(kw) or 0)
    cfg_path = make_config()
    monkeypatch.setattr(sys, "argv", ["camera_demo", "--config", str(cfg_path),
                                      "--camera", "25", "--port", "0"])
    assert cam_main() == 0
    assert seen.get("camera_index") == 25
    assert seen.get("port") is None  # --port 0 → 关闭 MJPEG
