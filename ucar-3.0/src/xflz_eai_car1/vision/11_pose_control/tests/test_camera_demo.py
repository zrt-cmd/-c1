"""camera_demo 测试（FakeCap 注入；MJPEG 起停）。"""
from __future__ import annotations

import sys

import numpy as np
import pytest

from pose_control_core.camera_demo import FakeCap, main, run_camera
from pose_control_core.pose import PoseResult

from test_app import FakeEngine


@pytest.fixture
def frames():
    return [np.zeros((120, 160, 3), dtype=np.uint8) for _ in range(3)]


def test_run_camera_processes_all_frames(make_config, frames):
    cfg = make_config()
    engine = FakeEngine()
    n = run_camera(cfg, engine=engine, cap=FakeCap(frames), port=None)
    assert n == 3
    assert engine.closed


def test_run_camera_zero_frames(make_config):
    engine = FakeEngine()
    n = run_camera(make_config(), engine=engine, cap=FakeCap([]), port=None)
    assert n == 0


def test_run_camera_with_mjpeg(make_config, frames, tmp_path):
    import socket
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    engine = FakeEngine()
    n = run_camera(make_config(), engine=engine, cap=FakeCap(frames),
                   port=port, stream_name="pose.mjpg")
    assert n == 3


def test_run_camera_display_off_by_default(make_config, frames):
    # 无 display、无 mjpeg 也能跑完
    n = run_camera(make_config(), engine=FakeEngine(), cap=FakeCap(frames), port=None)
    assert n == 3


def test_main_missing_config_returns_2(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["camera_demo", "--config",
                                      str(tmp_path / "nope.yaml")])
    assert main() == 2


def test_main_ros_topic_fail_returns_2(make_config, tmp_path, monkeypatch):
    # --ros 但话题无数据（无驱动）→ 2；注入假 Ros2Camera 保证确定性
    class _FailCam:
        DEFAULT_TOPIC = "/fake/topic"

        def __init__(self, *a, **k):
            pass
        def open(self):
            pass
        def read(self, timeout=5.0):
            return False, None
        def release(self):
            pass
    monkeypatch.setattr("pose_control_core.camera_demo.Ros2Camera", _FailCam)
    cfg_path = make_config()
    monkeypatch.setattr(sys, "argv", ["camera_demo", "--config", str(cfg_path),
                                      "--ros", "--port", "0"])
    assert main() == 2
