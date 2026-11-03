"""camera_demo 测试（FakeCap 注入；MJPEG 起停）。"""
from __future__ import annotations

import sys

import numpy as np
import pytest

from hand_control_core.camera_demo import FakeCap, main, run_camera

from test_app import FakeEngine


@pytest.fixture
def frames():
    return [np.zeros((120, 160, 3), dtype=np.uint8) for _ in range(3)]


def test_run_camera_processes_all_frames(make_config, frames):
    n = run_camera(make_config(), engine=FakeEngine(), cap=FakeCap(frames),
                   port=None)
    assert n == 3


def test_run_camera_zero_frames(make_config):
    n = run_camera(make_config(), engine=FakeEngine(), cap=FakeCap([]), port=None)
    assert n == 0


def test_run_camera_with_mjpeg(make_config, frames):
    import socket
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    n = run_camera(make_config(), engine=FakeEngine(), cap=FakeCap(frames),
                   port=port, stream_name="hand.mjpg")
    assert n == 3


def test_main_missing_config_returns_2(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["camera_demo", "--config",
                                      str(tmp_path / "nope.yaml")])
    assert main() == 2


def test_main_ros_topic_fail_returns_2(make_config, monkeypatch):
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

    monkeypatch.setattr("hand_control_core.camera_demo.Ros2Camera", _FailCam)
    cfg_path = make_config()
    monkeypatch.setattr(sys, "argv", ["camera_demo", "--config", str(cfg_path),
                                      "--ros", "--port", "0"])
    assert main() == 2
