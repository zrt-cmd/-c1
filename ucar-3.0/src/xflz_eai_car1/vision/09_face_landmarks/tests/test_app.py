"""demo_image / mjpeg_server 测试（fake 引擎注入，无 mediapipe）。"""
from __future__ import annotations

import sys

import cv2
import numpy as np
import pytest

from face_landmark_core.demo_image import demo_image, main as demo_main
from face_landmark_core.landmark import LandmarkResult
from face_landmark_core.mjpeg_server import MjpegServer


class FakeEngine:
    def __init__(self, face_count=1, points=468):
        self._face_count = face_count
        self._points = points
        self.closed = False

    def process(self, frame_bgr, timestamp_ms):
        rng = np.random.default_rng(1)
        faces = []
        for _ in range(self._face_count):
            pts = np.zeros((self._points, 3), dtype=np.float32)
            pts[:, 0] = rng.random(self._points)
            pts[:, 1] = rng.random(self._points)
            faces.append(pts)
        return LandmarkResult(faces=faces)

    def close(self):
        self.closed = True


@pytest.fixture
def rgb_image(tmp_path):
    img = np.full((120, 160, 3), 60, dtype=np.uint8)
    p = tmp_path / "face.png"
    cv2.imwrite(str(p), img)
    return p


def _cfg_with_rgb(make_config, rgb_path, **over):
    return make_config({"source": {"rgb_path": str(rgb_path)}, **over})


def test_demo_image_returns_composed(make_config, rgb_image):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    engine = FakeEngine(face_count=1)
    dst = demo_image(str(cfg_path), engine=engine)
    assert dst.shape == (120, 320, 3)  # 双栏
    assert np.any(dst > 60)  # 网格已绘制
    assert engine.closed


def test_demo_image_saves_file(make_config, rgb_image, tmp_path):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    out = tmp_path / "out.jpg"
    dst = demo_image(str(cfg_path), save=str(out), engine=FakeEngine(face_count=2))
    assert out.exists()
    reloaded = cv2.imread(str(out))
    assert reloaded.shape == dst.shape


def test_demo_image_no_faces(make_config, rgb_image):
    cfg_path = _cfg_with_rgb(make_config, rgb_image)
    dst = demo_image(str(cfg_path), engine=FakeEngine(face_count=0))
    assert dst.shape == (120, 320, 3)
    # 状态栏 FPS: 0 Faces: 0 有文字（底部区域非零）
    assert np.any(dst[96:, :] > 0)


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


def _wait_ready(host: str, port: int, timeout: float = 3.0) -> None:
    """等待 aiohttp 线程完成绑定（start() 异步，避免连接竞态）。"""
    import socket
    import time

    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return
        except OSError:
            time.sleep(0.05)
    raise TimeoutError(f"server not ready on {host}:{port}")


def test_mjpeg_server_start_stop_and_stream():
    import socket
    pytest.importorskip("mjpeg_streamer")
    import urllib.request

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]  # 库不解析 port=0，须用具体空闲端口
    srv = MjpegServer(port=port, stream_name="face.mjpg")
    srv.start()
    _wait_ready("127.0.0.1", srv.port)
    try:
        srv.set_frame(np.zeros((10, 20, 3), dtype=np.uint8))
        with urllib.request.urlopen(f"http://127.0.0.1:{srv.port}/face.mjpg",
                                    timeout=5) as resp:
            assert resp.status == 200
            chunk = resp.read(2048)
        assert b"\xff\xd8" in chunk  # multipart 内含 JPEG SOI
    finally:
        srv.stop()


def test_mjpeg_server_serves_root_index():
    import socket
    pytest.importorskip("mjpeg_streamer")
    import urllib.request

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    srv = MjpegServer(port=port, stream_name="face.mjpg")
    srv.start()
    _wait_ready("127.0.0.1", srv.port)
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{srv.port}/",
                                    timeout=5) as resp:
            assert resp.status == 200
            assert b"face.mjpg" in resp.read(2048)  # 库的 streams index 页
    finally:
        srv.stop()


def test_mjpeg_server_unknown_path_404():
    import socket
    pytest.importorskip("mjpeg_streamer")
    import urllib.error
    import urllib.request

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    srv = MjpegServer(port=port, stream_name="face.mjpg")
    srv.start()
    _wait_ready("127.0.0.1", srv.port)
    try:
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(f"http://127.0.0.1:{srv.port}/nope.jpg",
                                   timeout=5)
    finally:
        srv.stop()
