"""app 模块 TDD 测试（构建文档 03 的 4 条计划）。"""
from __future__ import annotations

import http.client
from pathlib import Path

import numpy as np
import pytest

from app import process_frame
from config import load_colors, load_config

REAL_CFG = Path(__file__).resolve().parents[1] / "config" / "lab_config.yaml"


def _red_circle_img(size=200, radius=40):
    import cv2
    img = np.full((size, size, 3), 200, dtype=np.uint8)
    cv2.circle(img, (size // 2, size // 2), radius, (0, 0, 255), -1)
    return img


def test_process_frame_annotates():
    cfg = load_config(REAL_CFG)
    colors = load_colors(cfg)
    frame = _red_circle_img()
    out = process_frame(frame, cfg, colors)
    assert out.shape == (frame.shape[0], frame.shape[1] * 2 + 2, 3)  # 左RGB | 右Mask
    assert out.dtype == np.uint8
    assert not np.array_equal(out, frame)  # 有标注（红圆检出 + 名称）


def test_draw_detection_smoke():
    import numpy as np
    from preprocess import ColorDetect, draw_detection

    img = np.zeros((100, 100, 3), dtype=np.uint8)
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[30:70, 30:70] = 255
    dets = [ColorDetect("红色", 50.0, 50.0, 20.0, 1200.0, mask)]
    out = draw_detection(img, dets)
    assert out.shape == img.shape


def test_main_no_source_returns_2(monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(REAL_CFG)])
    assert app.main() == 2


def test_main_unopenable_image_returns_2(monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--image", "/nonexistent.jpg",
                                     "--config", str(REAL_CFG)])
    assert app.main() == 2


def test_main_unopenable_video_returns_2(monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--video", "/nonexistent.mp4",
                                     "--config", str(REAL_CFG)])
    assert app.main() == 2


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


def test_mjpeg_server_smoke():
    import socket
    pytest.importorskip("mjpeg_streamer")
    from mjpeg_server import MjpegServer

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]  # 库不解析 port=0，须用具体空闲端口
    server = MjpegServer("127.0.0.1", port, stream_name="lab_color").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        rng = np.random.default_rng(0)
        server.set_frame(rng.integers(0, 255, (320, 240, 3), dtype=np.uint8))
        conn = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
        conn.request("GET", "/lab_color.mjpg")
        resp = conn.getresponse()
        head = resp.read(4096)
        conn.close()
        assert resp.status == 200
        assert resp.getheader("Content-Type").startswith("multipart/x-mixed-replace")
        assert b"\xff\xd8" in head  # JPEG SOI
    finally:
        server.stop()


def test_dual_view_stacks_with_separator():
    from app import dual_view

    bgr = np.zeros((10, 20, 3), dtype=np.uint8)
    out = np.full((10, 20, 3), 255, dtype=np.uint8)
    view = dual_view(bgr, out, sep=2)
    assert view.shape == (10, 42, 3)  # 20 + 2 绿分隔线 + 20
    assert int(view[0, 20, 1]) == 255  # 分隔线绿色
    assert int(view[0, 0, 0]) == 0     # 左原图
    assert int(view[0, 30, 0]) == 255  # 右处理图


def test_tune_unknown_color_prints_and_returns(capsys):
    from app import _tune_mode

    _tune_mode("不存在色", [], None, cap=None)
    assert "不在配置中" in capsys.readouterr().out
