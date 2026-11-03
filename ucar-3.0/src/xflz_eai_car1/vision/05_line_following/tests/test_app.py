"""app 模块 TDD 测试（构建文档 03 的 5 条计划）。"""
from __future__ import annotations

import http.client
from pathlib import Path

import numpy as np

from config import load_config
from pipeline import detect_polygons

REAL_CFG = Path(__file__).resolve().parents[1] / "config" / "line_config.yaml"


def _square_img(size=200, side=80):
    img = np.full((size, size, 3), 255, dtype=np.uint8)
    img[60:60 + side, 60:60 + side] = (0, 0, 0)
    return img


def test_process_frame_annotates():
    from app import process_frame

    cfg = load_config(REAL_CFG)
    frame = _square_img()
    out = process_frame(frame, cfg)
    h, w = frame.shape[:2]
    # colored_mode=true → 双栏宽度 ×2；false → 原尺寸
    colored = (cfg.get("display") or {}).get("colored_mode", True)
    expected_w = w * 2 if colored else w
    assert out.shape == (h, expected_w, 3)
    assert out.dtype == np.uint8
    if colored:
        # 左栏 = 原图；右栏 = 黑底彩色（与原图不同）
        assert np.array_equal(out[:, :w], frame)
        assert not np.array_equal(out[:, w:], frame)
    else:
        assert not np.array_equal(out, frame)


def test_main_no_input_returns_2(monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(REAL_CFG)])
    assert app.main() == 2


def test_main_unopenable_image_returns_2(monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--image", "/nonexistent.png",
                                     "--config", str(REAL_CFG)])
    assert app.main() == 2


def test_main_unopenable_video_returns_2(monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--video", "/nonexistent.mp4",
                                     "--config", str(REAL_CFG)])
    assert app.main() == 2


def test_detect_on_real_config_square():
    cfg = load_config(REAL_CFG)
    results = detect_polygons(_square_img(), cfg)
    assert len(results) == 1
    assert results[0].n_vertices == 4


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
    import pytest
    import socket
    pytest.importorskip("mjpeg_streamer")
    from mjpeg_server import MjpegServer

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]  # 库不解析 port=0，须用具体空闲端口
    server = MjpegServer("127.0.0.1", port, stream_name="polygon").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        rng = np.random.default_rng(0)
        server.set_frame(rng.integers(0, 255, (320, 240, 3), dtype=np.uint8))
        conn = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
        conn.request("GET", "/polygon.mjpg")
        resp = conn.getresponse()
        head = resp.read(4096)
        conn.close()
        assert resp.status == 200
        assert b"\xff\xd8" in head  # JPEG SOI
    finally:
        server.stop()
