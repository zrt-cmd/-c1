"""app 模块 TDD 测试（构建文档 07 的 6 条计划）。"""
from __future__ import annotations

import http.client
from pathlib import Path

import numpy as np
import pytest

from config import load_config
from rknnpool import RKNNPool

REAL_CFG = Path(__file__).resolve().parents[1] / "config" / "seg_config.yaml"


def _synthetic_outputs(imgsz=640):
    """仅 P3 层 cell(20,20) 一个 person 检测：dfl dist=2、cls=0.9、mc=10、proto plane0=1。"""
    g = imgsz // 8

    def z(shape):
        return np.zeros(shape, dtype=np.float32)

    outs = []
    for i, gg in enumerate((g, g // 2, g // 4)):
        b = z((1, 64, gg, gg))
        cl = z((1, 80, gg, gg))
        s = z((1, 1, gg, gg))
        m = z((1, 32, gg, gg))
        if i == 0:
            for c in range(4):
                b[0, 16 * c + 2, 20, 20] = 100.0
            cl[0, 0, 20, 20] = 0.9
            s[0, 0, 20, 20] = 0.9
            m[0, 0, 20, 20] = 10.0
        outs += [b, cl, s, m]
    proto = z((1, 32, imgsz // 4, imgsz // 4))
    proto[0, 0] = 1.0
    outs.append(proto)
    return outs


def test_render_end_to_end():
    from app import render

    cfg = load_config(REAL_CFG)
    bgr = np.full((480, 640, 3), 100, dtype=np.uint8)
    out, info = render(bgr, _synthetic_outputs(640), cfg)
    assert info["det"] == 1
    # person 框 (148,148,180,180)@640；480x640 帧 letterbox pad_y=80 -> 原图行 68..100
    assert out[80, 160].tolist() == [100, 100, 100]   # 前景保留
    assert out[10, 10].tolist() == [0, 0, 0]          # pad 区域 -> 黑背景


def test_render_no_detection_full_background():
    from app import render

    cfg = load_config(REAL_CFG)
    bgr = np.full((480, 640, 3), 100, dtype=np.uint8)
    outs = _synthetic_outputs(640)
    for o in outs:
        o[...] = 0
    out, info = render(bgr, outs, cfg)
    assert info["det"] == 0
    assert out[10, 10].tolist() == [0, 0, 0]


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
    server = MjpegServer("127.0.0.1", port, stream_name="seg").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        server.set_frame(np.full((320, 240, 3), 50, dtype=np.uint8))
        conn = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
        conn.request("GET", "/seg.mjpg")
        resp = conn.getresponse()
        head = resp.read(4096)
        conn.close()
        assert resp.status == 200
        assert b"\xff\xd8" in head  # JPEG SOI
    finally:
        server.stop()


def test_pool_integration_fake_backend():
    """FakeBackend 池 + render 全链路：1 检测 + 合成输出。"""
    from app import render
    from backend import FakeBackend

    cfg = load_config(REAL_CFG)
    cfg["pool"]["workers"] = 2
    pool = RKNNPool(lambda tid: FakeBackend(lambda img: _synthetic_outputs(640)),
                    "dummy.rknn", 2, lambda b, f: b.predict(f))
    bgr = np.full((480, 640, 3), 100, dtype=np.uint8)
    pool.put(bgr)
    outputs, ok = pool.get(timeout=10)
    pool.release()
    assert ok is True
    out, info = render(bgr, outputs, cfg)
    assert info["det"] == 1
    assert out[80, 160].tolist() == [100, 100, 100]
    assert out[10, 10].tolist() == [0, 0, 0]
