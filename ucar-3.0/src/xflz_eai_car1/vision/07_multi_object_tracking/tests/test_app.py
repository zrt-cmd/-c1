"""app 模块 TDD 测试（构建文档 06 的 5 条计划）。"""
from __future__ import annotations

import http.client
from pathlib import Path

import numpy as np
import pytest

from config import load_config
from rknnpool import RKNNPool

REAL_CFG = Path(__file__).resolve().parents[1] / "config" / "track_config.yaml"


def _synthetic_det_outputs(imgsz=640):
    """仅 P3 cell(20,20) 一个 person(0) 检测（detect.py 同款）。"""
    g = imgsz // 8

    def z(shape):
        return np.zeros(shape, dtype=np.float32)

    outs = []
    for i, gg in enumerate((g, g // 2, g // 4)):
        b = z((1, 64, gg, gg))
        cl = z((1, 80, gg, gg))
        s = z((1, 1, gg, gg))
        if i == 0:
            for c in range(4):
                b[0, 16 * c + 2, 20, 20] = 100.0
            cl[0, 0, 20, 20] = 0.9
            s[0, 0, 20, 20] = 0.9
        outs += [b, cl, s]
    return outs


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
    server = MjpegServer("127.0.0.1", port, stream_name="track").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        server.set_frame(np.full((320, 240, 3), 50, dtype=np.uint8))
        conn = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
        conn.request("GET", "/track.mjpg")
        resp = conn.getresponse()
        head = resp.read(4096)
        conn.close()
        assert resp.status == 200
        assert b"\xff\xd8" in head  # JPEG SOI
    finally:
        server.stop()


def test_pipeline_tracks_across_frames():
    """FakeBackend 池 + FakeReID 全链路：2 帧同一目标 -> 1 条确认轨迹。"""
    from app import process_frame
    from backend import FakeBackend
    from reid import FakeReID
    from tracker import Tracker

    cfg = load_config(REAL_CFG)
    cfg["pool"]["workers"] = 2
    pool = RKNNPool(lambda tid: FakeBackend(lambda img: _synthetic_det_outputs(640)),
                    "dummy.rknn", 2, lambda b, f: b.predict(f))
    tracker = Tracker(max_iou_distance=0.7, max_age=10, n_init=1,
                      nn_budget=100, max_dist=0.2, mode="deepsort")
    reid = FakeReID()
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    try:
        for i in range(2):
            pool.put(frame)
            outputs, ok = pool.get(timeout=10)
            assert ok is True
            out, info = process_frame(frame, outputs, cfg, tracker, reid)
            assert out.shape == frame.shape
            if i == 1:  # 确认后才有 ID 标注
                assert not np.array_equal(out, frame)
            # 首帧轨迹 tentative（0 确认），次帧确认
            assert info["tracks"] == i
        assert all(t.track_id == 1 for t in tracker.tracks)
    finally:
        pool.release()
