"""mjpeg_server 模块 TDD 测试（2026-08-06 起 mjpeg_streamer 库，无 stdlib 回退）。"""
import http.client
import socket
import sys

import numpy as np
import pytest

from mjpeg_server import INSTALL_HINT, MjpegServer, require


def _free_port() -> int:
    """预分配空闲端口（库不解析 port=0，须给具体端口）。"""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _wait_ready(host: str, port: int, timeout: float = 3.0) -> None:
    """等待 aiohttp 线程完成绑定（start() 是异步的，有连接竞态）。"""
    import time

    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return
        except OSError:
            time.sleep(0.05)
    raise TimeoutError(f"server not ready on {host}:{port}")


def _read_first_chunk(host, port, path, timeout=5.0, amount=4096):
    conn = http.client.HTTPConnection(host, port, timeout=timeout)
    conn.request("GET", path)
    resp = conn.getresponse()
    body = resp.read(amount)
    conn.close()
    return resp, body


def test_stream_returns_multipart_jpeg():
    pytest.importorskip("mjpeg_streamer")
    server = MjpegServer("127.0.0.1", _free_port(), stream_name="garbage_cam").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        rng = np.random.default_rng(0)
        frame = rng.integers(0, 255, (320, 240, 3), dtype=np.uint8)
        server.set_frame(frame)
        resp, body = _read_first_chunk("127.0.0.1", server.port, "/garbage_cam.mjpg")
        assert resp.status == 200
        assert resp.getheader("Content-Type").startswith("multipart/x-mixed-replace")
        assert b"\xff\xd8" in body  # JPEG SOI
    finally:
        server.stop()


def test_root_serves_streams_index():
    pytest.importorskip("mjpeg_streamer")
    server = MjpegServer("127.0.0.1", _free_port(), stream_name="garbage_cam").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        resp, body = _read_first_chunk("127.0.0.1", server.port, "/")
        assert resp.status == 200
        assert b"garbage_cam" in body
    finally:
        server.stop()


def test_unknown_path_404():
    pytest.importorskip("mjpeg_streamer")
    server = MjpegServer("127.0.0.1", _free_port(), stream_name="garbage_cam").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        conn = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
        conn.request("GET", "/nope.mjpg")
        resp = conn.getresponse()
        conn.close()
        assert resp.status == 404
    finally:
        server.stop()


def test_start_stop_does_not_crash():
    """stop() 会触发库的 GracefulExit（BaseException）——wrapper 必须吞掉。"""
    pytest.importorskip("mjpeg_streamer")
    server = MjpegServer("127.0.0.1", _free_port(), stream_name="garbage_cam").start()
    _wait_ready("127.0.0.1", server.port)
    server.set_frame(np.zeros((8, 8, 3), dtype=np.uint8))
    server.stop()  # 不抛异常即通过


def test_require_prompts_and_exits_when_missing(monkeypatch, capsys):
    # sys.modules[name]=None → import 抛 ImportError → require() 提示安装并 exit 2
    monkeypatch.setitem(sys.modules, "mjpeg_streamer", None)
    with pytest.raises(SystemExit) as exc:
        require()
    assert exc.value.code == 2
    assert "pip install" in capsys.readouterr().out
    assert "tuna.tsinghua" in INSTALL_HINT


def test_require_ok_when_installed():
    pytest.importorskip("mjpeg_streamer")
    require()  # 已安装 → 直接返回


def test_start_raises_when_port_busy():
    """端口被占用 → start() 明确报错（否则浏览器静默 404，难排查）。"""
    pytest.importorskip("mjpeg_streamer")
    import socket

    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.listen(1)
    try:
        server = MjpegServer("0.0.0.0", port, "lab_color")
        with pytest.raises(SystemExit, match="占用"):
            server.start()
    finally:
        s.close()
