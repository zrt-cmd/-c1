"""app 模块 TDD 测试（构建文档 03 的 5 条计划）。"""
from __future__ import annotations

import http.client
from pathlib import Path

import numpy as np

from config import load_config
from qr import detect_qr, generate_qr

REAL_CFG = Path(__file__).resolve().parents[1] / "config" / "qrcode_config.yaml"


def _gen(tmp_path, text="app-test"):
    cfg = load_config(REAL_CFG)
    img = generate_qr(text, cfg)
    p = tmp_path / "qr.png"
    import cv2
    cv2.imwrite(str(p), img)
    return p


def test_generate_via_main_saves_file(tmp_path, monkeypatch, capsys):
    import app

    out = tmp_path / "gen.png"
    monkeypatch.setattr("sys.argv", ["app.py", "--generate", "hello-app",
                                     "--save", str(out), "--config", str(REAL_CFG)])
    assert app.main() == 0
    assert out.exists()
    # 保存的图可被识别还原
    import cv2
    img = cv2.imread(str(out))
    results = detect_qr(img, scale=1.0)
    assert any(r.text == "hello-app" for r in results)


def test_detect_image_prints_text(tmp_path, monkeypatch, capsys):
    import app

    qr_path = _gen(tmp_path, text="print-me")
    monkeypatch.setattr("sys.argv", ["app.py", "--detect", "--image", str(qr_path),
                                     "--config", str(REAL_CFG)])
    assert app.main() == 0
    assert "print-me" in capsys.readouterr().out


def test_main_no_input_returns_2(monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(REAL_CFG)])
    assert app.main() == 2


def test_main_unopenable_image_returns_2(monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--detect", "--image", "/nonexistent.png",
                                     "--config", str(REAL_CFG)])
    assert app.main() == 2


def test_generate_empty_text_fails(tmp_path, monkeypatch):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--generate", "", "--save",
                                     str(tmp_path / "x.png"), "--config", str(REAL_CFG)])
    # 空字符串被 argparse 视为缺省值，走 print_help 分支
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
    import pytest
    import socket
    pytest.importorskip("mjpeg_streamer")
    from mjpeg_server import MjpegServer

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]  # 库不解析 port=0，须用具体空闲端口
    server = MjpegServer("127.0.0.1", port, stream_name="qrcode").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        rng = np.random.default_rng(0)
        server.set_frame(rng.integers(0, 255, (320, 240, 3), dtype=np.uint8))
        conn = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
        conn.request("GET", "/qrcode.mjpg")
        resp = conn.getresponse()
        head = resp.read(4096)
        conn.close()
        assert resp.status == 200
        assert b"\xff\xd8" in head  # JPEG SOI
    finally:
        server.stop()
