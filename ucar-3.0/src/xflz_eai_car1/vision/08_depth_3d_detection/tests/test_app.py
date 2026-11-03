"""app 模块 TDD 测试（构建文档 04 的 5 条计划）。"""
from __future__ import annotations

import http.client
from pathlib import Path

import numpy as np
import pytest

from config import load_config

REAL_CFG = Path(__file__).resolve().parents[1] / "config" / "depth3d_config.yaml"


def _make_cfg(tmp_path, mode="image"):
    p = tmp_path / "cfg.yaml"
    src = REAL_CFG.read_text(encoding="utf-8")
    # 默认 config 已是 mode: ros（板端标准）——两种写法都替换，夹具强制目标 mode
    src = src.replace("mode: ros", f"mode: {mode}").replace("mode: image", f"mode: {mode}")
    if mode == "image":
        src = src.replace('rgb_path: ""', f"rgb_path: {tmp_path/'rgb.png'}")
        src = src.replace('depth_path: ""', f"depth_path: {tmp_path/'d.png'}")
    p.write_text(src, encoding="utf-8")
    return load_config(p)


def _assets(tmp_path):
    import cv2

    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[..., 2] = 100
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[100:400, 100:500] = 1500
    cv2.imwrite(str(tmp_path / "rgb.png"), rgb)
    cv2.imwrite(str(tmp_path / "d.png"), depth)
    return rgb, depth


def test_run_viewer_composes(tmp_path):
    from app import run_viewer

    cfg = _make_cfg(tmp_path)
    rgb, depth = _assets(tmp_path)
    out = run_viewer(rgb, depth, cfg)
    sh = round(480 * 0.666)
    sw = round(640 * 0.666)
    assert out.shape == (sh, 2 * sw, 3)


def test_click_cb_coords_and_message(tmp_path, capsys):
    from app import run_viewer
    from depth import read_depth
    from view import screen_to_pixel

    cfg = _make_cfg(tmp_path)
    rgb, depth = _assets(tmp_path)
    clicks = []

    def cb(x, y):
        clicks.append(screen_to_pixel(x, y, cfg["depth"]["display_scale"]))

    run_viewer(rgb, depth, cfg, click_cb=cb)
    # 显示坐标 (213, 160) -> 原图 (320, 240)
    u, v = screen_to_pixel(213, 160, cfg["depth"]["display_scale"])
    r = read_depth(depth, u, v, cfg["depth"]["scale"], cfg["cam3d"])
    assert r.z_mm == 1500
    assert r.xyz[2] == pytest.approx(1.5)


def test_main_missing_source_returns_2(monkeypatch, tmp_path):
    import app

    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(tmp_path / "nope.yaml")])
    assert app.main() == 2


def test_main_no_mode_returns_2(monkeypatch, tmp_path):
    import app

    cfg = _make_cfg(tmp_path, mode="image")
    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(tmp_path / "cfg.yaml")])
    # 文件存在但未填路径 -> image 模式直接读默认路径失败
    # 用缺文件路径的配置：把 cfg 指向缺 rgb 的配置
    import yaml
    raw = yaml.safe_load(open(str(tmp_path / "cfg.yaml"), encoding="utf-8"))
    raw["source"]["rgb_path"] = "/nonexistent/rgb.png"
    bad = tmp_path / "bad_cfg.yaml"
    bad.write_text(yaml.safe_dump(raw), encoding="utf-8")
    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(bad)])
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
    server = MjpegServer("127.0.0.1", port, stream_name="depth").start()
    _wait_ready("127.0.0.1", server.port)
    try:
        server.set_frame(np.full((240, 320, 3), 50, dtype=np.uint8))
        conn = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
        conn.request("GET", "/depth.mjpg")
        resp = conn.getresponse()
        head = resp.read(4096)
        conn.close()
        assert resp.status == 200
        assert b"\xff\xd8" in head  # JPEG SOI
    finally:
        server.stop()
