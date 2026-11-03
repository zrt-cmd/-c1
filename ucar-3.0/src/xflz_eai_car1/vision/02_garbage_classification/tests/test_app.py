"""app 模块 TDD 测试（构建文档 07 的 4 条计划）。"""
from __future__ import annotations

import numpy as np
import pytest

from app import make_process_fn
from detector import DetectorConfig, FakeBackend
from conftest import ANCHORS_9, CAT_MAP_20, CAT_NAMES, LABELS_20


def _cell_tensor(grid_h, grid_w, cell, cls_id=10, obj_raw=0.0, cls_raw=0.0):
    t = np.full((1, 75, grid_h, grid_w), -10.0, dtype=np.float32)
    r, c = cell
    t[0, 4, r, c] = obj_raw
    t[0, 5 + cls_id, r, c] = cls_raw
    return t


def _three_scales():
    return [_cell_tensor(4, 4, (0, 0), cls_id=10),
            np.full((1, 75, 2, 2), -10.0, dtype=np.float32),
            np.full((1, 75, 1, 1), -10.0, dtype=np.float32)]


def test_process_fn_returns_annotated_frame():
    cfg = DetectorConfig(imgsz=32, anchors=ANCHORS_9)
    fn = make_process_fn(LABELS_20, CAT_MAP_20, CAT_NAMES, cfg)
    frame = np.zeros((64, 32, 3), dtype=np.uint8)
    backend = FakeBackend(_three_scales())
    out = fn(backend, frame)
    assert isinstance(out, np.ndarray)
    assert out.shape == frame.shape
    assert out.dtype == np.uint8


def test_process_fn_no_detection_frame_unchanged():
    cfg = DetectorConfig(imgsz=32, anchors=ANCHORS_9)
    fn = make_process_fn(LABELS_20, CAT_MAP_20, CAT_NAMES, cfg)
    frame = np.zeros((32, 32, 3), dtype=np.uint8)
    outs = [np.full((1, 75, 4, 4), -10.0, dtype=np.float32),
            np.full((1, 75, 2, 2), -10.0, dtype=np.float32),
            np.full((1, 75, 1, 1), -10.0, dtype=np.float32)]
    out = fn(FakeBackend(outs), frame)
    assert out.shape == frame.shape


def test_main_no_source_returns_2(monkeypatch):
    import app
    from pathlib import Path

    real_cfg = Path(__file__).resolve().parents[1] / "config" / "garbage_config.yaml"
    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(real_cfg)])
    assert app.main() == 2


def test_main_unopenable_source_returns_2(tmp_path, monkeypatch):
    import app

    cfg = tmp_path / "garbage_config.yaml"
    cfg.write_text(
        "model:\n  path: nope.rknn\n  labels: nope.txt\n  category_map: nope.txt\n"
        "  category_names:\n    \"1\": a\n    \"2\": b\n    \"3\": c\n    \"4\": d\n"
        "infer:\n  imgsz: 32\n  preprocess: letterbox\n  score_thr: 0.25\n"
        "  nms_thr: 0.45\n  sigmoid: true\n  anchors: nope.txt\n"
        "pool:\n  tpes: 1\n  core_mask: auto\n"
        "server:\n  host: 127.0.0.1\n  port: 0\n  stream_name: s\n  quality: 80\n  fps: 30\n",
        encoding="utf-8")
    (tmp_path / "labels.txt").write_text("\n".join(LABELS_20), encoding="utf-8")
    (tmp_path / "category_map.txt").write_text("\n".join(str(v) for v in CAT_MAP_20),
                                               encoding="utf-8")
    (tmp_path / "anchors.txt").write_text("\n".join(f"{w},{h}" for w, h in ANCHORS_9),
                                          encoding="utf-8")
    text = cfg.read_text(encoding="utf-8")
    text = text.replace("labels: nope.txt", f"labels: {tmp_path}/labels.txt")
    text = text.replace("category_map: nope.txt", f"category_map: {tmp_path}/category_map.txt")
    text = text.replace("anchors: nope.txt", f"anchors: {tmp_path}/anchors.txt")
    cfg.write_text(text, encoding="utf-8")
    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(cfg),
                                     "--video", "/nonexistent.mp4"])
    assert app.main() == 2


def test_main_image_missing_returns_2(monkeypatch):
    import app
    from pathlib import Path

    real_cfg = Path(__file__).resolve().parents[1] / "config" / "garbage_config.yaml"
    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(real_cfg),
                                     "--image", "/nonexistent.jpg"])
    assert app.main() == 2


def test_main_image_saves_output(tmp_path, monkeypatch):
    import cv2
    import app
    from pathlib import Path

    real_cfg = Path(__file__).resolve().parents[1] / "config" / "garbage_config.yaml"
    src = tmp_path / "in.jpg"
    cv2.imwrite(str(src), np.zeros((32, 32, 3), dtype=np.uint8))
    out = tmp_path / "out.jpg"
    monkeypatch.setattr("app.rknn_backend_factory",
                        lambda cm: lambda tid: FakeBackend([np.zeros((1, 1, 1), np.uint8)]))
    monkeypatch.setattr("app.make_process_fn", lambda *a, **k: (lambda backend, frame: frame))
    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(real_cfg),
                                     "--image", str(src), "--save", str(out)])
    assert app.main() == 0
    assert out.exists()


def test_main_image_no_save_no_display_returns_2(tmp_path, monkeypatch):
    import cv2
    import app
    from pathlib import Path

    real_cfg = Path(__file__).resolve().parents[1] / "config" / "garbage_config.yaml"
    src = tmp_path / "in.jpg"
    cv2.imwrite(str(src), np.zeros((32, 32, 3), dtype=np.uint8))
    monkeypatch.setattr("app.rknn_backend_factory",
                        lambda cm: lambda tid: FakeBackend([np.zeros((1, 1, 1), np.uint8)]))
    monkeypatch.setattr("app.make_process_fn", lambda *a, **k: (lambda backend, frame: frame))
    monkeypatch.setattr("sys.argv", ["app.py", "--config", str(real_cfg),
                                     "--image", str(src)])
    assert app.main() == 2
