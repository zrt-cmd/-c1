"""infer 模块 TDD 测试：Detector 编排 + 后端协议（FakeBackend 注入）。"""
import numpy as np
import pytest

from infer import Detector, DetectorConfig, FakeBackend
from postprocess import Detection


def _synthetic_outputs(grid=4, nc=2, high_cells=None):
    """单层输出：默认 grid(0,0) 类0 0.9 分。"""
    high_cells = high_cells or {(0, 0): 0.9}
    box = np.zeros((1, 64, grid, grid), dtype=np.float32)
    cls = np.full((1, nc, grid, grid), 0.05, dtype=np.float32)
    for (r, c), s in high_cells.items():
        cls[0, 0, r, c] = s
    return [box, cls, np.ones((1, 1, grid, grid), dtype=np.float32)]


def test_detector_load_calls_backend():
    backend = FakeBackend(_synthetic_outputs())
    det = Detector(backend, num_classes=2).load("model.rknn")
    assert backend.loaded == "model.rknn"


def test_detector_detect_returns_original_coords():
    # 输入 64x32，imgsz=32：scale=min(32/64,32/32)=0.5 -> new(32x16)，pad_x=8, pad_y=0
    # letterbox 空间 grid(0,0) 框 = (-56,-56,64,64) -> 原图 = (box-pad)/scale
    img = np.zeros((64, 32, 3), dtype=np.uint8)
    backend = FakeBackend(_synthetic_outputs(grid=4, nc=2))
    det = Detector(backend, num_classes=2, cfg=DetectorConfig(imgsz=32))
    dets = det.detect(img)
    assert len(dets) == 1
    d = dets[0]
    scale = 0.5
    assert d.x1 == (-56 - 8) / scale
    assert d.y1 == (-56 - 0) / scale
    assert d.x2 == (64 - 8) / scale
    assert d.y2 == (64 - 0) / scale
    assert d.class_id == 0
    assert d.score == pytest.approx(0.9)
    assert isinstance(d, Detection)


def test_detector_adjacent_cells_nms_suppresses():
    # 同层相邻高分格（框高度重叠，IoU≈0.875 > 0.45）-> NMS 只留高分
    img = np.zeros((32, 32, 3), dtype=np.uint8)
    backend = FakeBackend(_synthetic_outputs(grid=4, nc=2,
                                             high_cells={(0, 0): 0.9, (0, 1): 0.3}))
    det = Detector(backend, num_classes=2, cfg=DetectorConfig(imgsz=32))
    dets = det.detect(img)
    assert len(dets) == 1
    assert dets[0].score == pytest.approx(0.9)


def test_detector_close_and_context():
    backend = FakeBackend(_synthetic_outputs())
    with Detector(backend, num_classes=2) as det:
        det.load("m.rknn")
        assert not backend.closed
    assert backend.closed


def test_detector_empty_prediction():
    img = np.zeros((32, 32, 3), dtype=np.uint8)
    box = np.zeros((1, 64, 4, 4), dtype=np.float32)
    cls = np.full((1, 2, 4, 4), 0.05, dtype=np.float32)   # 全低分
    backend = FakeBackend([box, cls, np.ones((1, 1, 4, 4), dtype=np.float32)])
    det = Detector(backend, num_classes=2, cfg=DetectorConfig(imgsz=32, score_thr=0.25))
    assert det.detect(img) == []
