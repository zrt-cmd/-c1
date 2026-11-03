"""detector 模块 TDD 测试（构建文档 04 的 7 条计划）。"""
import numpy as np
import pytest

from detector import Detector, DetectorConfig, FakeBackend, RKNNBackend, make_rknn_backend
from postprocess import CategoryDetection
from conftest import ANCHORS_9, CAT_MAP_20, CAT_NAMES, LABELS_20


def _cell_tensor(grid_h, grid_w, cell, cls_id=10, obj_raw=0.0, cls_raw=0.0, box_raw=0.0):
    """单格高分张量（仅锚 0）；σ=0.5 -> xy=grid*stride+stride/2，wh=anchor。"""
    t = np.full((1, 75, grid_h, grid_w), -10.0, dtype=np.float32)
    r, c = cell
    t[0, 0:4, r, c] = box_raw
    t[0, 4, r, c] = obj_raw
    t[0, 5 + cls_id, r, c] = cls_raw
    return t


def _three_scales(imgsz=32):
    """层 h=4/2/1，仅层0 单格高分（cls_id=10 -> shuiping -> recyclable）。"""
    return [_cell_tensor(4, 4, (0, 0), cls_id=10, obj_raw=0.0, cls_raw=0.0),
            np.full((1, 75, 2, 2), -10.0, dtype=np.float32),
            np.full((1, 75, 1, 1), -10.0, dtype=np.float32)]


def test_detector_load_calls_backend():
    backend = FakeBackend(_three_scales())
    det = Detector(backend, LABELS_20, CAT_MAP_20, CAT_NAMES).load("m.rknn")
    assert backend.loaded == "m.rknn"


def test_detect_original_coords_and_category():
    # img 64x32 -> imgsz 32：scale=0.5, pad_x=8, pad_y=0
    # letterbox 空间 xyxy=(-1,-2.5,9,10.5) -> 原图 = (box-[8,0,8,0])/0.5
    img = np.zeros((64, 32, 3), dtype=np.uint8)
    backend = FakeBackend(_three_scales(imgsz=32))
    det = Detector(backend, LABELS_20, CAT_MAP_20, CAT_NAMES,
                   cfg=DetectorConfig(imgsz=32, anchors=ANCHORS_9))
    dets = det.detect(img)
    assert len(dets) == 1
    d = dets[0]
    assert isinstance(d, CategoryDetection)
    assert d.x1 == pytest.approx((-1 - 8) / 0.5)
    assert d.y1 == pytest.approx((-2.5 - 0) / 0.5)
    assert d.x2 == pytest.approx((9 - 8) / 0.5)
    assert d.y2 == pytest.approx((10.5 - 0) / 0.5)
    assert d.class_id == 10
    assert d.category == 1          # shuiping -> recyclable
    assert d.category_name == "recyclable"


def test_detect_category_mapping_variants():
    # 层0 两个不同类别的高分格（不相邻，NMS 不抑制——大框会重叠，用不同分数验证高分保留）
    img = np.zeros((32, 32, 3), dtype=np.uint8)
    t = np.full((1, 75, 4, 4), -10.0, dtype=np.float32)
    t[0, 0:4, 0, 0] = 0.0
    t[0, 4, 0, 0] = 0.0
    t[0, 15, 0, 0] = 0.0      # cls_id=10 (5+10) shuiping
    backend = FakeBackend([t, np.full((1, 75, 2, 2), -10.0, dtype=np.float32),
                           np.full((1, 75, 1, 1), -10.0, dtype=np.float32)])
    det = Detector(backend, LABELS_20, CAT_MAP_20, CAT_NAMES,
                   cfg=DetectorConfig(imgsz=32, anchors=ANCHORS_9))
    dets = det.detect(img)
    assert len(dets) == 1
    assert dets[0].category_name == "recyclable"


def test_detect_empty_prediction():
    img = np.zeros((32, 32, 3), dtype=np.uint8)
    outs = [np.full((1, 75, 4, 4), -10.0, dtype=np.float32),
            np.full((1, 75, 2, 2), -10.0, dtype=np.float32),
            np.full((1, 75, 1, 1), -10.0, dtype=np.float32)]
    det = Detector(FakeBackend(outs), LABELS_20, CAT_MAP_20, CAT_NAMES,
                   cfg=DetectorConfig(imgsz=32, anchors=ANCHORS_9))
    assert det.detect(img) == []


def test_detect_wrong_output_shape_propagates():
    img = np.zeros((32, 32, 3), dtype=np.uint8)
    outs = [np.zeros((1, 75, 4, 4), dtype=np.float32)]  # 只有 1 层
    det = Detector(FakeBackend(outs), LABELS_20, CAT_MAP_20, CAT_NAMES,
                   cfg=DetectorConfig(imgsz=32, anchors=ANCHORS_9))
    with pytest.raises(ValueError, match="3 scale"):
        det.detect(img)


def test_detector_context_closes_backend():
    backend = FakeBackend(_three_scales())
    with Detector(backend, LABELS_20, CAT_MAP_20, CAT_NAMES) as det:
        det.load("m.rknn")
        assert not backend.closed
    assert backend.closed


def test_make_rknn_backend_type():
    assert isinstance(make_rknn_backend(), RKNNBackend)
