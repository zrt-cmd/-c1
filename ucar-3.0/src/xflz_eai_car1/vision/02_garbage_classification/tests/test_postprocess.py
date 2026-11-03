"""postprocess 模块 TDD 测试（构建文档 03 计划；对照参考 func.py 算法）。"""
import numpy as np
import pytest

from postprocess import (
    Detection,
    classwise_nms,
    decode_rknn_outputs,
    decode_scale,
    map_categories,
)
from conftest import ANCHORS_9, CAT_MAP_20, CAT_NAMES


def _scale_tensor(grid_h, grid_w, value=0.0):
    """(1,75,h,w) 全 value 张量。"""
    return np.full((1, 75, grid_h, grid_w), value, dtype=np.float32)


def _cell_tensor(grid_h, grid_w, cell, obj_raw=5.0, cls_id=0, cls_raw=5.0, box_raw=0.0):
    """单格高分张量：**仅锚 0** 的 obj/cls/box 通道在 (r,c) 置值（其余 -10，σ≈0）。"""
    t = np.full((1, 75, grid_h, grid_w), -10.0, dtype=np.float32)
    r, c = cell
    t[0, 0:4, r, c] = box_raw         # box xywh（锚 0）
    t[0, 4, r, c] = obj_raw           # obj（锚 0）
    t[0, 5 + cls_id, r, c] = cls_raw  # cls[cls_id]（锚 0）
    return t


def test_decode_scale_hand_computed():
    # (1,75,1,1) 全 0，sigmoid 后 0.5：xy=(2*0.5-0.5)+grid(0,0)=0.5 → ×32=16
    # wh=(2*0.5)^2 × anchor=(10,13)；score=0.5*0.5=0.25
    t = _scale_tensor(1, 1, 0.0)
    boxes, scores, _ = decode_scale(t, [(10, 13), (16, 30), (33, 23)],
                                    stride=32, score_thr=0.2, sigmoid=True)
    assert boxes.shape[0] == 3  # 3 锚各一框
    b0 = boxes[0]
    assert b0[0] == pytest.approx(16.0)
    assert b0[1] == pytest.approx(16.0)
    assert b0[2] == pytest.approx(10.0)
    assert b0[3] == pytest.approx(13.0)
    assert scores[0] == pytest.approx(0.25)


def test_decode_scale_anchor_formula():
    # σ=0.75：raw=ln(0.75/0.25)；xy=(1.5-0.5)=1.0×32=32；wh=(1.5)^2×10=22.5
    raw = float(np.log(0.75 / 0.25))
    t = _scale_tensor(1, 1, raw)
    boxes, _, _ = decode_scale(t, [(10, 13), (16, 30), (33, 23)], stride=32,
                               score_thr=0.5, sigmoid=True)
    assert boxes[0][0] == pytest.approx(32.0)
    assert boxes[0][2] == pytest.approx(22.5)


def test_decode_scale_grid_offset():
    # grid(1,2) stride 8：xy=(0.5+1)*8=12, (0.5+2)*8=20
    t = _scale_tensor(4, 4, 0.0)
    boxes, _, _ = decode_scale(t, [(10, 13), (16, 30), (33, 23)], stride=8,
                               score_thr=0.2, sigmoid=True)
    hits = boxes[(np.abs(boxes[:, 0] - 12) < 1e-3) & (np.abs(boxes[:, 1] - 20) < 1e-3)]
    assert len(hits) == 3  # 3 个锚


def test_decode_scale_score_threshold():
    t = _scale_tensor(1, 1, -10.0)  # σ≈4.5e-5 < 0.25
    boxes, scores, _ = decode_scale(t, [(10, 13), (16, 30), (33, 23)],
                                    stride=32, score_thr=0.25, sigmoid=True)
    assert boxes.shape[0] == 0
    assert scores.shape[0] == 0


def test_decode_scale_sigmoid_off():
    # sigmoid=False 原值直通：obj=0.5, cls=0.5, box=0.5
    # xy=(2*0.5-0.5)*32=16；wh=(2*0.5)^2*anchor=(10,13)；score=0.25
    t = _cell_tensor(1, 1, (0, 0), obj_raw=0.5, cls_raw=0.5, box_raw=0.5)
    boxes, scores, _ = decode_scale(t, [(10, 13), (16, 30), (33, 23)],
                                    stride=32, score_thr=0.2, sigmoid=False)
    assert boxes.shape[0] == 1
    assert scores[0] == pytest.approx(0.25)
    assert boxes[0][0] == pytest.approx(16.0)
    assert boxes[0][2] == pytest.approx(10.0)


def test_decode_scale_wrong_channels():
    t = np.zeros((1, 100, 2, 2), dtype=np.float32)
    with pytest.raises(ValueError, match="75"):
        decode_scale(t, [(10, 13), (16, 30), (33, 23)], 32, 0.25)


def test_classwise_nms_same_class():
    boxes = np.array([[0, 0, 100, 100], [10, 10, 110, 110]], dtype=np.float32)
    kept = classwise_nms(boxes, np.array([0.9, 0.8]), np.array([0, 0]), iou_thr=0.45)
    assert len(kept) == 1
    assert kept[0].score == pytest.approx(0.9)


def test_classwise_nms_diff_class():
    boxes = np.array([[0, 0, 100, 100], [10, 10, 110, 110]], dtype=np.float32)
    kept = classwise_nms(boxes, np.array([0.9, 0.8]), np.array([0, 1]), iou_thr=0.45)
    assert len(kept) == 2


def test_classwise_nms_empty():
    assert classwise_nms(np.zeros((0, 4)), np.zeros(0), np.zeros(0, dtype=np.int64)) == []


def test_decode_rknn_outputs_three_scales():
    # imgsz=32：层 h=4/2/1 -> stride 8/16/32；仅层0 单锚高分
    # σ=0.5 -> xy=(2*0.5-0.5)*8=4；wh=(10,13) -> xyxy=(-1,-2.5,9,10.5)；score=0.25
    outs = [_cell_tensor(4, 4, (0, 0), obj_raw=0.0, cls_raw=0.0),
            _scale_tensor(2, 2, -10.0), _scale_tensor(1, 1, -10.0)]
    dets = decode_rknn_outputs(outs, ANCHORS_9, imgsz=32, score_thr=0.25, nms_thr=0.45)
    assert len(dets) == 1
    d = dets[0]
    assert d.x1 == pytest.approx(4 - 5)
    assert d.y1 == pytest.approx(4 - 6.5)
    assert d.x2 == pytest.approx(4 + 5)
    assert d.y2 == pytest.approx(4 + 6.5)
    assert d.class_id == 0
    assert d.score == pytest.approx(0.25)


def test_decode_rknn_outputs_wrong_count():
    outs = [_scale_tensor(4, 4), _scale_tensor(2, 2)]
    with pytest.raises(ValueError, match="3 scale"):
        decode_rknn_outputs(outs, ANCHORS_9, imgsz=32)


def test_decode_rknn_outputs_wrong_anchors():
    outs = [_scale_tensor(4, 4), _scale_tensor(2, 2), _scale_tensor(1, 1)]
    with pytest.raises(ValueError, match="9 anchors"):
        decode_rknn_outputs(outs, ANCHORS_9[:6], imgsz=32)


def test_decode_rknn_outputs_all_low_score_empty():
    outs = [_scale_tensor(4, 4, -10.0), _scale_tensor(2, 2, -10.0), _scale_tensor(1, 1, -10.0)]
    assert decode_rknn_outputs(outs, ANCHORS_9, imgsz=32) == []


def test_map_categories():
    dets = [Detection(0, 0, 10, 10, 0.9, 10),   # shuiping -> recyclable
            Detection(0, 0, 10, 10, 0.8, 4),    # dianchi -> hazardous_waste
            Detection(0, 0, 10, 10, 0.7, 19)]   # zhuankuai -> other_waste
    mapped = map_categories(dets, CAT_MAP_20, CAT_NAMES)
    assert mapped[0].category == 1
    assert mapped[0].category_name == "recyclable"
    assert mapped[1].category == 3
    assert mapped[1].category_name == "hazardous_waste"
    assert mapped[2].category == 4
    assert mapped[2].category_name == "other_waste"
    assert mapped[0].score == 0.9
