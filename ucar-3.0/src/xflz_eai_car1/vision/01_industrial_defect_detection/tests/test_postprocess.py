"""postprocess 模块 TDD 测试（对照 rknn_model_zoo yolov8.py 解码）。

设计说明：解码测试统一用「高分格」构造——DFL 偏移产生的框较大，
不同格/层的框可能重叠，NMS 行为单独设计用例断言。
"""
import numpy as np
import pytest

from postprocess import Detection, classwise_nms, decode_rknn, dfl_decode


def _level_box(grid_h, grid_w, value=0.0):
    """单层 box 张量 (1,64,h,w)：全 value -> DFL 输出 = 7.5（均值 0..15）。"""
    return np.full((1, 64, grid_h, grid_w), value, dtype=np.float32)


def _level_cls(grid_h, grid_w, nc, high_cells):
    """cls 张量：high_cells = {(r,c): score}，其余 0.05（低于阈值）。"""
    cls = np.full((1, nc, grid_h, grid_w), 0.05, dtype=np.float32)
    for (r, c), s in high_cells.items():
        cls[0, 0, r, c] = s
    return cls


def test_dfl_uniform_bins_gives_7p5():
    # 16 个全等 bin -> softmax 均匀 -> 加权和 = mean(0..15) = 7.5
    pos = np.zeros((1, 64, 4, 4), dtype=np.float32)
    out = dfl_decode(pos)
    assert out.shape == (1, 4, 4, 4)
    assert np.allclose(out, 7.5)


def test_decode_single_level_hand_computed():
    # imgsz=32, 4x4 grid, stride=8；grid(0,0) 高分格，DFL=7.5
    # xy = grid+0.5±7.5 -> x1=(0+0.5-7.5)*8=-56, x2=(0+0.5+7.5)*8=64
    imgsz, grid = 32, 4
    outs = [_level_box(grid, grid),
            _level_cls(grid, grid, 2, {(0, 0): 0.9}),
            np.ones((1, 1, grid, grid), dtype=np.float32)]
    dets = decode_rknn(outs, num_classes=2, imgsz=imgsz, score_thr=0.25)
    assert len(dets) == 1
    d = dets[0]
    assert d.x1 == pytest.approx(-56.0)
    assert d.y1 == pytest.approx(-56.0)
    assert d.x2 == pytest.approx(64.0)
    assert d.y2 == pytest.approx(64.0)
    assert d.class_id == 0
    assert d.score == pytest.approx(0.9)


def test_decode_class_argmax():
    # 类1 分数更高 -> class_id=1
    imgsz, grid = 32, 4
    cls = np.full((1, 2, grid, grid), 0.05, dtype=np.float32)
    cls[0, 1, 1, 1] = 0.8
    outs = [_level_box(grid, grid), cls, np.ones((1, 1, grid, grid), dtype=np.float32)]
    dets = decode_rknn(outs, num_classes=2, imgsz=imgsz, score_thr=0.25)
    assert len(dets) == 1
    assert dets[0].class_id == 1


def test_decode_filters_low_score():
    imgsz, grid = 16, 2
    cls = np.full((1, 1, grid, grid), 0.05, dtype=np.float32)
    cls[0, 0, 0, 0] = 0.9
    cls[0, 0, 0, 1] = 0.1   # < 阈值
    outs = [_level_box(grid, grid), cls, np.ones((1, 1, grid, grid), dtype=np.float32)]
    dets = decode_rknn(outs, num_classes=1, imgsz=imgsz, score_thr=0.25)
    assert len(dets) == 1
    assert dets[0].score == pytest.approx(0.9)


def test_decode_three_levels_all_kept():
    # 3 层各 1 高分格（都在 grid(0,0)）：框同心嵌套，IoU=0.25/0.0625 < 0.45 -> NMS 全保留
    imgsz = 32
    outs = []
    for grid in (4, 2, 1):
        outs.append(_level_box(grid, grid))
        outs.append(_level_cls(grid, grid, 1, {(0, 0): 0.9}))
        outs.append(np.ones((1, 1, grid, grid), dtype=np.float32))
    assert len(outs) == 9
    dets = decode_rknn(outs, num_classes=1, imgsz=imgsz)
    assert len(dets) == 3
    assert all(d.score == pytest.approx(0.9) for d in dets)


def test_decode_adjacent_cells_nms_suppresses():
    # 同层相邻高分格：框重叠 IoU≈0.875 > 0.45 -> 只保留高分
    imgsz, grid = 32, 4
    cls = np.full((1, 1, grid, grid), 0.05, dtype=np.float32)
    cls[0, 0, 0, 0] = 0.9
    cls[0, 0, 0, 1] = 0.8
    outs = [_level_box(grid, grid), cls, np.ones((1, 1, grid, grid), dtype=np.float32)]
    dets = decode_rknn(outs, num_classes=1, imgsz=imgsz)
    assert len(dets) == 1
    assert dets[0].score == pytest.approx(0.9)


def test_decode_wrong_output_count_raises():
    outs = [np.zeros((1, 64, 4, 4), dtype=np.float32)]  # 缺 cls/cls_sum
    with pytest.raises(AssertionError):
        decode_rknn(outs, num_classes=1, imgsz=32)


def test_decode_wrong_class_channels_raises():
    imgsz, grid = 16, 2
    outs = [_level_box(grid, grid),
            np.zeros((1, 5, grid, grid), dtype=np.float32),   # nc 声明 1，实际 5
            np.ones((1, 1, grid, grid), dtype=np.float32)]
    with pytest.raises(ValueError, match="cls channels"):
        decode_rknn(outs, num_classes=1, imgsz=imgsz)


def test_nms_same_class_suppresses():
    boxes = np.array([[0, 0, 100, 100], [10, 10, 110, 110]], dtype=np.float32)  # IoU≈0.67
    scores = np.array([0.9, 0.8], dtype=np.float32)
    cls = np.array([0, 0], dtype=np.int64)
    kept = classwise_nms(boxes, scores, cls, iou_thr=0.45)
    assert len(kept) == 1
    assert kept[0].score == pytest.approx(0.9)


def test_nms_different_classes_kept():
    boxes = np.array([[0, 0, 100, 100], [10, 10, 110, 110]], dtype=np.float32)
    scores = np.array([0.9, 0.8], dtype=np.float32)
    cls = np.array([0, 1], dtype=np.int64)
    kept = classwise_nms(boxes, scores, cls, iou_thr=0.45)
    assert len(kept) == 2


def test_nms_empty_input():
    assert classwise_nms(np.zeros((0, 4)), np.zeros(0), np.zeros(0, dtype=np.int64)) == []


def test_nms_low_score_kept_by_design():
    # classwise_nms 只做 IoU 抑制；分数过滤在 decode_rknn 层完成
    boxes = np.array([[0, 0, 10, 10]], dtype=np.float32)
    scores = np.array([0.1], dtype=np.float32)
    kept = classwise_nms(boxes, scores, np.array([0]))
    assert len(kept) == 1


def test_detection_fields():
    d = Detection(1.0, 2.0, 3.0, 4.0, 0.9, 2)
    assert (d.x1, d.y1, d.x2, d.y2, d.score, d.class_id) == (1.0, 2.0, 3.0, 4.0, 0.9, 2)
