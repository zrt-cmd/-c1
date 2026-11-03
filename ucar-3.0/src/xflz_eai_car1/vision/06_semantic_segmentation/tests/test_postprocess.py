"""postprocess 模块 TDD 测试（构建文档 03 的 5 条计划）。"""
import numpy as np
import pytest

from postprocess import (
    SegResult,
    box_process,
    decode_outputs,
    dfl_decode,
    foreground_mask,
    nms,
)

CFG = {"model": {"score_thr": 0.25, "iou_thr": 0.45, "mask_thr": 0.5}}


def _synthetic_outputs(imgsz=64):
    """单 cell (2,2) person 检测：box one-hot dist=2，cls=0.9，mc=10，proto plane0=1。"""
    g = imgsz // 8  # P3 grid
    boxes = [np.zeros((1, 64, g, g), dtype=np.float32),
             np.zeros((1, 64, g // 2, g // 2), dtype=np.float32),
             np.zeros((1, 64, g // 4, g // 4), dtype=np.float32)]
    clss = [np.zeros((1, 80, g, g), dtype=np.float32),
            np.zeros((1, 80, g // 2, g // 2), dtype=np.float32),
            np.zeros((1, 80, g // 4, g // 4), dtype=np.float32)]
    sums = [np.zeros((1, 1, g, g), dtype=np.float32),
            np.zeros((1, 1, g // 2, g // 2), dtype=np.float32),
            np.zeros((1, 1, g // 4, g // 4), dtype=np.float32)]
    mcs = [np.zeros((1, 32, g, g), dtype=np.float32),
           np.zeros((1, 32, g // 2, g // 2), dtype=np.float32),
           np.zeros((1, 32, g // 4, g // 4), dtype=np.float32)]
    for c in range(4):  # 4 个坐标的 dfl one-hot bin=2
        boxes[0][0, 16 * c + 2, 2, 2] = 100.0
    clss[0][0, 0, 2, 2] = 0.9
    sums[0][0, 0, 2, 2] = 0.9
    mcs[0][0, 0, 2, 2] = 10.0
    proto = np.zeros((1, 32, imgsz // 4, imgsz // 4), dtype=np.float32)
    proto[0, 0] = 1.0
    outs = []
    for i in range(3):
        outs += [boxes[i], clss[i], sums[i], mcs[i]]
    outs.append(proto)
    return outs


def test_dfl_onehot_and_uniform():
    x = np.zeros((1, 64, 1, 1), dtype=np.float32)
    x[0, 2, 0, 0] = 100.0
    assert dfl_decode(x)[0, 0, 0, 0] == pytest.approx(2.0)
    assert dfl_decode(np.zeros((1, 64, 1, 1)))[0, 0, 0, 0] == pytest.approx(7.5)


def test_box_process_known_cell():
    x = np.zeros((1, 64, 8, 8), dtype=np.float32)
    for c in range(4):
        x[0, 16 * c + 2, 2, 2] = 100.0
    xyxy = box_process(x, imgsz=64)
    assert xyxy[0, :, 2, 2].tolist() == pytest.approx([4, 4, 36, 36], abs=0.01)


def test_nms_class_aware():
    boxes = np.array([[10, 10, 50, 50], [12, 12, 48, 48]], dtype=np.float32)
    scores = np.array([0.9, 0.5], dtype=np.float32)
    same = nms(boxes, scores, 0.45, class_ids=np.zeros(2, dtype=np.int32))
    assert len(same) == 1 and same[0] == 0
    diff = nms(boxes, scores, 0.45, class_ids=np.array([0, 1], dtype=np.int32))
    assert len(diff) == 2


def test_decode_outputs_end_to_end():
    outs = _synthetic_outputs(imgsz=64)
    res = decode_outputs(outs, CFG, imgsz=64)
    assert res is not None
    assert len(res.boxes) == 1
    assert res.classes[0] == 0
    assert res.scores[0] == pytest.approx(0.81, abs=1e-4)
    assert res.boxes[0].tolist() == pytest.approx([4, 4, 36, 36], abs=1.0)
    m = res.masks[0]
    assert m.shape == (64, 64)
    assert m[20, 20]          # 框内掩码 True
    assert not m[0, 0]        # 框外 False
    assert not m[40, 40]
    assert isinstance(res, SegResult)

def test_foreground_mask_class_filter():
    left = np.zeros((32, 32), dtype=bool)
    left[:, :16] = True
    right = np.zeros((32, 32), dtype=bool)
    right[:, 16:] = True
    res = SegResult(
        boxes=np.array([[0, 0, 10, 10], [0, 0, 10, 10]], dtype=np.float32),
        classes=np.array([0, 15], dtype=np.int32),
        scores=np.array([0.9, 0.8], dtype=np.float32),
        masks=np.stack([left, right]),
    )
    only_person = foreground_mask(res, [0])
    assert only_person[5, 5] and not only_person[5, 20]
    all_kept = foreground_mask(res, [])
    assert all_kept[5, 5] and all_kept[5, 20]
    none_kept = foreground_mask(res, [2])
    assert not none_kept.any()


def test_decode_outputs_no_detection():
    outs = _synthetic_outputs(imgsz=64)
    for o in outs:
        o[...] = 0
    assert decode_outputs(outs, CFG, imgsz=64) is None
