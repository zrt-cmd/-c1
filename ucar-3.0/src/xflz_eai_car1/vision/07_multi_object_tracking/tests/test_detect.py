"""detect 模块 TDD 测试（构建文档 02/03 的 6 条计划）。"""
import numpy as np
import pytest

from detect import Detection, decode_outputs, dfl_decode, box_process, nms


def _synthetic_outputs(imgsz=64):
    """仅 P3 cell(2,2) 一个 person(0) 检测：dfl dist=2、cls=0.9、sum=0.9。"""
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
                b[0, 16 * c + 2, 2, 2] = 100.0
            cl[0, 0, 2, 2] = 0.9
            s[0, 0, 2, 2] = 0.9
        outs += [b, cl, s]
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
    assert len(nms(boxes, scores, 0.45, np.zeros(2, dtype=np.int32))) == 1
    assert len(nms(boxes, scores, 0.45, np.array([0, 1], dtype=np.int32))) == 2


def test_decode_outputs_end_to_end():
    dets = decode_outputs(_synthetic_outputs(64), 0.25, 0.45, imgsz=64)
    assert len(dets) == 1
    d = dets[0]
    assert isinstance(d, Detection)
    assert d.class_id == 0
    assert d.score == pytest.approx(0.81, abs=1e-4)
    assert [d.x1, d.y1, d.x2, d.y2] == pytest.approx([4, 4, 36, 36], abs=1.0)


def test_decode_no_detection():
    outs = _synthetic_outputs(64)
    for o in outs:
        o[...] = 0
    assert decode_outputs(outs, 0.25, 0.45, imgsz=64) == []


def test_decode_filters_low_score():
    outs = _synthetic_outputs(64)
    dets = decode_outputs(outs, 0.9, 0.45, imgsz=64)  # 0.81 < 0.9
    assert dets == []
