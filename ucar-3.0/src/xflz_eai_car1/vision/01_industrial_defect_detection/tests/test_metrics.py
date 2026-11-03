"""metrics 模块 TDD 测试（mAP50）。"""
import numpy as np
import pytest

from metrics import iou_matrix, map50
from postprocess import Detection


def _det(x1, y1, x2, y2, score, cls):
    return Detection(float(x1), float(y1), float(x2), float(y2), float(score), cls)


def test_iou_matrix_identity():
    boxes = np.array([[0, 0, 10, 10], [5, 5, 15, 15]], dtype=np.float32)
    ious = iou_matrix(boxes, boxes)
    assert np.allclose(np.diag(ious), 1.0)
    # 交集 5x5=25；并集 100+100-25=175 -> IoU=25/175
    assert ious[0, 1] == pytest.approx(25 / 175)


def test_iou_matrix_empty():
    assert iou_matrix(np.zeros((0, 4)), np.zeros((3, 4))).shape == (0, 3)


def test_map50_perfect_single_image():
    gt = [_det(0, 0, 100, 100, 1.0, 0)]
    pred = [_det(0, 0, 100, 100, 0.9, 0)]
    r = map50([pred], [gt], num_classes=2)
    assert r.map50 == 1.0
    assert r.per_class[0] == 1.0
    assert r.per_class[1] == 0.0


def test_map50_wrong_class_zero():
    # 预测类别与 GT 不同 -> 类0 AP=0；类1 无 GT -> 约定 AP=0
    gt = [_det(0, 0, 100, 100, 1.0, 0)]
    pred = [_det(0, 0, 100, 100, 0.9, 1)]
    r = map50([pred], [gt], num_classes=2)
    assert r.map50 == 0.0
    assert r.per_class[0] == 0.0
    assert r.per_class[1] == 0.0
def test_map50_single_gt_ap_saturates():
    # 1 GT + 2 预测（0.9 命中、0.7 假阳）：首预测即 recall=1 -> AP=1.0（AP 正确语义）
    gt = [_det(0, 0, 100, 100, 1.0, 0)]
    pred = [_det(0, 0, 100, 100, 0.9, 0), _det(200, 200, 300, 300, 0.7, 0)]
    r = map50([pred], [gt], num_classes=1)
    assert r.map50 == pytest.approx(1.0)


def test_map50_two_gt_partial():
    # 2 GT：TP(0.9)->r=0.5,p=1.0；FP(0.7)->r=0.5,p=0.5（recall 不再上升）
    # all-point AP = 0.5*1.0 + 0*0.5 = 0.5
    gt = [_det(0, 0, 100, 100, 1.0, 0), _det(200, 200, 300, 300, 1.0, 0)]
    pred = [_det(0, 0, 100, 100, 0.9, 0), _det(400, 400, 500, 500, 0.7, 0)]
    r = map50([pred], [gt], num_classes=1)
    assert r.map50 == pytest.approx(0.5)

def test_map50_three_images_curve():
    # 3 图各 1 GT：2 图命中 + 1 图漏检 -> recall=2/3, precision=1 -> AP=2/3
    gts = [[_det(0, 0, 10, 10, 1.0, 0)]] * 3
    preds = [[_det(0, 0, 10, 10, 0.9, 0)]] * 2 + [[]]
    r = map50(preds, gts, num_classes=1)
    assert r.map50 == pytest.approx(2 / 3)


def test_map50_empty_preds():
    gt = [_det(0, 0, 100, 100, 1.0, 0)]
    r = map50([[]], [gt], num_classes=1)
    assert r.map50 == 0.0
