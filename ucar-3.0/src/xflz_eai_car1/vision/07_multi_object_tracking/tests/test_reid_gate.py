"""ReID 门控：稳定目标不需新特征；新目标/未匹配才提。"""
import numpy as np
from detect import Detection
from tracker import Tracker
from reid import FakeReID

def _det(x1, y1, x2, y2, cls=2):
    return Detection(x1, y1, x2, y2, 0.9, cls)

def _trk_with(dets_seq, n_init=3):
    trk = Tracker(max_age=70, n_init=n_init, max_iou_distance=0.8)
    feats = np.eye(128)[np.arange(len(dets_seq[-1])) % 128]
    for i, dets in enumerate(dets_seq):
        trk.predict()
        f = np.eye(128)[np.arange(len(dets)) % 128]
        trk.update(dets, f)
    return trk

def test_gate_stable_then_new():
    from app import _reid_gate
    trk = _trk_with([[ _det(10, 10, 60, 60) ] for _ in range(5)])
    # 稳定位置(12,12 vs track 10,10 IoU 高) → 不需特征；新位置(300) → 需特征
    needed, reuse = _reid_gate([_det(12, 12, 58, 58), _det(300, 300, 400, 400)], trk, iou_thr=0.65)
    assert 0 not in needed and 1 in needed

def test_gate_no_confirmed_all_needed():
    from app import _reid_gate
    trk = Tracker(max_age=70, n_init=3)
    needed, reuse = _reid_gate([_det(0, 0, 10, 10)], trk, iou_thr=0.65)
    assert needed == [0] and reuse == []

def test_stable_id_preserved_with_gate():
    from app import _reid_gate, process_frame  # 引用门控即可；完整流程在 smoke
    trk = Tracker(max_age=70, n_init=3)
    for _ in range(3):                        # 3 次 update → confirmed
        trk.predict()
        trk.update([_det(10, 10, 60, 90)], np.eye(128)[:1])
    needed, reuse = _reid_gate([_det(10, 10, 60, 90)], trk, iou_thr=0.65)
    assert needed == [] and len(reuse) == 1  # 稳定目标被判 reuse（不提新特征）
