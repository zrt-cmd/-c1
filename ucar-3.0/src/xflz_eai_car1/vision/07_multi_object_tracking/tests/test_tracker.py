"""tracker 模块 TDD 测试（构建文档 04 的 7 条计划）。"""
import numpy as np
import pytest

from detect import Detection
from kalman import KalmanFilter
from matching import iou_batch, min_cost_matching
from track import CONFIRMED, DELETED, Track
from tracker import Tracker


def _det(x1, y1, x2, y2, score=0.9, cls=0):
    return Detection(x1, y1, x2, y2, score, cls)


def _feat(i, dim=128):
    """确定性单位向量（同 i 相似、异 i 正交）。"""
    v = np.zeros(dim, dtype=np.float32)
    v[i % dim] = 1.0
    return v


# ---------- Kalman ----------
def test_kalman_predict_moves_with_velocity():
    kf = KalmanFilter()
    mean, cov = kf.initiate(np.array([50.0, 50.0, 1.0, 20.0]))
    mean[4:] = np.array([5.0, 3.0, 0.0, 0.0])  # 注入速度
    mean, _ = kf.predict(mean, cov)
    assert mean[0] == pytest.approx(55.0, abs=0.5)
    assert mean[1] == pytest.approx(53.0, abs=0.5)


def test_kalman_update_corrects_measurement():
    kf = KalmanFilter()
    mean, cov = kf.initiate(np.array([50.0, 50.0, 1.0, 20.0]))
    mean, cov = kf.update(mean, cov, np.array([60.0, 50.0, 1.0, 20.0]))
    assert 50.0 < mean[0] < 60.0


def test_kalman_gating_far_larger_than_near():
    kf = KalmanFilter()
    mean, cov = kf.initiate(np.array([50.0, 50.0, 1.0, 20.0]))
    near = kf.gating_distance(mean, cov, np.array([[52.0, 50.0, 1.0, 20.0]]))
    far = kf.gating_distance(mean, cov, np.array([[500.0, 500.0, 1.0, 20.0]]))
    assert far[0] > near[0] and far[0] > 9.4877  # chi2 4d 0.95


# ---------- matching ----------
def test_iou_batch_known():
    a = np.array([[0, 0, 10, 10], [0, 0, 5, 5]], dtype=np.float32)
    b = np.array([[0, 0, 10, 10], [100, 100, 110, 110]], dtype=np.float32)
    iou = iou_batch(a, b)
    assert iou[0, 0] == pytest.approx(1.0)
    assert iou[0, 1] == pytest.approx(0.0)
    assert iou[1, 0] == pytest.approx(0.25)


def test_min_cost_matching_hungarian_then_threshold():
    cost = np.array([[0.1, 10.0], [10.0, 0.2]], dtype=np.float32)
    rows, cols, unmatched = min_cost_matching(cost, 1.0)
    assert rows.tolist() == [0, 1] and cols.tolist() == [0, 1]
    assert unmatched.tolist() == []
    # 门限 0.15：全局最优指派后仅保留 (0,0)（0.1）；(1,1)=0.2 被裁
    rows2, cols2, unmatched2 = min_cost_matching(cost, 0.15)
    assert rows2.tolist() == [0] and cols2.tolist() == [0]
    assert unmatched2.tolist() == [1]


# ---------- Track lifecycle ----------
def test_track_tentative_to_confirmed():
    t = Track(np.array([10.0, 10.0, 1.0, 10.0]), 1, n_init=3)
    assert not t.confirmed
    for _ in range(3):
        t.update(np.array([10.0, 10.0, 1.0, 10.0]))
    assert t.confirmed and t.state == CONFIRMED


def test_track_missed_deletes_after_max_age():
    t = Track(np.array([10.0, 10.0, 1.0, 10.0]), 1, n_init=1, max_age=2)
    t.update(np.array([10.0, 10.0, 1.0, 10.0]))  # 确认
    for _ in range(3):  # 每帧 predict 累加 time_since_update 再 mark_missed
        t.predict()
        t.mark_missed()
    assert t.state == DELETED


# ---------- Tracker 集成 ----------
def test_single_object_stable_id():
    trk = Tracker(max_age=10, n_init=2)
    boxes = [(10, 10, 40, 40), (12, 12, 42, 42), (14, 14, 44, 44),
             (16, 16, 46, 46), (18, 18, 48, 48)]
    for b in boxes:
        trk.predict()
        trk.update([_det(*b)])
    confirmed = [t for t in trk.tracks if t.confirmed]
    assert len(confirmed) == 1
    assert confirmed[0].track_id == 1


def test_two_objects_distinct_ids():
    trk = Tracker(max_age=10, n_init=2)
    for _ in range(4):
        trk.predict()
        trk.update([_det(10, 10, 40, 40), _det(100, 100, 130, 130)])
    ids = sorted(t.track_id for t in trk.tracks if t.confirmed)
    assert len(ids) == 2 and ids == [1, 2]


def test_lost_object_removed_after_max_age():
    trk = Tracker(max_age=3, n_init=1)
    trk.predict()
    trk.update([_det(10, 10, 40, 40)])
    for _ in range(4):
        trk.predict()
        trk.update([])
    assert all(t.deleted for t in trk.tracks)
def test_occlusion_recovery_appearance_gate():
    """遮挡 2 帧后近预测位重现（外观特征相同）：
    deepsort 经外观级联恢复原 ID；sort 纯 IoU 时 tsu>1 轨迹不参与匹配 -> 换 ID。"""

    def run(with_appearance):
        trk = Tracker(max_age=10, n_init=1,
                      mode="deepsort" if with_appearance else "sort")
        trk.predict()
        trk.update([_det(10, 10, 40, 40)],
                   np.array([_feat(1)]) if with_appearance else None)
        trk.predict()
        trk.update([_det(11, 11, 41, 41)],  # 第二帧命中 -> 确认
                   np.array([_feat(1)]) if with_appearance else None)
        ids_before = [t.track_id for t in trk.tracks]
        for _ in range(2):  # 遮挡
            trk.predict()
            trk.update([])
        # 在预测位置重现（平滑运动语义），外观特征不变
        trk.predict()
        trk.update([_det(16, 16, 36, 36)],  # 中心 (26,26) = 预测位置
                   np.array([_feat(1)]) if with_appearance else None)
        return ids_before, [t.track_id for t in trk.tracks if not t.deleted]

    id_b, id_a = run(True)
    assert id_a == id_b      # deepsort：外观级联恢复原 ID
    id_b2, id_a2 = run(False)
    assert id_a2 != id_b2    # sort：旧轨迹不参与 IoU -> 新 ID


def test_track_class_id_propagates_from_detections():
    trk = Tracker(mode="sort")
    trk.update([_det(0, 0, 10, 10, cls=2)])
    assert trk.tracks[0].class_id == 2
    trk.update([_det(0, 0, 10, 10, cls=5)])  # 同位置检测，IoU 匹配
    assert trk.tracks[0].class_id == 5       # 更新为最新检测类别


def test_new_track_keeps_detection_class():
    trk = Tracker(mode="sort")
    trk.update([_det(0, 0, 10, 10, cls=0), _det(50, 50, 60, 60, cls=5)])
    cls_by_id = {t.track_id: t.class_id for t in trk.tracks}
    assert set(cls_by_id.values()) == {0, 5}
