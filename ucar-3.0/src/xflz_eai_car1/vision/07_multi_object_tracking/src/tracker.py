"""Tracker 编排：predict + 级联匹配 + 更新（deep_sort 标准结构，纯 numpy 自实现）。

对照 vendor Deepsort/DeepSORT/sort/tracker.py（BSD 参考，自实现）：
- 确认轨迹：外观（cosine + 马氏门限）按 time_since_update 从新到旧级联，剩余按 IoU
- 未确认（time_since_update==1）：IoU 匹配
- 未匹配检测 -> 新 Track（n_init 连续命中后确认）；未匹配轨迹 mark_missed（超龄删除）
- mode='sort'：无外观，纯 IoU（SORT 语义）
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from detect import Detection
from kalman import CHI2INV95
from matching import gate_cost_matrix, iou_batch, min_cost_matching
from track import Track


def det_to_xyah(d: Detection) -> np.ndarray:
    w, h = d.x2 - d.x1, d.y2 - d.y1
    return np.array([d.x1 + w / 2, d.y1 + h / 2, w / max(h, 1e-6), h],
                    dtype=np.float32)


def _track_xyxy(t: Track) -> np.ndarray:
    cx, cy, a, h = t.to_xyah()
    w = a * h
    return np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2],
                    dtype=np.float32)


def det_xyah_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    """(N,4) xyah -> (N,4) xyxy。"""
    b = boxes.astype(np.float32)
    w = b[:, 2] * b[:, 3]
    out = np.empty_like(b)
    out[:, 0] = b[:, 0] - w / 2
    out[:, 1] = b[:, 1] - b[:, 3] / 2
    out[:, 2] = b[:, 0] + w / 2
    out[:, 3] = b[:, 1] + b[:, 3] / 2
    return out


def _cosine_dist(features: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """(M,128) x (N,128) -> (M,N) cosine 距离。"""
    fn = features / np.maximum(np.linalg.norm(features, axis=1, keepdims=True), 1e-9)
    tn = targets / np.maximum(np.linalg.norm(targets, axis=1, keepdims=True), 1e-9)
    return 1.0 - fn @ tn.T


@dataclass
class Tracker:
    max_iou_distance: float = 0.7
    max_age: int = 70
    n_init: int = 3
    nn_budget: int = 100
    max_dist: float = 0.2
    mode: str = "deepsort"
    tracks: list = None
    _next_id: int = 1

    def __post_init__(self):
        if self.tracks is None:
            self.tracks = []

    # ---- 主循环 ----
    def predict(self) -> None:
        for t in self.tracks:
            t.predict()

    def update(self, detections: list[Detection],
               features: np.ndarray | None = None) -> None:
        """features (N,128) 与 detections 对齐；None -> 纯 IoU（sort 语义）。"""
        det_xyah = np.array([det_to_xyah(d) for d in detections],
                            dtype=np.float32).reshape(-1, 4)
        n_dets = len(detections)
        det_indices = list(range(n_dets))

        confirmed = [i for i, t in enumerate(self.tracks) if t.confirmed]
        matches_a, unmatched_tracks_a, unmatched_dets = self._cascade(
            confirmed, det_xyah, det_indices, features)

        iou_candidates = [i for i, t in enumerate(self.tracks)
                          if not t.confirmed] + \
            [i for i in unmatched_tracks_a if self.tracks[i].time_since_update == 1]
        matches_b, unmatched_tracks_b, unmatched_dets = self._iou_match(
            iou_candidates, det_xyah, unmatched_dets, features)
        for ti, di in matches_a + matches_b:
            self.tracks[ti].update(det_xyah[di],
                                   features[di] if features is not None else None,
                                   class_id=int(detections[di].class_id))
        for di in unmatched_dets:
            t = Track(det_xyah[di], self._next_id,
                      n_init=self.n_init, max_age=self.max_age,
                      class_id=int(detections[di].class_id))
            if features is not None:
                t.features.append(features[di])
            self.tracks.append(t)
            self._next_id += 1

        for ti in unmatched_tracks_a + unmatched_tracks_b:
            self.tracks[ti].mark_missed()
        self.tracks = [t for t in self.tracks if not t.deleted]

    # ---- 级联（外观） ----
    def _cascade(self, track_indices: list[int], det_xyah: np.ndarray,
                 det_indices: list[int], features: np.ndarray | None
                 ) -> tuple[list, list, list]:
        matches, unmatched_dets = [], list(det_indices)
        for level in range(self.max_age):
            if not unmatched_dets:
                break
            level_tracks = [i for i in track_indices
                            if self.tracks[i].time_since_update == 1 + level]
            if not level_tracks:
                continue
            cost = self._appearance_cost(level_tracks, det_xyah,
                                         unmatched_dets, features)
            rows, cols, _ = min_cost_matching(cost, self.max_dist)
            for r, c in zip(rows, cols):
                matches.append((level_tracks[r], unmatched_dets[c]))
            matched_dets = {unmatched_dets[c] for c in cols}
            unmatched_dets = [d for d in unmatched_dets if d not in matched_dets]
        matched_tracks = {ti for ti, _ in matches}
        unmatched_tracks = [i for i in track_indices if i not in matched_tracks]
        return matches, unmatched_tracks, unmatched_dets

    def _appearance_cost(self, track_indices: list[int], det_xyah: np.ndarray,
                         det_indices: list[int], features: np.ndarray | None
                         ) -> np.ndarray:
        """(M,N) 代价：cosine 外观距离 + 马氏门限；无外观/特征 -> 全大值。"""
        if features is None or not track_indices or not det_indices:
            return np.full((len(track_indices), len(det_indices)), 1e5,
                           dtype=np.float32)
        targets = np.array([np.mean(np.asarray(self.tracks[i].features), axis=0)
                            for i in track_indices], dtype=np.float32)
        cost = _cosine_dist(features[det_indices], targets).T  # (M,N)
        gating = np.stack([
            self.tracks[i].gating_distance(det_xyah[det_indices],
                                           only_position=True)
            for i in track_indices])
        return gate_cost_matrix(cost, CHI2INV95[2], gating)

    # ---- IoU 匹配 ----
    def _iou_match(self, track_indices: list[int], det_xyah: np.ndarray,
                   det_indices: list[int], features: np.ndarray | None
                   ) -> tuple[list, list, list]:
        if not track_indices or not det_indices:
            return [], track_indices, det_indices
        trk_boxes = np.stack([_track_xyxy(self.tracks[i]) for i in track_indices])
        det_boxes = det_xyah_to_xyxy(det_xyah[det_indices])
        cost = 1.0 - iou_batch(trk_boxes, det_boxes)
        rows, cols, unmatched_rows = min_cost_matching(cost, self.max_iou_distance)
        matches = [(track_indices[r], det_indices[c]) for r, c in zip(rows, cols)]
        matched_tracks = {track_indices[r] for r in rows}
        unmatched_tracks = [track_indices[r] for r in unmatched_rows
                            if track_indices[r] not in matched_tracks]
        unmatched_dets = [det_indices[i] for i in range(len(det_indices))
                          if i not in cols]
        return matches, unmatched_tracks, unmatched_dets
