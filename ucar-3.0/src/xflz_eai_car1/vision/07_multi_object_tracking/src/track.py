"""Track 生命周期（deep_sort 标准：Tentative -> Confirmed -> Deleted）。"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from kalman import KalmanFilter

TENTATIVE, CONFIRMED, DELETED = 1, 2, 3


@dataclass
class Track:
    measurement: np.ndarray     # (4,) cx,cy,a,h
    track_id: int
    n_init: int = 3
    max_age: int = 70
    kf: KalmanFilter = field(default_factory=KalmanFilter, repr=False)
    mean: np.ndarray = field(init=False, repr=False)
    covariance: np.ndarray = field(init=False, repr=False)
    hits: int = 1
    age: int = 1
    time_since_update: int = 0
    state: int = TENTATIVE
    features: list = field(default_factory=list)
    class_id: int = -1            # 最近一次关联检测的 COCO 类别（绘制用）
    _next_meas: np.ndarray = field(init=False, repr=False, default=None)

    def __post_init__(self):
        self.mean, self.covariance = self.kf.initiate(self.measurement)
        self._next_meas = self.measurement.copy()

    # ---- 状态查询 ----
    @property
    def confirmed(self) -> bool:
        return self.state == CONFIRMED

    @property
    def deleted(self) -> bool:
        return self.state == DELETED

    def to_xyah(self) -> np.ndarray:
        return self.mean[:4].copy()

    def to_tlwh(self) -> np.ndarray:
        cx, cy, a, h = self.mean[:4]
        w = a * h
        return np.array([cx - w / 2, cy - h / 2, w, h], dtype=np.float32)

    # ---- 生命周期 ----
    def predict(self) -> None:
        self.mean, self.covariance = self.kf.predict(self.mean, self.covariance)
        self.age += 1
        self.time_since_update += 1

    def update(self, measurement: np.ndarray, feature: np.ndarray | None = None,
               class_id: int | None = None) -> None:
        self.mean, self.covariance = self.kf.update(self.mean, self.covariance,
                                                    measurement)
        if feature is not None:
            self.features.append(feature)
        if class_id is not None:
            self.class_id = class_id
        self.hits += 1
        self.time_since_update = 0
        if self.state == TENTATIVE and self.hits >= self.n_init:
            self.state = CONFIRMED

    def mark_missed(self) -> None:
        if self.state == TENTATIVE:
            self.state = DELETED
        elif self.time_since_update > self.max_age:
            self.state = DELETED

    def gating_distance(self, measurements: np.ndarray,
                        only_position: bool = False) -> np.ndarray:
        return self.kf.gating_distance(self.mean, self.covariance,
                                       measurements, only_position)
