"""8 维常速 Kalman 滤波器（deep_sort 标准运动模型，纯 numpy 自实现）。

状态 x = [cx, cy, a(宽高比), h, vx, vy, va, vh]；观测 z = [cx, cy, a, h]。
**关键约定（deep_sort）**：过程/观测噪声标准差按当前高度 h 缩放：
std_position = std_weight_position * h，std_velocity = std_weight_velocity * h。
对照 vendor Deepsort/DeepSORT/sort/kalman_filter.py（BSD 参考，自实现）。
"""
from __future__ import annotations

import numpy as np

NDIM = 4
CHI2INV95 = {1: 3.8415, 2: 5.9915, 3: 7.8147, 4: 9.4877, 5: 11.070}


class KalmanFilter:
    def __init__(self):
        self._std_weight_position = 1.0 / 20
        self._std_weight_velocity = 1.0 / 160

    def _std(self, h: float, velocity: bool = False) -> np.ndarray:
        wp, wv = self._std_weight_position, self._std_weight_velocity
        if velocity:
            return np.array([10 * wv * h, 10 * wv * h, 1e-5, 10 * wv * h],
                            dtype=np.float32)
        return np.array([2 * wp * h, 2 * wp * h, 1e-2, 2 * wp * h],
                        dtype=np.float32)

    def initiate(self, measurement: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(cx, cy, a, h) -> (mean, covariance)。std 按 h 缩放。"""
        mean = np.r_[measurement, np.zeros((NDIM,), dtype=np.float32)]
        std = np.r_[self._std(measurement[3]),
                    self._std(measurement[3], velocity=True)]
        return mean.astype(np.float32), np.diag(std ** 2).astype(np.float32)

    def predict(self, mean: np.ndarray, covariance: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray]:
        """常速预测一步（std 按当前 h 缩放）。"""
        h = max(mean[3], 1e-6)
        wp, wv = self._std_weight_position, self._std_weight_velocity
        std = np.array([wp * h, wp * h, 1e-2, wp * h,
                        wv * h, wv * h, 1e-5, wv * h], dtype=np.float32)
        motion_cov = np.diag(std ** 2).astype(np.float32)
        F = np.eye(2 * NDIM)
        F[0:NDIM, NDIM:] = np.eye(NDIM)
        mean = F @ mean
        covariance = F @ covariance @ F.T + motion_cov
        return mean.astype(np.float32), covariance

    def project(self, mean: np.ndarray, covariance: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray]:
        """映射到观测空间（含 R，std 按 h 缩放）。"""
        h = max(mean[3], 1e-6)
        std = np.array([self._std_weight_position * h,
                        self._std_weight_position * h, 1e-2,
                        self._std_weight_position * h], dtype=np.float32)
        H = np.eye(NDIM, 2 * NDIM)
        z = H @ mean
        s = H @ covariance @ H.T + np.diag(std ** 2)
        return z.astype(np.float32), s.astype(np.float32)

    def update(self, mean: np.ndarray, covariance: np.ndarray,
               measurement: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """卡尔曼更新。"""
        z, s = self.project(mean, covariance)
        H = np.eye(NDIM, 2 * NDIM)
        k = covariance @ H.T @ np.linalg.inv(s)
        mean = mean + k @ (measurement - z)
        covariance = covariance - k @ H @ covariance
        return mean.astype(np.float32), covariance.astype(np.float32)

    def gating_distance(self, mean: np.ndarray, covariance: np.ndarray,
                        measurements: np.ndarray, only_position: bool = False
                        ) -> np.ndarray:
        """马氏距离（观测空间）；only_position 时只算 (cx,cy)。"""
        z, s = self.project(mean, covariance)
        if only_position:
            z, s = z[:2], s[:2, :2]
            measurements = measurements[:, :2]
        d = measurements - z
        return np.array([di @ np.linalg.inv(s) @ di for di in d], dtype=np.float32)
