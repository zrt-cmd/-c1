"""匹配工具：IoU、Hungarian、门限指派、级联（deep_sort 结构，纯 numpy 自实现）。"""
from __future__ import annotations

import numpy as np


def iou_batch(bboxes: np.ndarray, candidates: np.ndarray) -> np.ndarray:
    """(M,4) xyxy x (N,4) xyxy -> (M,N) IoU。"""
    bboxes = bboxes.astype(np.float32)
    candidates = candidates.astype(np.float32)
    xx1 = np.maximum(bboxes[:, None, 0], candidates[None, :, 0])
    yy1 = np.maximum(bboxes[:, None, 1], candidates[None, :, 1])
    xx2 = np.minimum(bboxes[:, None, 2], candidates[None, :, 2])
    yy2 = np.minimum(bboxes[:, None, 3], candidates[None, :, 3])
    inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
    area_b = (bboxes[:, 2] - bboxes[:, 0]) * (bboxes[:, 3] - bboxes[:, 1])
    area_c = (candidates[:, 2] - candidates[:, 0]) * (candidates[:, 3] - candidates[:, 1])
    union = area_b[:, None] + area_c[None, :] - inter
    return inter / np.maximum(union, 1e-9)


def linear_assignment(cost_matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Hungarian：返回 (行索引, 列索引)。scipy 板载已有。"""
    from scipy.optimize import linear_sum_assignment

    rows, cols = linear_sum_assignment(cost_matrix)
    return np.asarray(rows, dtype=np.int64), np.asarray(cols, dtype=np.int64)


def min_cost_matching(cost_matrix: np.ndarray, max_cost: float
                      ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """门限内最小代价指派。返回 (匹配行, 匹配列, 未匹配行)。"""
    if cost_matrix.size == 0:
        return (np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64),
                np.arange(cost_matrix.shape[0]))
    rows, cols = linear_assignment(cost_matrix)
    keep = cost_matrix[rows, cols] <= max_cost
    rows, cols = rows[keep], cols[keep]
    unmatched = np.setdiff1d(np.arange(cost_matrix.shape[0]), rows)
    return rows, cols, unmatched


def gate_cost_matrix(cost_matrix: np.ndarray, gating_threshold: float,
                     gating_scores: np.ndarray | None = None,
                     only_position: bool = False) -> np.ndarray:
    """马氏门限裁剪：超门限的代价置为大值。gating_scores: (M,N) 马氏距离。"""
    if gating_scores is None:
        return cost_matrix
    invalid = gating_scores > gating_threshold
    cost_matrix = np.where(invalid, 1e5, cost_matrix)
    return cost_matrix
