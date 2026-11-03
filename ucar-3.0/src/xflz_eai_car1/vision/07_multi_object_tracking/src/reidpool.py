"""ReID 外观提取多线程池（构建文档 05）。

检测池（exp06 设计）复用：N worker × 独立后端实例（RKNN 实例非线程安全，
每线程一个）；提交时携带序号 (idx, crop)，按 idx 回收保证帧内顺序。
"""
from __future__ import annotations

from typing import Callable

import numpy as np

from reid import FakeReID, ReIDBackend
from rknnpool import RKNNPool


def _reid_process(backend, item):
    idx, crop = item
    return idx, backend([crop])[0]


class ReIDPool:
    """多线程外观提取：extract(crops) -> (N,128)，顺序与输入一致。"""

    def __init__(self, model_path: str, workers: int = 2,
                 factory: Callable[[], object] | None = None):
        factory = factory or (lambda: ReIDBackend())
        self._pool = RKNNPool(lambda tid: factory(), model_path,
                              workers, _reid_process)

    def extract(self, crops: list) -> np.ndarray:
        if not crops:
            return np.zeros((0, 128), dtype=np.float32)
        for i, c in enumerate(crops):
            self._pool.put((i, c))
        out = [self._pool.get(timeout=10) for _ in crops]
        valid = [(res[0], res[1]) for res, ok in out if ok]
        valid.sort(key=lambda p: p[0])  # 按提交序号还原帧内顺序
        return np.array([f for _, f in valid], dtype=np.float32)

    def close(self) -> None:
        self._pool.release()
