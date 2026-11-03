"""RKNN 推理线程池（参照 rknn-python-Multithreading/rknnpool.py 改造）。

1. 后端工厂注入：板上统一 rknnlite（`rknnlite.api.RKNNLite`，2.3.2，RK3576 专用）；
   测试注入 FakeBackend。
2. 核轮转：`rknn_backend_factory('round_robin')` 按线程 id % 3 绑定
   NPU_CORE_0/1/2（参考实现 initRKNNs 的逻辑）。

契约（TDD 断言目标）：
- rknn_backend_factory(core_mask) -> Callable[[int], Backend]
- RKNNPool(backend_factory, model_path, tpes, process_fn)
  process_fn(backend, item) -> result；每线程独立后端实例（id 0..tpes-1）
- put(item) / get(timeout=None) -> (result, ok)；worker 异常 -> (None, False)
- release()：停止并 join 全部线程
"""
from __future__ import annotations

import queue
import threading
from typing import Callable

from detector import Backend, RKNNBackend


def rknn_backend_factory(core_mask: str = "auto") -> Callable[[int], Backend]:
    """构造 RKNN 后端工厂；round_robin 时按线程 id % 3 轮转 NPU 核心。"""
    if core_mask not in ("auto", "round_robin"):
        raise ValueError(f"unknown core_mask: {core_mask} (auto|round_robin)")
    if core_mask == "round_robin":
        masks = [RKNNBackend.NPU_CORE_0, RKNNBackend.NPU_CORE_1, RKNNBackend.NPU_CORE_2]
        return lambda tid: RKNNBackend(core_mask=masks[tid % 3])
    return lambda tid: RKNNBackend()


class RKNNPool:
    def __init__(
        self,
        backend_factory: Callable[[int], Backend],
        model_path: str,
        tpes: int,
        process_fn: Callable[[Backend, object], object],
    ):
        self.tpes = tpes
        self.process_fn = process_fn
        self.model_path = model_path
        self.inputs: queue.Queue = queue.Queue()
        self.outputs: queue.Queue = queue.Queue()
        self.running = True
        self.threads = []
        self._backends = []
        for i in range(tpes):
            backend = backend_factory(i)
            self._backends.append(backend)
            t = threading.Thread(target=self._worker, args=(i, backend), daemon=True)
            t.start()
            self.threads.append(t)

    def _worker(self, thread_id: int, backend: Backend) -> None:
        try:
            backend.load(self.model_path)
        except Exception as exc:  # pragma: no cover - 板载环境路径
            print(f"[pool:{thread_id}] load failed: {exc}")
            self.outputs.put((None, False))
            return
        while self.running:
            try:
                item = self.inputs.get(timeout=1)
            except queue.Empty:
                continue
            try:
                result = self.process_fn(backend, item)
                self.outputs.put((result, True))
            except Exception as exc:
                print(f"[pool:{thread_id}] error: {exc}")
                self.outputs.put((None, False))
                break
        backend.close()

    def put(self, item) -> None:
        self.inputs.put(item)

    def get(self, timeout: float | None = None) -> tuple[object, bool]:
        return self.outputs.get(timeout=timeout)

    def release(self) -> None:
        self.running = False
        for t in self.threads:
            t.join(timeout=5)
        for b in self._backends:
            b.close()
