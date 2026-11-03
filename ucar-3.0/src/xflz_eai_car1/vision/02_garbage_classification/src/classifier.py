"""分类器编排：后端注入（rknn/onnx/fake）+ 预处理 + 后处理。

契约（TDD 断言目标）：
- Backend(Protocol): load(path) / predict(input) -> np.ndarray / close()
  input: (imgsz, imgsz, 3) uint8 HWC BGR；predict 返回原始 logits（1-D 或 (1,N)）
- RKNNBackend：板上 rknnlite（RKNNLite 2.3.2，RK3576 专用；可选 core_mask）
- ONNXBackend / FakeBackend：同协议
- Classifier(backend, labels, cfg): classify(img) -> list[TopResult]
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from postprocess import TopResult, softmax, topk
from preprocess import prepare_input


class Backend(Protocol):
    def load(self, model_path: str) -> None: ...
    def predict(self, input_img: np.ndarray) -> np.ndarray: ...

class RKNNBackend:
    def __init__(self, core_mask: int = 0, verbose: bool = False):
        self._rknn = None
        self._core_mask = core_mask  # 0 = NPU_CORE_AUTO（板端实测：None 会报 core mode not supported）
        self._verbose = verbose

    def load(self, model_path: str) -> None:
        from rknnlite.api import RKNNLite  # 延迟导入：纯函数测试无需 rknnlite

        self._rknn = RKNNLite(verbose=self._verbose)
        if self._rknn.load_rknn(model_path) != 0:
            raise RuntimeError(f"load_rknn failed: {model_path}")
        if self._rknn.init_runtime(core_mask=self._core_mask) != 0:
            raise RuntimeError("init_runtime failed")

    def predict(self, input_img: np.ndarray) -> np.ndarray:
        outs = self._rknn.inference(inputs=[input_img], data_format="nhwc")
        out = outs[0]
        return out.reshape(-1)  # 分类输出 -> 1-D logits

    def close(self) -> None:
        self._rknn.release()


class ONNXBackend:
    def __init__(self, providers: list[str] | None = None):
        import onnxruntime as ort  # 延迟导入

        self._providers = list(providers) if providers else ["CPUExecutionProvider"]
        self._sess = None

    def load(self, model_path: str) -> None:
        import onnxruntime as ort

        self._sess = ort.InferenceSession(model_path, providers=self._providers)
        self._input_name = self._sess.get_inputs()[0].name

    def predict(self, input_img: np.ndarray) -> np.ndarray:
        inp = input_img.astype(np.float32)[..., ::-1]  # BGR -> RGB
        inp = inp.transpose(2, 0, 1)[None] / 255.0
        out = self._sess.run(None, {self._input_name: inp})[0]
        return np.asarray(out).reshape(-1)

    def close(self) -> None:
        self._sess = None


class FakeBackend:
    """测试用后端。"""

    def __init__(self, logits: np.ndarray):
        self.logits = logits
        self.loaded = None
        self.closed = False

    def load(self, model_path: str) -> None:
        self.loaded = model_path

    def predict(self, input_img: np.ndarray) -> np.ndarray:
        return self.logits.copy()

    def close(self) -> None:
        self.closed = True


@dataclass
class ClassifierConfig:
    imgsz: int = 224
    preprocess: str = "resize"
    softmax: bool = True
    topk: int = 3


class Classifier:
    def __init__(self, backend: Backend, labels: list[str], cfg: ClassifierConfig | None = None):
        self.backend = backend
        self.labels = labels
        self.cfg = cfg or ClassifierConfig()

    def load(self, model_path: str) -> "Classifier":
        self.backend.load(model_path)
        return self

    def classify(self, bgr_img: np.ndarray) -> list[TopResult]:
        inp = prepare_input(bgr_img, self.cfg.imgsz, self.cfg.preprocess)
        logits = self.backend.predict(inp)
        if logits.shape[0] != len(self.labels):
            raise ValueError(f"logits({logits.shape[0]}) != labels({len(self.labels)})")
        scores = softmax(logits) if self.cfg.softmax else logits
        return topk(scores, self.cfg.topk, self.labels)

    def close(self) -> None:
        self.backend.close()

    def __enter__(self) -> "Classifier":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
