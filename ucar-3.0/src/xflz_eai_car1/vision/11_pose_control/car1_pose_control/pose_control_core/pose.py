"""BlazePose 几何：POSE_CONNECTIONS 拓扑、letterbox、输出解码。

模型：pose_landmark_full.tflite（mediapipe，整图 256×256 输入，无独立检测器）。
输出契约（mediapipe-rknn utils/postprocess/pose_landmark.py 核实）：
  [0] (1,195)  —— 前 132 = 33×4 [x,y,z,vis]（x/y 输入图归一化，z 相对尺度）
  [1] (1,1)    —— pose presence（logit）
  [2] (1,256,256,1) —— 分割掩膜（本实验不用）
  [3] (1,64,64,39)  —— 热图（本实验不用）
  [4] (1,117)  —— 39×3 world 关键点（米制，本实验不用）
逆映射公式（与 09 denormalize 同式）：px = (norm × 256 − pad) / scale；z' = z / scale。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

INPUT_SIZE = 256

# 官方 POSE_CONNECTIONS（mediapipe pose_landmarker 35 边，Apache-2.0）
POSE_CONNECTIONS: frozenset = frozenset([
    (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8),
    (9, 10), (11, 12), (11, 13), (13, 15), (15, 17), (15, 19), (15, 21),
    (17, 19), (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20),
    (11, 23), (12, 24), (23, 24), (23, 25), (24, 26), (25, 27), (26, 28),
    (27, 29), (28, 30), (29, 31), (30, 32), (27, 31), (28, 32),
])


@dataclass
class PoseResult:
    """一帧姿态结果：landmarks = 33×4 原图像素 [x,y,z,vis]（None = 未检出）。"""
    landmarks: np.ndarray | None = None
    score: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def detected(self) -> bool:
        return self.landmarks is not None


def resize_pad(img: np.ndarray, size: int = INPUT_SIZE) -> tuple[np.ndarray, float, tuple[int, int]]:
    """letterbox 缩放（保持纵横比 + pad 0），返回 (padded, scale, (pad_x, pad_y))。"""
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded = np.full((size, size, 3), 114, dtype=np.uint8)  # 灰边（mediapipe-rknn 参考）
    pad_x = (size - nw) // 2
    pad_y = (size - nh) // 2
    padded[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return padded, scale, (pad_x, pad_y)


def decode_pose(outputs: list[np.ndarray], scale: float = 1.0,
                pad: tuple[int, int] = (0, 0),
                orig_shape: tuple[int, int] | None = None) -> PoseResult:
    """解码 pose_landmark_full 输出 → PoseResult（landmarks 为原图像素坐标）。

    输出布局（板端实测 2026-08-07，修正 mediapipe-rknn postprocess 的错误假设）：
    outputs[0] (1,195) = **[x,y,z,vis,presence]×39 交错（39 点 = 33 pose + 6 附加；
    world 输出 (1,117) = 39×3 自洽）**，不是参考写的 [x,y,z,vis]×33；
    x/y 为 256 输入空间像素坐标；vis/presence 为 sigmoid 前 logit（每点列）；
    整体 presence 在 outputs[1]（fp16 模型已含 sigmoid）。
    逆映射：px = (lm − pad) / scale（参考同式）。
    """
    raw = np.asarray(outputs[0]).reshape(-1)
    lm5 = raw[:195].reshape(39, 5).astype(np.float32)   # [x,y,z,vis,presence]
    score = float(np.asarray(outputs[1]).reshape(-1)[0])

    lm = np.zeros((33, 4), dtype=np.float32)
    lm[:, 0] = lm5[:33, 0]
    lm[:, 1] = lm5[:33, 1]
    lm[:, 2] = lm5[:33, 2]
    lm[:, 3] = lm5[:33, 3]                                # vis logit
    if orig_shape is not None:
        pad_w, pad_h = pad
        lm[:, 0] = (lm[:, 0] - pad_w) / scale
        lm[:, 1] = (lm[:, 1] - pad_h) / scale
        lm[:, 2] = lm[:, 2] / scale
    return PoseResult(landmarks=lm, score=score,
                      extra={"segmentation": np.asarray(outputs[2]).reshape(INPUT_SIZE, INPUT_SIZE)
                             if len(outputs) > 2 else None})


def prepare(img: np.ndarray, size: int = INPUT_SIZE) -> np.ndarray:
    """BGR → RGB uint8 NHWC（不手动归一化）。

    板端实测（2026-08-07）：模型转换已配置 mean 127.5/std 128，rknn 推理时
    对 uint8 输入内部执行归一化；手动 /127.5-1 反而破坏输入（presence 0.055
    vs uint8 0.998）。
    """
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return rgb.reshape(1, size, size, 3)
