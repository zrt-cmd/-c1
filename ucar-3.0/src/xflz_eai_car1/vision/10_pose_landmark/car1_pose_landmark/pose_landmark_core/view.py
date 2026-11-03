"""双栏合成与状态栏（vendor frame_combine 移植）。"""
from __future__ import annotations

import cv2
import numpy as np

STATUS_HEIGHT = 24
STATUS_BG = (0, 0, 0)
STATUS_FG = (255, 255, 255)


def compose(frame: np.ndarray, mesh: np.ndarray) -> np.ndarray:
    """原帧 | 网格 双栏合成；形状 = (max_h, w1+w2, 3)。"""
    if frame.ndim != 3 or mesh.ndim != 3:
        raise ValueError("frame/mesh must be 3-channel BGR images")
    h = max(frame.shape[0], mesh.shape[0])
    dst = np.zeros((h, frame.shape[1] + mesh.shape[1], 3), dtype=np.uint8)
    dst[: frame.shape[0], : frame.shape[1]] = frame
    dst[: mesh.shape[0], frame.shape[1]:] = mesh
    return dst


def put_status(img: np.ndarray, fps: float, pose: bool) -> np.ndarray:
    """底部黑条状态栏：FPS: N Pose: OK/--（实验 10 特化文本；09 为 Faces）。"""
    out = img.copy()
    h, w = out.shape[:2]
    top = max(0, h - STATUS_HEIGHT)
    out[top:, :] = STATUS_BG
    text = f"FPS: {int(fps)} Pose: {'OK' if pose else '--'}"
    cv2.putText(out, text, (8, h - 7), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, STATUS_FG, 1, cv2.LINE_AA)
    return out


def add_fps(frame: np.ndarray, fps: float) -> np.ndarray:
    """左上角 FPS（vendor 对照：红字 `FPS : N`）。"""
    out = frame.copy()
    cv2.putText(out, f"FPS : {int(fps)}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (0, 0, 255), 1)
    return out
