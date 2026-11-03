"""手势分类：21 点 → embedding → 8 类（CSDN 方案第 3/4 阶段）。

移植自 mediapipe-rknn/utils/postprocess/gesture.py（test_board.py 管线语义），
按板端实测 2026-08-07 修正：

gesture_embedder.rknn 实测输入 = **单个 `norm_tensor:0` (1,21,3) float16**——
转换产物只保留 1 个输入（参考仓库 README 的 3 输入 hand+handedness+world
与模型不符，作废）。输入语义 = **已归一化 21 点**（norm_tensor 名字即含义）：
  x, y = 224 输入空间像素 /224 → [0,1]
  z     = 同像素单位（-10~+10 量级）/224  → [0,1] 量级
板端差分实测（用户结论 + 复现）：z 不归一化 → embedding 被 z 主导
（norm 10.5 vs 3.3）→ classifier logits 饱和 [1,0,0,...] → 全部判 None；
z/224 → embedding norm 3.3、logits 分化 → Open_Palm/Victory 正确。

gesture_embedder 输出：(1,128) embedding
canned_gesture_classifier 输入：(1,128) `hand_embedding`；输出：(1,8) logits → softmax。
"""
from __future__ import annotations

import numpy as np

GESTURE_LABELS = [
    "None", "Closed_Fist", "Open_Palm", "Pointing_Up",
    "Thumb_Down", "Thumb_Up", "Victory", "ILoveYou",
]


def prepare_embedder_inputs(landmarks_224: np.ndarray,
                            input_size: int = 224) -> list:
    """224 输入空间 21 点 → gesture_embedder 的 1 个 float 输入（norm_tensor）。

    Args:
        landmarks_224: (21,3) —— hand_landmarks 模型输出原始坐标（224 空间像素），
            布局 [x, y, z]（板端实测，无需交换）。
        input_size: hand_landmark 模型输入边长（224）。

    Returns:
        [hand_in] —— (1,21,3) float32，x/y/z 均除以 input_size 归一化。
    """
    norm = landmarks_224.copy().astype(np.float32)
    norm /= input_size            # x/y/z 同像素单位，全部归一化
    return [norm.reshape(1, 21, 3)]


def decode_gesture(classifier_output: np.ndarray) -> dict:
    """分类器 (1,8) logits → softmax → {label, score, scores}。"""
    logits = np.asarray(classifier_output).reshape(8).astype(np.float64)
    e = np.exp(logits - logits.max())
    probs = e / e.sum()
    idx = int(np.argmax(probs))
    return {
        "label": GESTURE_LABELS[idx],
        "score": float(probs[idx]),
        "scores": probs.tolist(),
    }
