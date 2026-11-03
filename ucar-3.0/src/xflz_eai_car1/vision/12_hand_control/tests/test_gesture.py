"""手势分类测试（embedder 输入准备 + softmax 解码）。"""
from __future__ import annotations

import numpy as np

from hand_control_core.gesture import (
    GESTURE_LABELS,
    decode_gesture,
    prepare_embedder_inputs,
)


def test_labels_8_classes():
    assert GESTURE_LABELS == [
        "None", "Closed_Fist", "Open_Palm", "Pointing_Up",
        "Thumb_Down", "Thumb_Up", "Victory", "ILoveYou",
    ]


def test_prepare_embedder_inputs_single_norm_tensor():
    lm = np.zeros((21, 3), dtype=np.float32)
    lm[0] = (112.0, 112.0, 56.0)
    ins = prepare_embedder_inputs(lm, input_size=224)
    assert len(ins) == 1                       # norm_tensor 单输入（板端实测）
    assert ins[0].shape == (1, 21, 3) and ins[0].dtype == np.float32
    assert abs(ins[0][0, 0, 0] - 0.5) < 1e-4   # 112/224
    assert abs(ins[0][0, 0, 1] - 0.5) < 1e-4   # 112/224
    assert abs(ins[0][0, 0, 2] - 0.25) < 1e-4  # z/224（z 同像素单位，必须归一化）


def test_prepare_embedder_inputs_z_normalized():
    # z 不归一化 → embedding 被 z 主导 → classifier 饱和（板端差分实测）
    lm = np.zeros((21, 3), dtype=np.float32)
    lm[:, 0] = 112.0
    lm[:, 1] = 112.0
    lm[:, 2] = np.linspace(-6.0, 3.0, 21)      # 实测 z 值域
    ins = prepare_embedder_inputs(lm, input_size=224)
    z = ins[0][0, :, 2]
    assert z.max() <= 0.02 and z.min() >= -0.03   # z 已缩到 [0,1] 量级


def test_prepare_embedder_inputs_does_not_mutate():
    lm = np.full((21, 3), 224.0, dtype=np.float32)
    prepare_embedder_inputs(lm, input_size=224)
    assert lm[0, 0] == 224.0                          # 调用方数组不被改


def test_decode_gesture_softmax():
    out = np.zeros((1, 8), dtype=np.float32)
    out[0, 6] = 5.0                                   # Victory 最高
    g = decode_gesture(out)
    assert g["label"] == "Victory"
    assert g["score"] > 0.95


def test_decode_gesture_argmax():
    out = np.zeros((1, 8), dtype=np.float32)
    out[0, 4] = 3.0                                   # Thumb_Down
    g = decode_gesture(out)
    assert g["label"] == "Thumb_Down"
    assert g["scores"][4] > 0.5


def test_decode_gesture_all_labels_reachable():
    for i, label in enumerate(GESTURE_LABELS):
        out = np.zeros((1, 8), dtype=np.float32)
        out[0, i] = 2.0
        assert decode_gesture(out)["label"] == label
