"""pose 几何模块测试（解码、逆映射、拓扑）。"""
from __future__ import annotations

import numpy as np
import pytest

from pose_control_core.pose import (
    INPUT_SIZE,
    POSE_CONNECTIONS,
    PoseResult,
    decode_pose,
    prepare,
    resize_pad,
)


def make_outputs(score=5.0, first=(128.0, 128.0, 0.0, 1.0, 10.0)):
    """构造输出：195 = [x,y,z,vis,presence]×39（板端实测布局；33 pose + 6 附加）。"""
    raw = np.zeros(195, dtype=np.float32)
    for i in range(39):
        raw[i * 5:(i + 1) * 5] = first
    return [
        raw.reshape(1, 195),
        np.array([[[[score]]]], dtype=np.float32),
        np.zeros((1, 256, 256, 1), dtype=np.float32),
        np.zeros((1, 64, 64, 39), dtype=np.float32),
        np.zeros((1, 117), dtype=np.float32),
    ]


def test_topology_35_edges_within_33():
    assert len(POSE_CONNECTIONS) == 35
    for a, b in POSE_CONNECTIONS:
        assert 0 <= a < 33 and 0 <= b < 33
        assert a != b


def test_topology_known_edges():
    assert (11, 12) in POSE_CONNECTIONS   # 肩
    assert (0, 1) in POSE_CONNECTIONS     # 鼻-眼
    assert (27, 29) in POSE_CONNECTIONS   # 左踝-左足


def test_resize_pad_letterbox():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    padded, scale, pad = resize_pad(img)
    assert padded.shape == (256, 256, 3)
    assert scale == 0.8              # min(256/320, 256/240)
    assert pad == (0, 32)            # (256-256)//2, (256-192)//2
    assert np.all(padded[:32] == 114)  # 上灰边（mediapipe-rknn 参考 pad 114）


def test_decode_pose_maps_center_back():
    # 320x240 帧：scale 0.8, pad (0,32)；256 空间中心 (128,128) → (160, 120)
    res = decode_pose(make_outputs(), scale=0.8, pad=(0, 32), orig_shape=(240, 320))
    assert res.detected
    assert res.landmarks.shape == (33, 4)
    assert abs(res.landmarks[0, 0] - 160.0) < 1e-3
    assert abs(res.landmarks[0, 1] - 120.0) < 1e-3


def test_decode_pose_z_scaled():
    outs = make_outputs(first=(128.0, 128.0, 0.4, 1.0, 10.0))
    res = decode_pose(outs, scale=0.8, pad=(0, 32), orig_shape=(240, 320))
    assert abs(res.landmarks[0, 2] - 0.5) < 1e-3   # 0.4 / 0.8


def test_decode_pose_score():
    res = decode_pose(make_outputs(score=-2.0), orig_shape=(240, 320))
    assert res.score < 0.5


def test_decode_pose_vis_passthrough():
    outs = make_outputs(first=(128.0, 128.0, 0.0, 0.3, 10.0))
    res = decode_pose(outs, scale=0.8, pad=(0, 32), orig_shape=(240, 320))
    assert abs(res.landmarks[0, 3] - 0.3) < 1e-4  # vis 原样透传（绘制端按阈值过滤）


def test_prepare_passthrough_uint8():
    img = np.full((256, 256, 3), 255, dtype=np.uint8)
    out = prepare(img)
    assert out.shape == (1, 256, 256, 3)
    assert out.dtype == np.uint8
    assert int(out[0, 0, 0, 0]) == 255  # 原样透传：归一化由 rknn 转换配置内部执行


def test_pose_result_empty_not_detected():
    assert not PoseResult().detected
    assert PoseResult(landmarks=np.zeros((33, 4))).detected
