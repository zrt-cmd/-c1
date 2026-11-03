"""hand 几何模块测试（拓扑、palm 解码、ROI、21 点解码）。"""
from __future__ import annotations

import numpy as np

from hand_control_core.hand import (
    HAND_CONNECTIONS,
    HAND_SIZE,
    PALM_SIZE,
    crop_hand,
    decode_hand,
    decode_palm,
    palm_anchors,
    prepare,
    resize_pad,
    scale_detections,
    union_bbox,
    detection2roi,
    detection_box2roi,
    roi_affine,
    unproject_landmarks,
)


def test_topology_21_edges_within_21():
    assert len(HAND_CONNECTIONS) == 21
    for a, b in HAND_CONNECTIONS:
        assert 0 <= a < 21 and 0 <= b < 21
        assert a != b


def test_topology_known_edges():
    assert (3, 4) in HAND_CONNECTIONS   # 拇指尖
    assert (0, 5) in HAND_CONNECTIONS   # 掌
    assert (13, 17) in HAND_CONNECTIONS # 小指根


def test_palm_anchors_count_2016():
    anchors = palm_anchors(192)
    assert anchors.shape == (2016, 4)
    assert abs(anchors[0, 0] - 0.5 / 24) < 1e-6  # 首锚：stride 8 → feat 24（float32）


def test_resize_pad_192():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    padded, scale, pad = resize_pad(img, 192)
    assert padded.shape == (192, 192, 3)
    assert abs(scale - 0.6) < 1e-6      # min(192/320, 192/240)
    assert pad == (0, 24)               # (192-192)//2, (192-144)//2


def test_prepare_float_0_1():
    img = np.full((192, 192, 3), 255, dtype=np.uint8)
    out = prepare(img, 192)
    assert out.shape == (1, 192, 192, 3)
    assert out.dtype == np.float32
    assert abs(float(out[0, 0, 0, 0]) - 1.0) < 1e-4   # 255/255
    assert abs(float(out[0, 0, 0, 1]) - 1.0) < 1e-4
    dark = np.full((192, 192, 3), 128, dtype=np.uint8)
    assert abs(float(prepare(dark, 192)[0, 0, 0, 0]) - 128/255) < 1e-4


def test_decode_palm_zero_reg_at_anchor0():
    # 锚 0 位置 (4,4)，回归宽高 10px → bbox (-1,-1,9,9)
    reg = np.zeros((2016, 18), dtype=np.float32)
    reg[0, 2] = 10.0
    reg[0, 3] = 10.0
    cls = np.full((2016, 1), -100.0, dtype=np.float32)
    cls[0] = 10.0
    dets = decode_palm(reg, cls, palm_anchors(192), 192)
    assert len(dets) == 1
    x1, y1, x2, y2 = dets[0]["bbox"]
    assert abs(x1 + 1.0) < 1e-3 and abs(y1 + 1.0) < 1e-3
    assert abs(x2 - 9.0) < 1e-3 and abs(y2 - 9.0) < 1e-3
    assert len(dets[0]["keypoints"]) == 7   # palm 18 = 4 + 7×2
    assert dets[0]["score"] > 0.99


def test_decode_palm_low_score_empty():
    reg = np.zeros((2016, 18), dtype=np.float32)
    cls = np.full((2016, 1), -100.0, dtype=np.float32)
    assert decode_palm(reg, cls, palm_anchors(192), 192) == []


def test_decode_palm_nms_dedup():
    # 两个重叠高分锚（有面积）→ NMS 只留 1 个
    reg = np.zeros((2016, 18), dtype=np.float32)
    reg[0, 2] = reg[1, 2] = 10.0
    reg[0, 3] = reg[1, 3] = 10.0
    cls = np.full((2016, 1), -100.0, dtype=np.float32)
    cls[0] = 10.0
    cls[1] = 9.0
    dets = decode_palm(reg, cls, palm_anchors(192), 192)
    assert len(dets) == 1


def test_scale_detections_maps_back():
    # letterbox 输入空间 bbox (96,96,144,144) → 原图（scale 0.6, pad (0,24), 240x320）
    dets = scale_detections(
        [{"bbox": [96.0, 96.0, 144.0, 144.0], "keypoints": [], "score": 0.9}],
        0.6, (0, 24), (240, 320))
    x1, y1, x2, y2 = dets[0]["bbox"]
    assert abs(x1 - 160.0) < 1e-3 and abs(y1 - 120.0) < 1e-3
    assert abs(x2 - 240.0) < 1e-3 and abs(y2 - 200.0) < 1e-3


def test_union_bbox_merges_all_dets():
    # 特写大图 palm 分裂 4 框 → union 合并（含帧外裁剪）
    dets = [
        {"bbox": [196.0, 0.0, 409.0, 137.0], "score": 0.9},
        {"bbox": [196.0, 372.0, 409.0, 538.0], "score": 0.9},
        {"bbox": [58.0, 368.0, 276.0, 538.0], "score": 0.8},
        {"bbox": [149.0, 277.0, 364.0, 492.0], "score": 0.6},
    ]
    box = union_bbox(dets, (538, 474))
    assert box == (58, 0, 409, 538)


def test_union_bbox_clips_to_frame():
    dets = [{"bbox": [-50.0, -20.0, 500.0, 600.0], "score": 0.9}]
    box = union_bbox(dets, (200, 300))
    assert box == (0, 0, 300, 200)


def test_union_bbox_empty():
    assert union_bbox([], (100, 100)) == (0, 0, 0, 0)


def test_detection2roi_vertical_hand():
    # 竖直手（腕下中指根上，x 相同）：腕(100,200) 中指根(100,150)
    lm = np.zeros((21, 3), dtype=np.float32)
    lm[0] = (100.0, 200.0, 0.0)
    lm[9] = (100.0, 150.0, 0.0)
    xc, yc, scale, theta = detection2roi(lm)
    assert abs(xc - 100.0) < 1e-3          # 中心 = 腕
    L = 50.0
    assert abs(scale - L * 2 * 2.6) < 1e-3  # 2×dist × dscale
    assert abs(yc - (200.0 - 0.5 * 2 * L)) < 1e-3  # dy 偏移 -0.5×scale
    assert abs(theta + np.pi / 2) < 1e-6   # atan2(中指根-腕)=-π/2（向上，手正立）


def test_detection_box2roi_box_mode():
    # box 模式：中心 = 框中心，scale = 框宽×dscale，dy 偏移
    xc, yc, scale, theta = detection_box2roi((80.0, 40.0, 180.0, 140.0))
    assert abs(xc - 130.0) < 1e-3
    assert abs(scale - 100.0 * 2.6) < 1e-3
    assert abs(yc - (90.0 - 0.5 * 100.0)) < 1e-3
    assert theta == 0.0                     # 无关键点 → theta0=π/2 抵消 → 0


def test_roi_affine_roundtrip():
    # 原图点 → M → 224 空间 → M_inv → 原图（往返一致）
    lm = np.zeros((21, 3), dtype=np.float32)
    lm[0] = (100.0, 200.0, 0.0)
    lm[9] = (100.0, 150.0, 0.0)
    xc, yc, scale, theta = detection2roi(lm)
    m, m_inv = roi_affine(xc, yc, scale, theta, 224)
    pts = np.zeros((3, 3), dtype=np.float32)
    pts[0] = (100.0, 150.0, 0.0)            # 手中心附近
    pts[1] = (80.0, 200.0, 0.0)
    pts[2] = (120.0, 200.0, 3.0)            # z 直通
    back = unproject_landmarks(pts, m_inv, 224)
    # 先正向：224 空间坐标
    fwd = pts[:, :2].astype(np.float64) @ m[:2, :2].T + m[:, 2]
    back2 = unproject_landmarks(np.concatenate([fwd, pts[:, 2:]], axis=1).astype(np.float32),
                                m_inv, 224)
    for i in range(3):
        assert abs(back2[i, 0] - pts[i, 0]) < 1e-2
        assert abs(back2[i, 1] - pts[i, 1]) < 1e-2
        assert abs(back2[i, 2] - pts[i, 2]) < 1e-5   # z 不受仿射影响


def test_detection2roi_zero_length_safe():
    # 腕=中指根 同点 → 不崩（length 0，scale 0）
    lm = np.zeros((21, 3), dtype=np.float32)
    lm[0] = (50.0, 50.0, 0.0)
    lm[9] = (50.0, 50.0, 0.0)
    xc, yc, scale, theta = detection2roi(lm)
    assert scale == 0.0


def test_crop_hand_padding():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    roi, (ox, oy) = crop_hand(img, [40.0, 40.0, 60.0, 60.0])  # 20x20 + 20% pad
    assert (ox, oy) == (36, 36)
    assert roi.shape[:2] == (28, 28)


def test_crop_hand_clips_frame():
    img = np.zeros((50, 50, 3), dtype=np.uint8)
    roi, (ox, oy) = crop_hand(img, [0.0, 0.0, 20.0, 20.0])
    assert ox == 0 and oy == 0
    assert roi.shape[:2] == (24, 24)


def test_crop_hand_centered_pad():
    # (40,40,60,60) 20x20 → 各边 20% = 4px → (36,36)-(64,64)
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    roi, (ox, oy) = crop_hand(img, [40.0, 40.0, 60.0, 60.0])
    assert (ox, oy) == (36, 36)
    assert roi.shape[:2] == (28, 28)


def test_decode_hand_maps_center_no_swap():
    # 输出布局 [x, y, z]（用户重转模型板端实测）：输入 (x=112, y=112) → landmark (112,112)
    joints = np.zeros((21, 3), dtype=np.float32)
    joints[0] = (112.0, 112.0, 0.0)      # [x, y, z] 布局
    outs = [joints, np.array([[[[0.9]]]], dtype=np.float32),
            np.array([[[[0.8]]]], dtype=np.float32),
            np.zeros((1, 63), dtype=np.float32)]
    hand = decode_hand(outs, scale=2.0, pad=(0, 0), orig_shape=(240, 320))
    assert abs(hand.landmarks[0, 0] - 56.0) < 1e-3   # x = raw[:,0]/2
    assert abs(hand.landmarks[0, 1] - 56.0) < 1e-3   # y = raw[:,1]/2
    assert abs(hand.score - 0.9) < 1e-6
    assert hand.is_right                      # 0.8 > 0.5
    assert hand.world.shape == (21, 3)        # world 透传


def test_decode_hand_xy_standard_semantics():
    # [x,y,z]：raw x=30 y=100 → landmark x=30 y=100（直通，不交换）
    joints = np.zeros((21, 3), dtype=np.float32)
    joints[0] = (30.0, 100.0, 0.5)
    outs = [joints, np.array([[[[0.9]]]], dtype=np.float32),
            np.array([[[[0.8]]]], dtype=np.float32),
            np.zeros((1, 63), dtype=np.float32)]
    hand = decode_hand(outs)
    assert abs(hand.landmarks[0, 0] - 30.0) < 1e-3
    assert abs(hand.landmarks[0, 1] - 100.0) < 1e-3
    assert abs(hand.landmarks[0, 2] - 0.5) < 1e-3


def test_decode_hand_left_when_below_half():
    joints = np.zeros((21, 3), dtype=np.float32)
    outs = [joints, np.array([[[[0.9]]]], dtype=np.float32),
            np.array([[[[0.4]]]], dtype=np.float32),
            np.zeros((1, 63), dtype=np.float32)]
    assert not decode_hand(outs).is_right


def test_decode_hand_world_passthrough():
    joints = np.zeros((21, 3), dtype=np.float32)
    world = np.full((1, 63), 0.5, dtype=np.float32)
    outs = [joints, np.array([[[[0.9]]]], dtype=np.float32),
            np.array([[[[0.8]]]], dtype=np.float32), world]
    hand = decode_hand(outs)
    assert abs(hand.world[0, 0] - 0.5) < 1e-6
