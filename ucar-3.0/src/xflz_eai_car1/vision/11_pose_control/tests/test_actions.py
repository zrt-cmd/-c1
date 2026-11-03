"""动作分类测试（33 点合成姿态 → 控制指令）。"""
from __future__ import annotations

import numpy as np

from pose_control_core.actions import (
    BACKWARD,
    FORWARD,
    STOP,
    TURN_LEFT,
    TURN_RIGHT,
    classify_action,
)

H, W = 240, 320


def make_lm(hip_y=0.5, l_wrist=None, r_wrist=None, shoulder_y=0.4,
            vis_hip=1.0, vis_wrist=1.0):
    """构造 33×4 像素 landmarks；默认站姿（髋 0.5，腕 0.5 = 垂手）。"""
    lm = np.zeros((33, 4), dtype=np.float32)
    lm[:, 3] = 1.0
    lm[11, 1] = lm[12, 1] = shoulder_y * H   # 双肩
    lm[15, 1] = lm[16, 1] = 0.5 * H          # 双腕（默认垂手）
    lm[23, 1] = lm[24, 1] = hip_y * H        # 双髋
    if l_wrist is not None:
        lm[15, 1] = l_wrist * H
    if r_wrist is not None:
        lm[16, 1] = r_wrist * H
    lm[23, 3] = lm[24, 3] = vis_hip
    lm[15, 3] = lm[16, 3] = vis_wrist
    return lm


def test_no_pose_stops():
    assert classify_action(None, H, W) == STOP


def test_standing_stops():
    assert classify_action(make_lm(), H, W) == STOP


def test_squat_backward():
    # 双髋 y=0.8 > 0.72 → 后退（即使举手也优先）
    lm = make_lm(hip_y=0.8, l_wrist=0.3, r_wrist=0.3)
    assert classify_action(lm, H, W) == BACKWARD


def test_both_hands_up_forward():
    lm = make_lm(l_wrist=0.3, r_wrist=0.3)   # 0.3 < 0.4-0.08
    assert classify_action(lm, H, W) == FORWARD


def test_left_up_turns_left():
    lm = make_lm(l_wrist=0.3, r_wrist=0.5)
    assert classify_action(lm, H, W) == TURN_LEFT


def test_right_up_turns_right():
    lm = make_lm(l_wrist=0.5, r_wrist=0.3)
    assert classify_action(lm, H, W) == TURN_RIGHT


def test_hand_barely_low_not_up():
    # 腕 y = 肩 y - 0.08 不满足严格小于 → 不举
    lm = make_lm(l_wrist=0.32, r_wrist=0.32)
    assert classify_action(lm, H, W) == STOP


def test_hip_low_vis_stops():
    lm = make_lm(vis_hip=0.1)
    assert classify_action(lm, H, W) == STOP


def test_wrist_low_vis_ignored():
    # 腕 vis 低 → 不判举手
    lm = make_lm(l_wrist=0.3, vis_wrist=0.1)
    assert classify_action(lm, H, W) == STOP


def test_pixel_coords_normalized():
    # 像素坐标在 240 高帧中 y=72 → 归一化 0.3 → 举手
    lm = make_lm(l_wrist=0.3, r_wrist=0.5)
    lm_px = lm.copy()
    lm_px[:, 1] = lm_px[:, 1] * 0.0 + lm[:, 1]  # 已是像素
    assert classify_action(lm_px, H, W) == TURN_LEFT


def test_labels_are_ascii():
    """cv2.putText 无 CJK 字形：label 必须 ASCII（中文会显示 ????）。"""
    for action in (STOP, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT):
        assert action.label.isascii()
        assert action.label == action.label.strip()
