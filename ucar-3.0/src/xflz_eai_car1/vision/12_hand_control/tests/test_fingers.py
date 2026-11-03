"""手指计数与夹角测试（vendor 语义构造）。"""
from __future__ import annotations

import numpy as np

from hand_control_core.fingers import (
    calc_angle,
    count_fingers,
    fingers_up,
    thumb_index_angle,
    thumb_index_value,
)


def straight_hand():
    """构造全伸直的右手（手掌向下，手指垂直朝上）。"""
    lm = np.zeros((21, 3), dtype=np.float32)
    lm[0] = (50, 150, 0.0)    # 腕
    lm[1] = (50, 130, 0.0)    # 拇指根1
    lm[2] = (50, 115, 0.0)
    lm[3] = (50, 100, 0.0)
    lm[4] = (50, 85, 0.0)     # 拇指尖（伸直 → 角度 180）
    for tip, pip, pip2 in [(8, 6, 5), (12, 10, 9), (16, 14, 13), (20, 18, 17)]:
        lm[pip2] = (50, 120, 0.0)
        lm[pip] = (50, 100, 0.0)
        lm[tip] = (50, 60, 0.0)
    return lm


def test_calc_angle_straight_180():
    lm = straight_hand()
    assert calc_angle(4, 3, 2, lm) > 179.0


def test_calc_angle_right_90():
    lm = straight_hand()
    lm[8] = (150, 100, 0.0)   # 食指尖右移 → 顶点 6 的夹角 ≈ 90
    a = calc_angle(8, 6, 5, lm)
    assert 80 < a < 100


def test_fingers_up_all_extended():
    assert fingers_up(straight_hand()) == [True, True, True, True, True]


def test_index_folded():
    lm = straight_hand()
    lm[8] = (55, 110, 0.0)    # tip.y=110 > pip.y=100 → 弯曲
    assert fingers_up(lm)[1] is False


def test_thumb_folded():
    lm = straight_hand()
    lm[3] = (55, 105, 0.0)
    lm[4] = (60, 110, 0.0)    # 折向手掌 → 角度 < 150
    assert fingers_up(lm)[0] is False


def test_count_fingers():
    assert count_fingers(straight_hand()) == 5
    lm = straight_hand()
    lm[8] = (55, 110, 0.0)
    lm[12] = (55, 110, 0.0)
    assert count_fingers(lm) == 3


def test_thumb_index_angle():
    lm = straight_hand()
    lm[8] = (150, 150, 0.0)   # 食指尖与腕同高右移 → 与拇指方向夹角 90°
    assert abs(thumb_index_angle(lm) - 90.0) < 5.0


def test_thumb_index_value_mapping():
    lm = straight_hand()
    # 完全张开：拇指左、食指右（腕在中间）→ 夹角 180° → 255
    lm[4] = (20, 150, 0.0)
    lm[8] = (100, 150, 0.0)
    assert thumb_index_value(lm) == 255
    # 接近并拢（两指尖几乎同点）→ 夹角 ~0° → 0
    lm[4] = (49.9, 145, 0.0)
    lm[8] = (50.1, 145, 0.0)
    assert thumb_index_value(lm) < 40
