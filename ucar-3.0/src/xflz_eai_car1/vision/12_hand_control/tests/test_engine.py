"""引擎 4 模型管线测试（fake 后端注入：det→crop→lm→emb→clf 全链路）。"""
from __future__ import annotations

import numpy as np

from hand_control_core.backend import FakeBackend
from hand_control_core.engine import HandEngine, draw_hands
from hand_control_core.hand import Hand, HandResult, palm_anchors


def det_outputs(anchor=0, score=10.0, reg0=92.0, reg1=92.0, w=38.4, h=38.4):
    """构造手掌检测输出：锚 anchor 回归到 192 空间 (96,96) 附近。"""
    reg = np.zeros((2016, 18), dtype=np.float32)
    cls = np.full((2016, 1), -100.0, dtype=np.float32)
    reg[anchor] = (reg0, reg1, w, h, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0)
    cls[anchor] = score
    return [reg, cls]


def lm_outputs(score=0.9, joint0=(112.0, 112.0, 0.0)):
    joints = np.zeros((21, 3), dtype=np.float32)
    joints[0] = joint0
    return [joints, np.array([[[[score]]]], dtype=np.float32),
            np.array([[[[0.8]]]], dtype=np.float32),
            np.zeros((1, 63), dtype=np.float32)]


def emb_outputs():
    return [np.zeros((1, 128), dtype=np.float32)]


def clf_outputs(gesture_idx=6):
    out = np.zeros((1, 8), dtype=np.float32)
    out[0, gesture_idx] = 5.0
    return [out]


def make_engine(**kwargs):
    det = FakeBackend(fn=lambda img: det_outputs())
    lm = FakeBackend(fn=lambda img: lm_outputs())
    emb = FakeBackend(fn=lambda img: emb_outputs())
    clf = FakeBackend(fn=lambda img: clf_outputs())
    eng = HandEngine("det.rknn", "lm.rknn", "emb.rknn", "clf.rknn",
                     create_fn=lambda: (det, lm, emb, clf), **kwargs)
    return eng, (det, lm, emb, clf)


def test_engine_lazy_four_stage():
    eng, backends = make_engine()
    assert eng._det is None
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.num_hands == 1


def test_engine_center_lands_in_frame_center():
    # 320x240 帧 letterbox 192：scale 0.6 pad (0,24)；palm bbox 中心 (160,120)
    # 检测模式 union 矩形 + letterbox 224 → landmark 中心映射回 (160,120)
    eng, _ = make_engine()
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    hand = res.hands[0]
    assert abs(hand.landmarks[0, 0] - 160.0) < 2.0
    assert abs(hand.landmarks[0, 1] - 120.0) < 2.0
    assert hand.is_right


def test_engine_tracking_skips_detector():
    # 官方状态机：第 1 帧 detection（palm 1 次）→ 第 2 帧 tracking（palm 不跑）
    calls = {"det": 0}
    def det_fn(img):
        calls["det"] += 1
        return det_outputs()
    det = FakeBackend(fn=det_fn)
    lm = FakeBackend(fn=lambda img: lm_outputs())
    emb = FakeBackend(fn=lambda img: emb_outputs())
    clf = FakeBackend(fn=lambda img: clf_outputs())
    eng = HandEngine("d", "l", "e", "c", create_fn=lambda: (det, lm, emb, clf))
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    res1 = eng.process(frame)
    assert calls["det"] == 1 and res1.num_hands == 1
    res2 = eng.process(frame)
    assert calls["det"] == 1      # tracking 帧跳过 palm
    assert res2.num_hands == 1
    assert res2.hands[0].box is None   # tracking 无检测框（box 仅 detection 填充）


def test_engine_tracking_falls_back_on_low_presence():
    # presence 丢失 → 下帧回退全图检测（官方状态机）
    calls = {"det": 0}
    def det_fn(img):
        calls["det"] += 1
        return det_outputs()
    det = FakeBackend(fn=det_fn)
    lm = FakeBackend(fn=lambda img: lm_outputs(score=0.3))   # 第 2 帧 presence 低
    emb = FakeBackend(fn=lambda img: emb_outputs())
    clf = FakeBackend(fn=lambda img: clf_outputs())
    eng = HandEngine("d", "l", "e", "c", create_fn=lambda: (det, lm, emb, clf))
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    assert eng.process(frame).num_hands == 0     # 第 1 帧 presence 就低
    assert calls["det"] == 1
    # 换成高 presence 的 lm → 第 2 帧 detection（上帧无跟踪态）
    lm._fn = lambda img: lm_outputs(score=0.9)
    assert eng.process(frame).num_hands == 1
    assert calls["det"] == 2
    # 第 3 帧 tracking（presence 达标）——lm 再回低 → presence 丢失 → 空
    lm._fn = lambda img: lm_outputs(score=0.3)
    assert eng.process(frame).num_hands == 0
    assert calls["det"] == 2      # tracking 帧不跑 palm；下帧回退
    res = eng.process(frame)
    assert calls["det"] == 3      # 丢失后回退 detection


def test_engine_lost_tracking_resets():
    eng, _ = make_engine()
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    eng.process(frame)
    assert eng._last_landmarks is not None
    eng.close()
    assert eng._last_landmarks is None   # close 清空跟踪态


def test_engine_gesture_classified():
    eng, _ = make_engine()
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.hands[0].gesture_label == "Victory"
    assert res.hands[0].gesture_score > 0.95


def test_engine_gesture_label_flow():
    det = FakeBackend(fn=lambda img: det_outputs())
    lm = FakeBackend(fn=lambda img: lm_outputs())
    emb = FakeBackend(fn=lambda img: emb_outputs())
    clf = FakeBackend(fn=lambda img: clf_outputs(gesture_idx=2))  # Open_Palm
    eng = HandEngine("d", "l", "e", "c", create_fn=lambda: (det, lm, emb, clf))
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.hands[0].gesture_label == "Open_Palm"


def test_engine_hand_score_filtered():
    lm = FakeBackend(fn=lambda img: lm_outputs(score=0.3))
    det = FakeBackend(fn=lambda img: det_outputs())
    emb = FakeBackend(fn=lambda img: emb_outputs())
    clf = FakeBackend(fn=lambda img: clf_outputs())
    eng = HandEngine("d", "l", "e", "c", create_fn=lambda: (det, lm, emb, clf))
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.num_hands == 0          # presence 0.3 < 0.5


def test_engine_no_detection_empty():
    det = FakeBackend(fn=lambda img: det_outputs(score=-100.0))
    lm = FakeBackend(fn=lambda img: lm_outputs())
    emb = FakeBackend(fn=lambda img: emb_outputs())
    clf = FakeBackend(fn=lambda img: clf_outputs())
    eng = HandEngine("d", "l", "e", "c", create_fn=lambda: (det, lm, emb, clf))
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.num_hands == 0


def test_engine_close_idempotent():
    eng, backends = make_engine()
    eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    eng.close()
    assert eng._det is None and eng._clf is None
    eng.close()


def test_engine_stores_core_mask():
    eng = HandEngine("d", "l", "e", "c", core_mask=7)
    assert eng._core_mask == 7
    assert eng._create_fn is None


def test_draw_boxes_paints_palm_box():
    from hand_control_core.engine import draw_boxes
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    lm = np.zeros((21, 3), dtype=np.float32)
    res = HandResult(hands=[Hand(landmarks=lm, score=0.9, handedness=0.8,
                                 box=(10, 20, 90, 80),
                                 gesture_label="Open_Palm", gesture_score=0.9)])
    out = draw_boxes(img, res)
    assert tuple(out[19, 50]) == (0, 255, 0)   # 框线
    assert not np.array_equal(out, img)


def test_draw_boxes_blank_without_box():
    from hand_control_core.engine import draw_boxes
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    res = HandResult(hands=[Hand(landmarks=np.zeros((21, 3)), score=0.9,
                                 handedness=0.8)])
    assert np.array_equal(draw_boxes(img, res), img)


def test_engine_hand_has_box():
    eng, _ = make_engine()
    res = eng.process(np.zeros((240, 320, 3), dtype=np.uint8))
    assert res.hands[0].box is not None
    x1, y1, x2, y2 = res.hands[0].box
    assert 0 <= x1 < x2 <= 320 and 0 <= y1 < y2 <= 240


def test_draw_hands_blank_when_empty():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    out = draw_hands(img, HandResult())
    assert np.array_equal(out, img)


def test_draw_hands_paints_red_points_green_lines():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    lm = np.zeros((21, 3), dtype=np.float32)
    lm[0] = (50, 90, 0.0)
    lm[5] = (50, 50, 0.0)
    lm[8] = (50, 20, 0.0)
    res = HandResult(hands=[Hand(landmarks=lm, score=0.9, handedness=0.8)])
    out = draw_hands(img, res)
    assert tuple(out[20, 50]) == (0, 0, 255)   # 红点
    assert tuple(out[70, 50]) == (0, 255, 0)   # 绿线（无点覆盖处）


def test_draw_hands_gesture_label_text():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    lm = np.zeros((21, 3), dtype=np.float32)
    lm[:, 0] = 50
    lm[:, 1] = np.linspace(30, 90, 21)
    res = HandResult(hands=[Hand(landmarks=lm, score=0.9, handedness=0.8,
                                 gesture_label="Victory", gesture_score=0.99)])
    out = draw_hands(img, res)
    assert not np.array_equal(out, img)
    assert np.any(out[:, :, 2] > 100)          # 蓝色标签文字
