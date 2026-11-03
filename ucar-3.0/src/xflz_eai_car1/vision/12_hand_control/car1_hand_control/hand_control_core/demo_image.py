"""图片模式演示（x86 本地验证；不依赖 ROS 也可跑 fake 引擎）。

用法：
  PYTHONPATH=car1_hand_control python3 -m hand_control_core.demo_image \
    --config config/hand_control_config.yaml [--display] [--save out.jpg]
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

from .config import ConfigError, load_config
from .engine import HandEngine, draw_boxes, draw_hands
from .fingers import count_fingers
from .view import add_fps, compose, put_status


def demo_image(cfg_path: str, display: bool = False, save: str | None = None,
               engine: HandEngine | None = None) -> object:
    """读配置+图片 → 手部 → 双栏（左=原图+FPS，右=黑底骨架）→ 状态栏。"""
    cfg = load_config(cfg_path)
    frame = cv2.imread(cfg["source"]["rgb_path"])
    if frame is None:
        raise FileNotFoundError(
            f"cannot read image: {cfg['source']['rgb_path']}")
    if engine is None:
        engine = HandEngine(
            detector_model=cfg["model"]["detector_model"],
            landmark_model=cfg["model"]["landmark_model"],
            embedder_model=cfg["model"]["embedder_model"],
            classifier_model=cfg["model"]["classifier_model"],
            max_hands=cfg["model"]["max_hands"],
            min_detection_confidence=cfg["model"]["min_detection_confidence"],
            min_tracking_confidence=cfg["model"]["min_tracking_confidence"],
            core_mask=cfg["backend"]["core_mask"],
        )
    try:
        result = engine.process(frame)
        fingers = count_fingers(result.hands[0].landmarks) if result.hands else 0
        gesture = (result.hands[0].gesture_label if result.hands else "None")
        left = draw_boxes(add_fps(frame, 0.0), result)
        canvas = draw_hands(np.zeros_like(frame), result)
        dst = compose(left, canvas)
        dst = put_status(dst, 0.0, result.num_hands, fingers, "color", gesture)
        print(f"[hand_control] gesture: {gesture}")
        if display:
            cv2.imshow(cfg["display"]["window_name"], dst)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        if save:
            cv2.imwrite(save, dst)
        return dst
    finally:
        if engine is not None:
            engine.close()


def main():
    parser = argparse.ArgumentParser(description="Hand demo (exp12)")
    parser.add_argument("--config", default="config/hand_control_config.yaml")
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--save", default="")
    args = parser.parse_args()

    try:
        demo_image(args.config, display=args.display,
                   save=args.save or None)
    except ConfigError as exc:
        print(f"[demo] config error: {exc}")
        return 2
    except FileNotFoundError as exc:
        print(f"[demo] {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
