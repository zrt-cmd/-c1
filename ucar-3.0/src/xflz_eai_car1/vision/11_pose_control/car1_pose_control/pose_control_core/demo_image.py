"""图片模式演示（x86 本地验证；不依赖 ROS 也可跑 fake 引擎）。

用法：
  PYTHONPATH=car1_pose_control python3 -m pose_control_core.demo_image \
    --config config/pose_config.yaml [--display] [--save out.jpg]
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

from .actions import classify_action
from .config import ConfigError, load_config
from .engine import PoseEngine, draw_skeleton
from .view import add_fps, compose, put_status


def demo_image(cfg_path: str, display: bool = False, save: str | None = None,
               engine: PoseEngine | None = None) -> object:
    """读配置+图片 → 姿态 → 双栏（左=原图+FPS，右=黑底骨架）→ 状态栏。"""
    cfg = load_config(cfg_path)
    frame = cv2.imread(cfg["source"]["rgb_path"])
    if frame is None:
        raise FileNotFoundError(
            f"cannot read image: {cfg['source']['rgb_path']}")
    if engine is None:
        engine = PoseEngine(
            pose_model=cfg["model"]["pose_model"],
            min_detection_confidence=cfg["model"]["min_detection_confidence"],
            core_mask=cfg["backend"]["core_mask"],
        )
    try:
        result = engine.process(frame)
        action = classify_action(result.landmarks, *frame.shape[:2])
        left = add_fps(frame, 0.0)
        skeleton = draw_skeleton(np.zeros_like(frame), result)
        dst = compose(left, skeleton)
        dst = put_status(dst, 0.0, action.label)
        print(f"[pose_control] action: {action.name} ({action.label})")
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
    parser = argparse.ArgumentParser(description="Pose demo (exp10)")
    parser.add_argument("--config", default="config/pose_config.yaml")
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
