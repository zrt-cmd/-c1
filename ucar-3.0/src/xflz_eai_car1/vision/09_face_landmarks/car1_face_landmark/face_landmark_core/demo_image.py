"""图片模式演示（x86 本地验证；不依赖 ROS 也可跑 fake 引擎）。

用法：
  python -m face_landmark_core.demo_image --config config/face_config.yaml [--display] [--save out.jpg]
"""
from __future__ import annotations

import argparse

import cv2

from .config import ConfigError, load_config
from .landmark import FaceLandmarkerEngine, draw_boxes, draw_mesh
from .view import compose, put_status


def demo_image(cfg_path: str, display: bool = False, save: str | None = None,
               engine: FaceLandmarkerEngine | None = None) -> object:
    """读配置+图片 → 检测 → 网格 → 双栏 → 状态栏；返回合成图（H,W,3 uint8）。"""
    cfg = load_config(cfg_path)
    frame = cv2.imread(cfg["source"]["rgb_path"])
    if frame is None:
        raise FileNotFoundError(
            f"cannot read image: {cfg['source']['rgb_path']}")
    if engine is None:
        engine = FaceLandmarkerEngine(
            detector_model=cfg["model"]["detector_model"],
            landmark_model=cfg["model"]["landmark_model"],
            num_faces=cfg["model"]["num_faces"],
            min_detection_confidence=cfg["model"]["min_detection_confidence"],
            core_mask=cfg["backend"]["core_mask"],
        )
    try:
        result = engine.process(frame, timestamp_ms=0)
        det = draw_boxes(frame, result)   # 左栏：检测框（YOLO 风格）
        mesh = draw_mesh(frame, result)   # 右栏：468 点网格
        dst = compose(det, mesh)
        dst = put_status(dst, 0.0, result.num_faces)
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
    parser = argparse.ArgumentParser(description="Face landmark demo (exp09)")
    parser.add_argument("--config", default="config/face_config.yaml")
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
