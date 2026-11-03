"""手部实时检测 + 图像特效控制（vendor 12_手指控制 移植）。

用法：
  # 板上标准：HP60C 走 ascamera 话题（opencv 直连读不了）
  ros2 launch ascamera hp60c.launch.py        # 先起驱动，ROS_DOMAIN_ID 一致（=2）
  PYTHONPATH=car1_hand_control:$PYTHONPATH python3 -m hand_control_core.camera_demo \
    --config config/hand_control_config.yaml --ros [--display] [--port 8084]
  # 无 ROS 的 UVC 回退：--camera N（V4L2+YUYV；本板 NOVATEK 相机 MJPG 损坏）
双栏（左=特效帧+FPS，右=黑底手骨架 + 拇指食指高亮 + 仪表）；'f' 切特效（5 种）。
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from .config import ConfigError, load_config
from .effects import EFFECTS, apply_effect
from .engine import HandEngine, draw_boxes, draw_hands
from .fingers import count_fingers, thumb_index_value
from .hand import HandResult
from .mjpeg_server import MjpegServer, require
from .ros2_source import Ros2Camera
from .view import add_fps, compose, put_status


class FakeCap:
    """可注入的摄像头替身：迭代 N 帧后 read 返回 (False, None)。"""

    def __init__(self, frames):
        self._frames = list(frames)
        self._idx = 0
        self.opened = True

    def read(self):
        if self._idx >= len(self._frames):
            return False, None
        frame = self._frames[self._idx]
        self._idx += 1
        return True, frame

    def release(self):
        self.opened = False


def draw_gauge(img: np.ndarray, value: int) -> np.ndarray:
    """右栏右下角仪表：value ∈ [0,255] → 竖条 400→150（vendor 12）。"""
    out = img.copy()
    h, w = out.shape[:2]
    bar_h = 250
    x0, y0 = w - 90, h - 40 - bar_h
    cv2.rectangle(out, (x0, y0), (x0 + 35, y0 + bar_h), (255, 0, 0), 2)
    filled = int(round(bar_h * value / 255.0))
    cv2.rectangle(out, (x0, y0 + bar_h - filled), (x0 + 35, y0 + bar_h),
                  (0, 255, 0), -1)
    cv2.putText(out, f"{value}", (x0, y0 + bar_h + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def run_camera(cfg: dict, engine: HandEngine | None = None,
               cap=None, display: bool = False, port: int | None = None,
               stream_name: str = "hand.mjpg", camera_index: int = 0) -> int:
    """实时循环：读帧 → 手部 → 特效（f 切换）→ 双栏 → 状态栏 → imshow/MJPEG。"""
    cap = cap or cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if cap is not None and hasattr(cap, "set") and hasattr(cap, "isOpened"):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
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
    mjpeg = None
    if port is not None:
        mjpeg = MjpegServer(port=port, stream_name=stream_name)
        mjpeg.start()

    fps = 0.0
    p_time = time.time()
    processed = 0
    effect_idx = 0
    effect = EFFECTS[effect_idx]
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                break
            result = engine.process(frame)
            value = 0
            fingers = 0
            if result.hands:
                value = thumb_index_value(result.hands[0].landmarks)
                fingers = count_fingers(result.hands[0].landmarks)
            fx = apply_effect(frame, effect, value)          # 左栏：特效帧
            left = draw_boxes(add_fps(fx, fps), result)
            canvas = draw_hands(np.zeros_like(frame), result)  # 右栏：黑底骨架
            canvas = draw_gauge(canvas, value)
            dst = compose(left, canvas)
            now = time.time()
            dt = now - p_time
            p_time = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            gesture = (result.hands[0].gesture_label if result.hands
                       else "None")
            dst = put_status(dst, fps, result.num_hands, fingers, effect,
                             gesture)
            processed += 1

            if mjpeg is not None:
                mjpeg.set_frame(dst)
            if display:
                cv2.imshow(cfg["display"]["window_name"], dst)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("f"):
                    effect_idx = (effect_idx + 1) % len(EFFECTS)
                    effect = EFFECTS[effect_idx]
    finally:
        engine.close()
        if mjpeg is not None:
            mjpeg.stop()
        cap.release()
        cv2.destroyAllWindows()
    return processed


def main():
    parser = argparse.ArgumentParser(description="Hand control camera demo (exp12)")
    parser.add_argument("--config", default="config/hand_control_config.yaml")
    parser.add_argument("--camera", type=int, default=0,
                        help="V4L2 摄像头设备号（v4l2-ctl --list-devices 查；板上常见 0/1/2/25/26）")
    parser.add_argument("--ros", action="store_true",
                        help="ROS2 话题取流（ascamera HP60C；需先起驱动，ROS_DOMAIN_ID 一致）")
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--port", type=int, default=8084, help="MJPEG port (0 = off)")
    args = parser.parse_args()
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"[camera_demo] config error: {exc}")
        return 2
    if args.ros:
        ros_cfg = (cfg.get("source") or {}).get("ros") or {}
        cap = Ros2Camera(ros_cfg.get("rgb_topic", Ros2Camera.DEFAULT_TOPIC))
        cap.open()
        ok, _ = cap.read(timeout=float(ros_cfg.get("timeout", 5.0)))
        if not ok:
            print("[camera_demo] cannot open ros topic (driver running? ROS_DOMAIN_ID match?)")
            cap.release()
            return 2
        return run_camera(cfg, cap=cap, display=args.display, port=args.port or None)
    run_camera(cfg, display=args.display, port=args.port or None,
               camera_index=args.camera)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
