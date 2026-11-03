"""摄像头实时检测（vendor 06_FaceLandmarks main 形态）。

用法：
  # 板上标准：HP60C 深度相机走 ascamera 话题（opencv 直连读不了）
  ros2 launch ascamera hp60c.launch.py        # 先起驱动，ROS_DOMAIN_ID 一致（=2）
  python -m face_landmark_core.camera_demo --config config/face_config.yaml --ros [--display] [--port 8081]
  # 无 ROS 的 UVC 回退：--camera N（V4L2+YUYV；本板 NOVATEK 相机 MJPG 损坏）
双栏 + 实时 FPS + MJPEG。
"""
from __future__ import annotations

import argparse
import time

import cv2

from .config import ConfigError, load_config
from .landmark import FaceLandmarkerEngine, draw_boxes, draw_mesh
from .mjpeg_server import MjpegServer, require
from .ros2_source import Ros2Camera
from .view import compose, put_status


class FakeCap:
    """可注入的摄像头替身：迭代 N 帧后 read 返回 (False, None)。"""

    def __init__(self, frames):
        self._frames = list(frames)
        self._idx = 0
        self.opened = True

    def read(self):
        if self._idx >= len(self._frames):
            self.opened = False
            return False, None
        frame = self._frames[self._idx]
        self._idx += 1
        return True, frame

    def release(self):
        self.opened = False


def run_camera(cfg: dict, engine: FaceLandmarkerEngine | None = None,
               cap=None, display: bool = False, port: int | None = None,
               stream_name: str = "face.mjpg", camera_index: int = 0) -> int:
    """实时循环：读帧 → 检测 → 双栏 → 状态栏 → imshow/MJPEG；q 退出。

    返回处理帧数。cap/engine 可注入（测试）；未注入 cap 时用
    VideoCapture(camera_index, CAP_V4L2)。

    2026-08-06 板上实测（ASJ ZNX_NVT / NOVATEK UVC）：强制 V4L2 后端 +
    YUYV 原生格式（1280x1040@8）才有正常彩图；该相机 MJPG 流损坏
    （Corrupt JPEG），默认后端协商会出雪花/打不开。设备号用
    v4l2-ctl --list-devices 查。进程崩溃后设备可能卡死 → 重插 USB。
    """
    cap = cap or cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if cap is not None and hasattr(cap, "set") and hasattr(cap, "isOpened"):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    if engine is None:
        engine = FaceLandmarkerEngine(
            detector_model=cfg["model"]["detector_model"],
            landmark_model=cfg["model"]["landmark_model"],
            num_faces=cfg["model"]["num_faces"],
            min_detection_confidence=cfg["model"]["min_detection_confidence"],
            core_mask=cfg["backend"]["core_mask"],
        )
    mjpeg = None
    if port is not None:
        mjpeg = MjpegServer(port=port, stream_name=stream_name)
        mjpeg.start()

    fps = 0.0
    p_time = time.time()
    processed = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                break
            result = engine.process(frame)
            det = draw_boxes(frame, result)   # 左栏：检测框（YOLO 风格）
            mesh = draw_mesh(frame, result)   # 右栏：468 点网格
            dst = compose(det, mesh)
            now = time.time()
            dt = now - p_time
            p_time = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            dst = put_status(dst, fps, result.num_faces)
            processed += 1

            if mjpeg is not None:
                mjpeg.set_frame(dst)
            if display:
                cv2.imshow(cfg["display"]["window_name"], dst)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        engine.close()
        if mjpeg is not None:
            mjpeg.stop()
        cap.release()
        cv2.destroyAllWindows()
    return processed


def main():
    parser = argparse.ArgumentParser(description="Face landmark camera demo (exp09)")
    parser.add_argument("--config", default="config/face_config.yaml")
    parser.add_argument("--camera", type=int, default=0,
                        help="V4L2 摄像头设备号（v4l2-ctl --list-devices 查；板上常见 0/1/2/25/26）")
    parser.add_argument("--ros", action="store_true",
                        help="ROS2 话题取流（ascamera HP60C；需先起驱动，ROS_DOMAIN_ID 一致）")
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--port", type=int, default=8081, help="MJPEG port (0 = off)")
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
