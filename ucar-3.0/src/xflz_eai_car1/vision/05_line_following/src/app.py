"""多边形轮廓应用 CLI（构建文档 03）。

内容 = 亚博课程「OpenCV应用——多边形轮廓」（课程实验名：自主巡线，可后续改名）。
用法（板上 ros2_venv）：
    python src/app.py --camera 0                  # 相机 + MJPEG :8080
    python src/app.py --video test.mp4
    python src/app.py --image x.jpg --save out.jpg
MJPEG 流：http://<板 IP>:8080/polygon.mjpg
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from config import load_config
from mjpeg_server import MjpegServer, require
from pipeline import detect_polygons, draw_polygons, draw_polygons_dark
from ros2_source import Ros2Camera


def process_frame(bgr, cfg):
    """单帧：多边形轮廓检测 + 标注（彩色黑底或原图叠加）。"""
    results = detect_polygons(bgr, cfg)
    colored = (cfg.get("display") or {}).get("colored_mode", True)
    if colored:
        right = draw_polygons_dark(bgr, results)
        # 双栏：左=原图，右=黑底彩色轮廓（与其它实验风格一致）
        return np.hstack([bgr, right])
    return draw_polygons(bgr, results)


def main() -> int:
    ap = argparse.ArgumentParser(description="polygon contour demo")
    ap.add_argument("--image", default=None)
    ap.add_argument("--video", default=None)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--ros", action="store_true",
                    help="ROS2 话题取流（ascamera HP60C；需先起驱动，ROS_DOMAIN_ID 一致）")
    ap.add_argument("--config", default="config/line_config.yaml")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--save", default=None)
    ap.add_argument("--display", action="store_true", help="本地窗口（需显示器）")
    args = ap.parse_args()
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    cfg = load_config(args.config)
    src = args.camera if args.camera is not None else (args.video if args.video else args.image)
    if src is None and not args.ros:
        ap.print_help()
        return 2

    if args.image is not None:
        img = cv2.imread(args.image)
        if img is None:
            print(f"cannot read image: {args.image}")
            return 2
        out = process_frame(img, cfg)
        if args.save:
            cv2.imwrite(args.save, out)
            print(f"saved -> {args.save}")
        elif args.display:
            cv2.imshow(cfg["display"]["window_name"], out)
            cv2.waitKey(0)
        else:
            print("use --save or --display for single image")
            return 2
        return 0

    if args.ros:
        ros_cfg = (cfg.get("source") or {}).get("ros") or {}
        cap = Ros2Camera(ros_cfg.get("rgb_topic", Ros2Camera.DEFAULT_TOPIC))
        cap.open()
        ok, _ = cap.read(timeout=float(ros_cfg.get("timeout", 5.0)))
        if not ok:
            print("cannot open ros topic (driver running? ROS_DOMAIN_ID match?)")
            cap.release()
            return 2
    else:
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            print(f"cannot open source: {src}")
            return 2

    if args.save and not args.ros:
        print("note: --save 在 --ros 模式不支持，忽略")
    server = MjpegServer("0.0.0.0", args.port or 8080, stream_name="polygon").start()
    frames, loop_t, init_t = 0, time.time(), time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames += 1
        out = process_frame(frame, cfg)
        server.set_frame(out)
        if args.display:
            cv2.imshow(cfg["display"]["window_name"], out)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if frames % 30 == 0:
            print(f"30帧平均帧率: {30 / (time.time() - loop_t):.1f}")
            loop_t = time.time()
    print(f"总平均帧率: {frames / (time.time() - init_t):.1f}")
    cap.release()
    server.stop()
    if args.display:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
