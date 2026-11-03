"""二维码生成/识别 CLI（构建文档 03）。

用法（板上 ros2_venv）：
    python src/app.py --generate "https://example.com" --save my_qr.png
    python src/app.py --detect --image my_qr.png
    python src/app.py --detect --camera 0             # 实时识别 + MJPEG :8080
    python src/app.py --detect --video test.mp4
MJPEG 流：http://<板 IP>:8080/qrcode.mjpg
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

from config import load_config
from mjpeg_server import MjpegServer, require
from ros2_source import Ros2Camera
from qr import detect_qr, draw_qr, generate_qr


def _print_results(results):
    for r in results:
        print(f"[QR] {r.text}")


def main() -> int:
    ap = argparse.ArgumentParser(description="qrcode generate/detect demo")
    ap.add_argument("--generate", default=None, metavar="文本", help="生成二维码")
    ap.add_argument("--save", default=None, help="生成/保存路径")
    ap.add_argument("--detect", action="store_true", help="识别模式")
    ap.add_argument("--image", default=None)
    ap.add_argument("--video", default=None)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--ros", action="store_true",
                    help="ROS2 话题取流（ascamera HP60C；需先起驱动，ROS_DOMAIN_ID 一致）")
    ap.add_argument("--config", default="config/qrcode_config.yaml")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--display", action="store_true", help="本地窗口（需显示器）")
    args = ap.parse_args()
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    cfg = load_config(args.config)

    # ---- 生成模式 ----------------------------------------------------
    if args.generate is not None:
        if not args.generate.strip():
            print("--generate 文本不能为空")
            return 2
        if not args.save:
            print("--generate 需要 --save <路径>")
            return 2
        img = generate_qr(args.generate, cfg)
        out = Path(args.save)
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), img)
        print(f"QR 已保存: {out.resolve()}")
        if args.display:
            cv2.imshow("generated", img)
            cv2.waitKey(0)
        return 0

    # ---- 识别模式 ----------------------------------------------------
    if not args.detect:
        ap.print_help()
        return 2
    src = args.camera if args.camera is not None else (args.video if args.video else args.image)
    if src is None and not args.ros:
        ap.print_help()
        return 2

    if args.image is not None:
        img = cv2.imread(args.image)
        if img is None:
            print(f"cannot read image: {args.image}")
            return 2
        results = detect_qr(img, cfg["detect"]["scale"])
        _print_results(results)
        out = draw_qr(img, results)
        if args.save:
            cv2.imwrite(args.save, out)
            print(f"saved -> {args.save}")
        elif args.display:
            cv2.imshow(cfg["display"]["window_name"], out)
            cv2.waitKey(0)
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
    server = MjpegServer("0.0.0.0", args.port or 8080, stream_name="qrcode").start()
    frames, loop_t, init_t = 0, time.time(), time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames += 1
        results = detect_qr(frame, cfg["detect"]["scale"])
        if results:
            _print_results(results)
        out = draw_qr(frame, results)
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
