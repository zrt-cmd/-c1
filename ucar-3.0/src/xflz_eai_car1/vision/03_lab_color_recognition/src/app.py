"""Lab 颜色识别 CLI（构建文档 03）。

用法（板上 ros2_venv）：
    python src/app.py --ros                                # 相机 + 左原图|右处理双栏 + MJPEG :8080
    python src/app.py --video test.mp4 --port 8081
    python src/app.py --image x.jpg --save out.jpg       # 单图
    python src/app.py --tune 红色 --ros                   # 调参（需显示器；滑块调 L/a/b，Ctrl+C 退出）
MJPEG 流：http://<板 IP>:8080/lab_color.mjpg（双栏拼接图）
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from config import load_colors, load_config
from mjpeg_server import MjpegServer, require
from preprocess import (auto_hsv_range, bgr_to_hsv, color_mask, detect_colors,
                        draw_detection, mask_view)
from ros2_source import Ros2Camera


def process_frame(bgr, cfg, colors):
    """单帧：HSV 检出 → RGB 栏（绿轮廓/黄框/红质心/颜色名）+ HSV Mask 栏（白匹配区）。"""
    hsv = bgr_to_hsv(bgr)
    dets = detect_colors(bgr, cfg, colors)
    left = draw_detection(bgr, dets)
    right = mask_view(color_mask(hsv, cfg, colors), dets)
    return dual_view(left, right)


def dual_view(bgr, processed, sep: int = 2) -> np.ndarray:
    """左原图 | 右处理后（绿分隔线）双栏拼接；显示与推流统一用。"""
    gap = np.zeros((bgr.shape[0], sep, 3), dtype=np.uint8)
    gap[:, :, 1] = 255  # 绿色分隔线
    return np.hstack([bgr, gap, processed])


def _tune_mode(color_name: str, colors, cfg, cap, port: int | None = None):
    """LAB TOOL 调参（需显示器）：6 滑块实时调 H/S/V 上下限。

    左原图|右处理双栏窗口 + MJPEG 推流；无按键设置（滑块即界面），
    Ctrl+C 退出并打印当前阈值（写入 config 生效）。cap 可注入（--ros 或 V4L2）。
    """
    from config import _parse_channel  # 复用校验

    spec = next((c for c in colors if c.name == color_name), None)
    if spec is None:
        print(f"颜色 {color_name!r} 不在配置中: {[c.name for c in colors]}")
        return
    win = "lab_tune"
    view_win = "lab_view"
    cv2.namedWindow(win)
    cv2.namedWindow(view_win)
    cur = {"h": list(spec.h), "s": list(spec.s), "v": list(spec.v)}
    ok0, frame0 = cap.read()
    if ok0 and frame0 is not None:
        auto = auto_hsv_range(frame0, spec)
        if auto is not None:
            cur.update(auto)
            print(f"[tune] 自动估计 {spec.name} 范围: H{cur['h']} S{cur['s']} V{cur['v']}（滑块已预置，可微调）")

    def on_trackbar(_):
        for i, ch in enumerate(("h", "s", "v")):
            lo = cv2.getTrackbarPos(f"{ch} min", win)
            hi = cv2.getTrackbarPos(f"{ch} max", win)
            if lo <= hi:
                cur[ch] = [lo, hi]

    for ch in ("h", "s", "v"):
        max_pos = 179 if ch == "h" else 255  # H∈0-179，S/V∈0-255
        cv2.createTrackbar(f"{ch} min", win, cur[ch][0], max_pos, on_trackbar)
        cv2.createTrackbar(f"{ch} max", win, cur[ch][1], max_pos, on_trackbar)

    server = MjpegServer("0.0.0.0", port or 8080, stream_name="lab_color").start()
    info = ""
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            spec2 = type(spec)(spec.name, tuple(cur["h"]), tuple(cur["s"]),
                               tuple(cur["v"]), spec.h2)
            dets = detect_colors(frame, cfg, [spec2])
            hsv = bgr_to_hsv(frame)
            left = draw_detection(frame, dets)
            right = mask_view(color_mask(hsv, cfg, [spec2]), dets)
            info = f"{spec.name}: H{cur['h']} S{cur['s']} V{cur['v']}"
            cv2.putText(right, info, (16, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2)
            view = dual_view(left, right)
            server.set_frame(view)
            cv2.imshow(view_win, view)
            cv2.waitKey(1)  # 仅刷新窗口，不绑定按键
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        server.stop()
        cv2.destroyAllWindows()
    print("调参结果（写入 config/lab_config.yaml 生效）:")
    print(info)


def main() -> int:
    ap = argparse.ArgumentParser(description="lab color recognition demo")
    ap.add_argument("--image", default=None)
    ap.add_argument("--video", default=None)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--config", default="config/lab_config.yaml")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--tune", default=None, metavar="颜色名", help="LAB TOOL 调参（需显示器）")
    ap.add_argument("--save", default=None)
    ap.add_argument("--display", action="store_true", help="本地窗口（需显示器）")
    ap.add_argument("--ros", action="store_true", help="从 ROS2 话题取流（ascamera HP60C）")
    args = ap.parse_args()
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    cfg = load_config(args.config)
    colors = load_colors(cfg)

    if args.tune is not None:
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
            cap = cv2.VideoCapture(args.camera if args.camera is not None
                                   else cfg["camera"]["index"])
            if not cap.isOpened():
                print(f"camera open failed (index={args.camera or cfg['camera']['index']})")
                return 2
        _tune_mode(args.tune, colors, cfg, cap=cap, port=args.port)
        return 0

    src = args.camera if args.camera is not None else (args.video if args.video else args.image)
    if src is None and not args.ros:
        ap.print_help()
        return 2

    if args.image is not None:
        img = cv2.imread(args.image)
        if img is None:
            print(f"cannot read image: {args.image}")
            return 2
        out = process_frame(img, cfg, colors)
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
        pass  # exp03 无视频写出；--save 仅单图模式使用
    elif args.save:
        print("note: --save 在 --ros 模式不支持，忽略")

    server = MjpegServer("0.0.0.0", args.port or 8080, stream_name="lab_color").start()
    frames, loop_t, init_t = 0, time.time(), time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames += 1
        out = process_frame(frame, cfg, colors)
        view = dual_view(frame, out)
        server.set_frame(view)
        if args.display:
            cv2.imshow(cfg["display"]["window_name"], view)
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
