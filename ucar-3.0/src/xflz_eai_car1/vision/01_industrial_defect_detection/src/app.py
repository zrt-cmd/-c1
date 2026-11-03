"""推理 CLI：单图 / 视频 / 相机，画框标注 + FPS。

用法（板载 ros2_venv）：
    python src/app.py --model models/defect_yolov8n_int8.rknn --image xxx.jpg [--save out.jpg]
    python src/app.py --model models/defect_yolov8n_int8.rknn --video test.mp4
    python src/app.py --model models/defect_yolov8n_int8.rknn --camera 0
后端按扩展名自动选择：.rknn -> RKNNBackend，.onnx -> ONNXBackend。
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

from config import load_config
from dataset import class_names
from infer import Detector, DetectorConfig, RKNNBackend, ONNXBackend
from mjpeg_server import MjpegServer, require
from ros2_source import Ros2Camera


def _make_backend(model: str):
    if model.endswith(".rknn"):
        return RKNNBackend()
    if model.endswith(".onnx"):
        return ONNXBackend()
    raise ValueError(f"unsupported model type: {model}")


def draw(img, dets, names):
    for d in dets:
        x1, y1, x2, y2 = int(d.x1), int(d.y1), int(d.x2), int(d.y2)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        label = f"{names[d.class_id]} {d.score:.2f}"
        cv2.putText(img, label, (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return img


def main() -> int:
    ap = argparse.ArgumentParser(description="defect detection demo")
    ap.add_argument("--model", required=True, help=".rknn 或 .onnx")
    ap.add_argument("--image", default=None, help="单图推理")
    ap.add_argument("--video", default=None, help="视频推理")
    ap.add_argument("--camera", type=int, default=None, help="相机索引")
    ap.add_argument("--ros", action="store_true",
                    help="ROS2 话题取流（ascamera HP60C；需先起驱动，ROS_DOMAIN_ID 一致）")
    ap.add_argument("--display", action="store_true", help="本地窗口（需显示器；q 退出）")
    ap.add_argument("--port", type=int, default=None, help="MJPEG 端口（默认 8080）")
    ap.add_argument("--save", default=None, help="输出图片/视频路径")
    ap.add_argument("--config", default="config/defect_config.yaml")
    args = ap.parse_args()
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    cfg = load_config(args.config)
    inf = cfg["infer"]
    try:
        names = class_names(Path(cfg["dataset"]["root"]) / cfg["dataset"]["data_yaml"])
    except (FileNotFoundError, OSError):
        # 板上无训练数据集：NEU-DET 6 类名固定（板上运行无需数据集）
        names = ["crazing", "inclusion", "patches",
                 "pitted_surface", "rolled-in_scale", "scratches"]
        print("note: NEU-DET data.yaml 不在本机，使用内置 6 类名")

    det_cfg = DetectorConfig(imgsz=inf["imgsz"], score_thr=inf["score_thr"],
                             iou_thr=inf["iou_thr"], pad_color=tuple(inf["pad_color"]))
    detector = Detector(_make_backend(args.model), len(names), det_cfg).load(args.model)

    def run_frame(img) -> tuple[cv2.Mat, float]:
        t0 = time.perf_counter()
        dets = detector.detect(img)
        dt = time.perf_counter() - t0
        return draw(img, dets, names), 1.0 / dt if dt > 0 else 0.0

    if args.image is not None:
        img = cv2.imread(args.image)
        if img is None:
            print(f"cannot read image: {args.image}")
            return 2
        out, fps = run_frame(img)
        print(f"fps={fps:.1f}")
        for d in detector.detect(cv2.imread(args.image)):
            print(f"  {names[d.class_id]} {d.score:.3f} ({d.x1:.0f},{d.y1:.0f},{d.x2:.0f},{d.y2:.0f})")
        if args.save:
            cv2.imwrite(args.save, out)
            print(f"saved -> {args.save}")
        else:
            cv2.imshow("defect", out)
            cv2.waitKey(0)
        return 0

    if args.video is not None or args.camera is not None or args.ros:
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
            src = args.camera if args.camera is not None else args.video
            cap = cv2.VideoCapture(src)
            if not cap.isOpened():
                print(f"cannot open source: {src}")
                return 2
        writer = None
        if args.save and not args.ros:
            fps_in = cap.get(cv2.CAP_PROP_FPS) or 25.0
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), fps_in, (w, h))
        elif args.save:
            print("note: --save 在 --ros 模式不支持，忽略")
        n, acc = 0, 0.0
        srv_cfg = cfg.get("server") or {}
        server = MjpegServer("0.0.0.0", args.port or srv_cfg.get("port", 8080),
                             stream_name=srv_cfg.get("stream_name", "defect.mjpg")).start()
        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                out, fps = run_frame(frame)
                n += 1
                acc += fps
                if n % 30 == 0:
                    print(f"frame {n}  avg_fps={acc / n:.1f}")
                if writer is not None:
                    writer.write(out)
                server.set_frame(out)
                if args.display:
                    cv2.imshow("defect", out)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        finally:
            server.stop()
        cap.release()
        if writer is not None:
            writer.release()
            print(f"saved -> {args.save}")
        print(f"done: {n} frames, avg_fps={acc / max(n, 1):.1f}")
        return 0

    ap.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
