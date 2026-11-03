"""语义分割背景替换 CLI（构建文档 07）。

用法（板上 ros2_venv）：
    python src/app.py --camera 0                  # 相机 + 多线程池 + MJPEG :8080/seg.mjpg
    python src/app.py --video test.mp4
    python src/app.py --image x.jpg --save out.jpg
    # 板载标准：HP60C 深度相机走 ascamera 话题（ros2_venv opencv 未编 libv4l，直连失败）
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
    python src/app.py --ros
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import cv2

from background import compose
from config import load_config
from mjpeg_server import MjpegServer, require
from rknnpool import RKNNPool, backend_factory
from preprocess import letterbox
from postprocess import decode_outputs, foreground_mask
from ros2_source import Ros2Camera

def render(bgr, outputs, cfg):
    """13 输出 -> (合成图, 信息 dict)。掩码为 imgsz 模型空间，映射回原图。无检测整图置背景。"""
    imgsz = cfg["model"]["imgsz"]
    res = decode_outputs(outputs, cfg, imgsz=imgsz)
    fg = foreground_mask(res, cfg["model"]["classes"])
    if fg is None:
        fg = np.zeros(bgr.shape[:2], dtype=bool)
    else:
        lb = letterbox(bgr, imgsz)
        h, w = bgr.shape[:2]
        region = fg[lb.pad_y:lb.pad_y + int(round(h * lb.ratio)),
                    lb.pad_x:lb.pad_x + int(round(w * lb.ratio))]
        fg = cv2.resize(region.astype(np.uint8), (w, h),
                        interpolation=cv2.INTER_NEAREST).astype(bool)
    out = compose(bgr, fg, cfg)
    n = len(res.boxes) if res is not None else 0
    return out, {"det": n}

def _pool_process(backend, frame):
    return backend.predict(frame)


def main() -> int:
    ap = argparse.ArgumentParser(description="semantic seg background demo")
    ap.add_argument("--image", default=None)
    ap.add_argument("--video", default=None)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--config", default="config/seg_config.yaml")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--save", default=None)
    ap.add_argument("--display", action="store_true", help="本地窗口（需显示器）")
    ap.add_argument("--ros", action="store_true",
                    help="ROS2 话题取流（ascamera HP60C；需先起驱动，ROS_DOMAIN_ID 一致）")
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
        pool = RKNNPool(backend_factory(cfg["pool"]["core_mask"]),
                        cfg["model"]["rknn"], 1, _pool_process)
        pool.put(img)
        outputs, ok = pool.get(timeout=30)
        if not ok or outputs is None:
            print("inference failed")
            return 2
        out, info = render(img, outputs, cfg)
        pool.release()
        print(f"[seg] {info['det']} instances")
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
    if args.save and args.ros:
        print("note: --save 在 --ros 模式不支持，忽略")

    pool = RKNNPool(backend_factory(cfg["pool"]["core_mask"]),
                    cfg["model"]["rknn"], cfg["pool"]["workers"], _pool_process)
    server = MjpegServer("0.0.0.0", args.port or cfg["server"]["port"],
                         stream_name=cfg["server"]["stream_name"]).start()
    frames, loop_t, init_t = 0, time.time(), time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames += 1
        pool.put(frame)
        outputs, p_ok = pool.get(timeout=10)
        if not p_ok or outputs is None:
            continue
        out, info = render(frame, outputs, cfg)
        server.set_frame(out)
        if args.display:
            cv2.imshow(cfg["display"]["window_name"], out)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if frames % 30 == 0:
            fps = 30 / (time.time() - loop_t)
            print(f"[seg] 30帧平均帧率: {fps:.1f} det={info['det']}")
            loop_t = time.time()
    print(f"[seg] 总平均帧率: {frames / (time.time() - init_t):.1f}")
    cap.release()
    pool.release()
    server.stop()
    if args.display:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
