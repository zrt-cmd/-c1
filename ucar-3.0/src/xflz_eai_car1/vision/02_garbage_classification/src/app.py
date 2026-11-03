"""垃圾分类推理 CLI（参照 rknn-python-Multithreading/main.py 骨架改造，构建文档 07）。

用法（板上 ros2_venv）：
    python src/app.py --image x.jpg --save out.jpg         # 单图检测
    python src/app.py --video test.mp4                     # 视频 + MJPEG :8080
    python src/app.py --camera 0 --display                 # 相机 + 本地窗口
    python src/app.py --ros                                # HP60C ROS2 话题取流（先起 ascamera 驱动）
MJPEG 流：http://<板 IP>:8080/garbage_cam.mjpg
"""
from __future__ import annotations

import argparse
import time

import cv2

from config import load_anchors, load_category_map, load_config, load_labels
from detector import Detector, DetectorConfig
from mjpeg_server import MjpegServer, require
from rknnpool import RKNNPool, rknn_backend_factory
from ros2_source import Ros2Camera


def make_process_fn(labels, cat_map, cat_names, det_cfg):
    """构造 pool 处理函数：后端上检测 + 画框/标注。"""
    def process_frame(backend, frame):
        dets = Detector(backend, labels, cat_map, cat_names, det_cfg).detect(frame)
        for d in dets:
            x1, y1, x2, y2 = int(d.x1), int(d.y1), int(d.x2), int(d.y2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            label = f"{d.category_name} {labels[d.class_id]} {d.score:.2f}"
            cv2.putText(frame, label, (x1, max(16, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return frame
    return process_frame


def main() -> int:
    ap = argparse.ArgumentParser(description="garbage classification demo")
    ap.add_argument("--model", default=None, help=".rknn 模型（默认取 config）")
    ap.add_argument("--labels", default=None, help="标签文件（默认取 config）")
    ap.add_argument("--video", default=None, help="视频文件")
    ap.add_argument("--image", default=None, help="单图检测")
    ap.add_argument("--save", default=None, help="单图输出路径")
    ap.add_argument("--camera", type=int, default=None, help="相机索引")
    ap.add_argument("--ros", action="store_true",
                    help="ROS2 话题取流（ascamera HP60C；需先起驱动，ROS_DOMAIN_ID 一致）")
    ap.add_argument("--tpes", type=int, default=None, help="线程数")
    ap.add_argument("--port", type=int, default=None, help="MJPEG 端口")
    ap.add_argument("--display", action="store_true", help="本地窗口显示")
    ap.add_argument("--config", default="config/garbage_config.yaml")
    args = ap.parse_args()
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    cfg = load_config(args.config)
    model_path = args.model or cfg["model"]["path"]
    labels = load_labels(args.labels or cfg["model"]["labels"])
    cat_map = load_category_map(cfg["model"]["category_map"], len(labels))
    cat_names = cfg["model"]["category_names"]
    inf, pool_cfg, srv = cfg["infer"], cfg["pool"], cfg["server"]
    tpes = args.tpes or pool_cfg["tpes"]

    det_cfg = DetectorConfig(
        imgsz=inf["imgsz"], score_thr=inf["score_thr"], nms_thr=inf["nms_thr"],
        sigmoid=inf["sigmoid"], anchors=load_anchors(inf["anchors"]))

    if args.image is not None:
        img = cv2.imread(args.image)
        if img is None:
            print(f"cannot read image: {args.image}")
            return 2
        backend = rknn_backend_factory(pool_cfg["core_mask"])(0)
        backend.load(model_path)
        out = make_process_fn(labels, cat_map, cat_names, det_cfg)(backend, img)
        backend.close()
        if args.save:
            cv2.imwrite(args.save, out)
            print(f"saved -> {args.save}")
        elif args.display:
            cv2.imshow("garbage", out)
            cv2.waitKey(0)
        else:
            print("use --save or --display for single image")
            return 2
        return 0

    server = MjpegServer(srv["host"], args.port or srv["port"],
                         srv["stream_name"], quality=srv["quality"]).start()

    src = args.camera if args.camera is not None else args.video
    if src is None and not args.ros and args.image is None:
        ap.print_help()
        server.stop()
        return 2
    if args.ros:
        ros_cfg = (cfg.get("source") or {}).get("ros") or {}
        cap = Ros2Camera(ros_cfg.get("rgb_topic", Ros2Camera.DEFAULT_TOPIC))
        cap.open()
        ok, _ = cap.read(timeout=float(ros_cfg.get("timeout", 5.0)))
        if not ok:
            print("cannot open ros topic (driver running? ROS_DOMAIN_ID match?)")
            cap.release()
            server.stop()
            return 2
    else:
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            print(f"cannot open source: {src}")
            server.stop()
            return 2

    pool = RKNNPool(rknn_backend_factory(pool_cfg["core_mask"]), model_path, tpes,
                    make_process_fn(labels, cat_map, cat_names, det_cfg))

    # 预填 TPEs+1 帧（与参考 main.py 一致）
    for _ in range(tpes + 1):
        ok, frame = cap.read()
        if not ok:
            cap.release()
            pool.release()
            server.stop()
            return 1
        pool.put(frame)

    frames, loop_t, init_t = 0, time.time(), time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames += 1
        pool.put(frame)
        result, flag = pool.get()
        if not flag or result is None:
            break
        server.set_frame(result)
        if args.display:
            cv2.imshow("garbage", result)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if frames % 30 == 0:
            print(f"30帧平均帧率: {30 / (time.time() - loop_t):.1f}")
            loop_t = time.time()

    print(f"总平均帧率: {frames / (time.time() - init_t):.1f}")
    cap.release()
    pool.release()
    server.stop()
    if args.display:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
