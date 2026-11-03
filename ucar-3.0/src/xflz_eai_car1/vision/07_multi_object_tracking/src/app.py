"""多目标跟踪 CLI（构建文档 06）。

用法（板上 ros2_venv）：
    python src/app.py --ros                 # ascamera HP60C 话题取流（需先起驱动，ROS_DOMAIN_ID 一致）
    python src/app.py --camera 0            # 相机 + 检测池 + ReID 池 + DeepSORT + MJPEG :8080/track.mjpg
    python src/app.py --video test.mp4
    python src/app.py --image x.jpg --save out.jpg

多线程：检测走 RKNNPool（N worker，round_robin 绑核）；ReID 特征提取走
ReIDPool（2 worker × 独立实例），主线程只做解码/跟踪/绘制。流水线双缓冲：
帧 N 处理期间帧 N+1 已在池内推理。
ReID 模型缺失/加载失败时自动回退 SORT（纯 IoU）并告警。
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import cv2

from config import load_config
from detect import Detection, decode_outputs_prefilter
from mjpeg_server import MjpegServer, require
from preprocess import letterbox
from reid import ReIDBackend
from reidpool import ReIDPool
from rknnpool import RKNNPool, backend_factory
from tracker import Tracker
from ros2_source import Ros2Camera


def _det_process(backend, frame):
    return backend.predict(frame)


def process_frame(frame, outputs, cfg, tracker, reid, names=None):
    """单帧：检测输出 -> 类别过滤 -> 回映射原图坐标 -> 特征（ReID 池）-> 跟踪 -> 标注。"""
    imgsz = cfg["model"]["imgsz"]
    lb = letterbox(frame, imgsz)
    raw = decode_outputs_prefilter(outputs, cfg["detect"]["score_thr"],
                                    cfg["detect"]["iou_thr"], imgsz=imgsz)
    keep = cfg["detect"].get("classes")
    if keep:
        raw = [d for d in raw if d.class_id in keep]  # 类别过滤（参考 exp06 classes）
    dets = []
    for d in raw:
        b = lb.unletterbox(np.array([[d.x1, d.y1, d.x2, d.y2]], dtype=np.float32))[0]
        dets.append(Detection(float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                              d.score, d.class_id))
    features = None
    if reid is not None and cfg["tracker"]["mode"] == "deepsort" and dets:
        gate_iou = float(cfg["tracker"].get("reid_gate_iou", 0.65))
        needed, reuse_pairs = _reid_gate(dets, tracker, iou_thr=gate_iou)
        # 仅对「新/未匹配」det 提特征（稳定目标跳过推理，省 ReID）
        crops, idx = [], []
        for i in needed:
            d = dets[i]
            x1 = max(0, int(d.x1)); y1 = max(0, int(d.y1))
            x2 = min(frame.shape[1], int(d.x2)); y2 = min(frame.shape[0], int(d.y2))
            if x2 - x1 >= 4 and y2 - y1 >= 4:
                crops.append(frame[y1:y2, x1:x2]); idx.append(i)
        crops = [c for c in crops if c.size > 0]
        feat = reid.extract(crops) if crops else np.zeros(
            (0, getattr(reid, "dim", None) or 512), dtype=np.float32)
        # 占位：稳定目标续用其 track 特征均值（防分裂），新目标填真实特征
        features = np.zeros((len(dets), feat.shape[1]), dtype=np.float32)
        for row, i in enumerate(idx):
            features[i] = feat[row]
        for di, ti in reuse_pairs:
            t = tracker.tracks[ti]
            if t.features:
                features[di] = np.mean(np.asarray(t.features), axis=0)
    tracker.predict()
    tracker.update(dets, features)
    out = draw_tracks(frame, tracker, names)
    return out, {"tracks": len([t for t in tracker.tracks if t.confirmed])}




def _reid_gate(dets, tracker, iou_thr: float = 0.65):
    """ReID 按需门控：返回 (需提特征 det 索引, [(det_idx, confirmed_track_idx)...])。

    对每个 det 与所有 confirmed track 算 IoU：高 IoU 且同 class、track 已有特征 →
    续用该 track 特征均值（防分裂/长遮挡丢 ID）；否则需提新特征。"""
    from matching import iou_batch
    confirmed = [i for i, t in enumerate(tracker.tracks) if t.confirmed]
    if not confirmed or not dets:
        return list(range(len(dets))), []
    trk_boxes = []
    for i in confirmed:
        t = tracker.tracks[i]
        cx, cy, a, h = t.to_xyah()
        w = a * h
        trk_boxes.append([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])
    trk_boxes = np.array(trk_boxes, dtype=np.float32)
    det_boxes = np.array([[d.x1, d.y1, d.x2, d.y2] for d in dets], dtype=np.float32)
    ious = iou_batch(trk_boxes, det_boxes).T          # (N, M)
    needed, reuse = [], []
    for i in range(len(dets)):
        best = ious[i].max()
        bi = int(ious[i].argmax())
        t = tracker.tracks[confirmed[bi]]
        if best >= iou_thr and t.features and t.class_id == dets[i].class_id:
            reuse.append((i, confirmed[bi]))
        else:
            needed.append(i)
    return needed, reuse


def draw_tracks(img, tracker, names=None):
    """绘制确认轨迹框 + ID + 类别名（黄色）。"""
    out = img.copy()
    names = names or {}
    for t in tracker.tracks:
        if not t.confirmed:
            continue
        cx, cy, a, h = t.to_xyah()
        w = a * h
        x1, y1 = int(cx - w / 2), int(cy - h / 2)
        x2, y2 = int(cx + w / 2), int(cy + h / 2)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)
        label = f"ID {t.track_id}"
        if t.class_id >= 0 and names:
            label += f" {names.get(str(t.class_id), t.class_id)}"
        cv2.putText(out, label, (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="deepsort multi-object tracking")
    ap.add_argument("--image", default=None)
    ap.add_argument("--video", default=None)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--config", default="config/track_config.yaml")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--save", default=None)
    ap.add_argument("--display", action="store_true")
    ap.add_argument("--ros", action="store_true",
                    help="ROS2 话题取流（ascamera HP60C；需先起驱动，ROS_DOMAIN_ID 一致）")
    args = ap.parse_args()
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    cfg = load_config(args.config)
    src = args.camera if args.camera is not None else (args.video if args.video else args.image)
    if src is None and not args.ros:
        ap.print_help()
        return 2

    names: dict = {}
    labels_path = Path(cfg["model"]["det_rknn"]).parent / "coco_classes.json"
    if labels_path.exists():
        import json
        names = json.loads(labels_path.read_text(encoding="utf-8"))

    reid = None
    if cfg["tracker"]["mode"] == "deepsort" and cfg["model"]["reid_rknn"]:
        probe = ReIDBackend()
        try:
            probe.load(cfg["model"]["reid_rknn"])  # 探针：模型可用性
            probe.close()
            reid = ReIDPool(cfg["model"]["reid_rknn"], workers=2)
        except Exception as exc:
            print(f"[warn] ReID load failed ({exc}); falling back to SORT")
            probe.close()
            reid = None
    mode = "deepsort" if reid is not None else "sort"
    tracker = Tracker(max_iou_distance=cfg["tracker"]["max_iou_distance"],
                      max_age=cfg["tracker"]["max_age"],
                      n_init=cfg["tracker"]["n_init"],
                      nn_budget=cfg["tracker"]["nn_budget"],
                      max_dist=cfg["tracker"]["max_dist"], mode=mode)
    print(f"[track] mode={mode}")

    if args.image is not None:
        img = cv2.imread(args.image)
        if img is None:
            print(f"cannot read image: {args.image}")
            return 2
        pool = RKNNPool(backend_factory(cfg["pool"]["core_mask"]),
                        cfg["model"]["det_rknn"], 1, _det_process)
        pool.put(img)
        outputs, ok = pool.get(timeout=30)
        pool.release()
        if not ok or outputs is None:
            print("inference failed")
            return 2
        out, info = process_frame(img, outputs, cfg, tracker, reid, names)
        print(f"[track] tracks={info['tracks']}")
        if args.save:
            cv2.imwrite(args.save, out)
            print(f"saved -> {args.save}")
        elif args.display:
            cv2.imshow(cfg["display"]["window_name"], out)
            cv2.waitKey(0)
        else:
            print("use --save or --display for single image")
            return 2
        if reid is not None:
            reid.close()
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

    pool = RKNNPool(backend_factory(cfg["pool"]["core_mask"]),
                    cfg["model"]["det_rknn"], cfg["pool"]["workers"], _det_process)
    server = MjpegServer("0.0.0.0", args.port or cfg["server"]["port"],
                         stream_name=cfg["server"]["stream_name"]).start()

    if args.save and args.ros:
        print("note: --save 在 --ros 模式不支持，忽略")
    # 流水线双缓冲：先入队首帧，处理帧 N 时帧 N+1 已在池内推理
    frames, loop_t, init_t = 0, time.time(), time.time()
    ok, frame = cap.read()
    if ok:
        pool.put(frame)
    while ok:
        outputs, p_ok = pool.get(timeout=10)
        ok, nxt = cap.read()
        if ok:
            pool.put(nxt)
        if p_ok and outputs is not None:
            out, info = process_frame(frame, outputs, cfg, tracker, reid, names)
            server.set_frame(out)
            frames += 1
            if args.display:
                cv2.imshow(cfg["display"]["window_name"], out)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if frames % 30 == 0:
                fps = 30 / (time.time() - loop_t)
                print(f"[track] 30帧平均帧率: {fps:.1f} tracks={info['tracks']}")
                loop_t = time.time()
        if not ok:
            break
        frame = nxt
    print(f"[track] 总平均帧率: {frames / (time.time() - init_t):.1f}")
    cap.release()
    pool.release()
    server.stop()
    if reid is not None:
        reid.close()
    if args.display:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
