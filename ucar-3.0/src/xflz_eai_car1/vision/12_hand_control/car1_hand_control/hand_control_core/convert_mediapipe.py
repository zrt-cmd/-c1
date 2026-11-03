"""mediapipe pose_landmark_full tflite → RKNN 转换与契约自检（构建文档 06）。

转换在 x86 转换环境执行（rknn-toolkit2 2.3.x + tensorflow<=2.14），产物 .rknn
入库；板上只用 rknnlite 推理（本模块不会在板上运行）。CLI 对齐
mediapipe-rknn/models/pose_landmark/convert.py 惯例：--platform / --dtype fp|i8。

输入契约：float32 RGB [0,1]（mean 0 / std 1 —— 与用户 mediapipe-rknn
convert.py MODELS 表一致；图像模型 uint8 直喂会爆炸，必须 float/255）。

用法（x86 转换环境）：
  python -m hand_control_core.convert_mediapipe --all                     # fp16 混合量化（默认）
  python -m hand_control_core.convert_mediapipe --dtype i8 --dataset calib.txt
  python -m hand_control_core.convert_mediapipe --check                   # 契约自检已转模型
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

EXPECTED: dict = {
    # CSDN 方案：gesture_recognizer.task 包提取转换（mediapipe-rknn convert.py 语义）
    # 输入契约（板端实测 2026-08-07）：图像模型 float32 RGB [0,1]（mean 0 / std 1，
    # uint8 直接喂 → 输出爆炸 cls -1044）——convert.py MODELS 表 mean [[0,0,0]]/std [[1,1,1]]
    "hand_detector": {
        "url": "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task",
        "input": [1, 192, 192, 3],
        "input_name": "input_1",    # 头部 JSON 实测
        "min_outputs": 2,          # 手掌检测 regressors + classificators
        "mean_std": "0/1",
    },
    "hand_landmarks_detector": {
        "url": "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task",
        "input": [1, 224, 224, 3],
        "input_name": "input_1",   # 头部 JSON 实测
        "min_outputs": 4,          # 21点(1,63) + presence(1,1) + handedness(1,1) + world(1,63)
        "mean_std": "0/1",
    },
    "gesture_embedder": {
        "url": None,               # 同 .task 包内提取
        "input": [1, 21, 3],
        "input_name": "hand",      # 头部 JSON 实测：仅 1 输入 norm_tensor（3 输入参考语义作废）
        "min_outputs": 1,          # (1,128) embedding；float 无 mean/std
        "mean_std": "none",
    },
    "canned_gesture_classifier": {
        "url": None,
        "input": [1, 128],
        "input_name": "hand_embedding",  # 头部 JSON 实测
        "min_outputs": 1,          # (1,8) logits；float 无 mean/std
        "mean_std": "none",
    },
}

MODELS_DIR = Path(__file__).resolve().parents[2] / "models"

# 与 mediapipe-rknn/models/<name>/convert.py --platform choices 对齐
PLATFORMS = ("rk3562", "rk3566", "rk3568", "rk3576", "rk3588")


def parse_rknn_header(path: str | Path) -> dict:
    """从 .rknn 二进制提取头部 JSON 契约（fp16/非量化模型头）。

    顶层对象键 = 输入/输出名，值 = {dtype, layout}；返回合并 dict。
    """
    data = Path(path).read_bytes()
    dec = json.JSONDecoder()
    merged: dict = {}
    for i, b in enumerate(data[:12000]):
        if b == ord("{"):
            try:
                obj, _ = dec.raw_decode(data[i:].decode("utf-8", "replace"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            if isinstance(obj, dict) and all(
                    isinstance(v, dict) and "dtype" in v for v in obj.values()):
                merged.update(obj)
    if not merged:
        raise ValueError(f"no JSON contract header found in {path}")
    return merged


def download_tflite(name: str, workdir: str | Path | None = None) -> Path:
    """下载 tflite（已存在则跳过）。"""
    workdir = Path(workdir or MODELS_DIR)
    tflite = workdir / f"{name}.tflite"
    if tflite.exists():
        return tflite
    import urllib.request

    url = EXPECTED[name]["url"]
    workdir.mkdir(parents=True, exist_ok=True)
    urllib.request.urlretrieve(url, tflite)
    return tflite


def convert(name: str, platform: str = "rk3576",
            workdir: str | Path | None = None,
            do_quantization: bool = False,
            dataset: str | None = None) -> Path:
    """tflite → rknn。输入契约：float32 RGB [0,1]（mean 0 / std 1，与用户
    mediapipe-rknn convert.py MODELS 表一致；127.5/128 会打出 [-1,1] → reg 爆炸）。

    默认不量化（fp16 混合，landmark 回归精度稳）；int8 需 do_quantization + dataset。
    """
    workdir = Path(workdir or MODELS_DIR)
    tflite = download_tflite(name, workdir)
    out = workdir / f"{name}.rknn"
    if out.exists():
        return out

    if do_quantization and not dataset:
        print("[convert] do_quantization=True requires --dataset; falling back to fp16")
        do_quantization = False

    from rknn.api import RKNN  # 转换环境依赖（x86，用户安装）

    rknn = RKNN(verbose=False)
    try:
        rknn.config(
            mean_values=[[0, 0, 0]], std_values=[[1, 1, 1]],
            target_platform=platform,
            quantized_dtype="w8a8",
        )
        if rknn.load_tflite(model=str(tflite)) != 0:
            raise RuntimeError(f"load_tflite failed: {tflite}")
        if rknn.build(do_quantization=do_quantization, dataset=dataset) != 0:
            raise RuntimeError(f"build failed: {name}")
        if rknn.export_rknn(str(out)) != 0:
            raise RuntimeError(f"export failed: {out}")
    finally:
        rknn.release()
    return out


def check(name: str, workdir: str | Path | None = None) -> dict:
    """契约自检：rknn 头 JSON 与 EXPECTED 比对（输入名 + 输出数），返回契约 dict。"""
    workdir = Path(workdir or MODELS_DIR)
    rknn_path = workdir / f"{name}.rknn"
    if not rknn_path.exists():
        raise FileNotFoundError(f"model not converted yet: {rknn_path}")
    header = parse_rknn_header(rknn_path)
    exp = EXPECTED[name]
    if exp["input_name"] not in header:
        raise AssertionError(
            f"{name}: input {exp['input_name']!r} not in header {sorted(header)} "
            f"(if name differs, update EXPECTED input_name)")
    n_outputs = len(header) - 1  # 除输入名外均为输出
    if n_outputs < exp["min_outputs"]:
        raise AssertionError(f"{name}: expected >= {exp['min_outputs']} outputs, got {n_outputs}")
    return header


def main():
    parser = argparse.ArgumentParser(description="mediapipe tflite -> rknn (exp10, x86)")
    parser.add_argument("--all", action="store_true", help="转换 EXPECTED 全部模型")
    parser.add_argument("--check", action="store_true", help="契约自检已转模型")
    parser.add_argument("--platform", default="rk3576", choices=PLATFORMS)
    parser.add_argument("--dtype", default="fp", choices=["fp", "i8"])
    parser.add_argument("--dataset", default=None)
    parser.add_argument("--workdir", default=None)
    args = parser.parse_args()

    names = list(EXPECTED)
    if args.check:
        for name in names:
            try:
                header = check(name, args.workdir)
            except FileNotFoundError as exc:
                print(f"[check] {exc}")
                return 2
            print(f"[check] {name}: OK ({len(header)} tensors)")
        return 0
    if args.all:
        for name in names:
            out = convert(name, args.platform, args.workdir,
                          do_quantization=args.dtype == "i8", dataset=args.dataset)
            print(f"[convert] {out} done")
        return 0
    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
