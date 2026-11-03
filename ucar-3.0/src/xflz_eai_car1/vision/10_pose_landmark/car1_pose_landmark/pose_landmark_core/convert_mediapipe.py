"""mediapipe pose_landmark_full tflite → RKNN 转换与契约自检（构建文档 06）。

转换在 x86 转换环境执行（rknn-toolkit2 2.3.x + tensorflow<=2.14），产物 .rknn
入库；板上只用 rknnlite 推理（本模块不会在板上运行）。CLI 对齐
mediapipe-rknn/models/pose_landmark/convert.py 惯例：--platform / --dtype fp|i8。

输入契约：float32 RGB [-1,1]（mean 127.5 / std 128 —— mediapipe-rknn 参考实现，
与 pose_landmark_core.pose.prepare() 的 rgb/127.5-1.0 一致）。

用法（x86 转换环境）：
  python -m pose_landmark_core.convert_mediapipe --all                     # fp16 混合量化（默认）
  python -m pose_landmark_core.convert_mediapipe --dtype i8 --dataset calib.txt
  python -m pose_landmark_core.convert_mediapipe --check                   # 契约自检已转模型
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

EXPECTED: dict = {
    "pose_landmark_full": {
        "url": "https://storage.googleapis.com/mediapipe-assets/pose_landmark_full.tflite",
        "input": [1, 256, 256, 3],
        "input_name": "input_1",   # 转换后以 --check 实测为准（若失败提示核实）
        "min_outputs": 5,          # 33点(1,195) + presence(1,1) + seg(1,256,256,1) + heatmap(1,64,64,39) + world(1,117)
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
    """tflite → rknn。输入契约：float32 RGB [-1,1]（mean 127.5 / std 128）。

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
            mean_values=[[127.5, 127.5, 127.5]], std_values=[[128, 128, 128]],
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
