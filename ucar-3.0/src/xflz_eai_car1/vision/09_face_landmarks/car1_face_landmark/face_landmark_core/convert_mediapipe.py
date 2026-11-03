"""mediapipe tflite → RKNN 转换与契约自检（构建文档 06）。

转换在 x86 转换环境执行（rknn-toolkit2 2.3.x + tensorflow<=2.14），产物 .rknn
入库；板上只用 rknnlite 推理（本模块不会在板上运行）。CLI 对齐
mediapipe-rknn/ 转换惯例：--platform / --dtype fp|i8（i8 需 --dataset）。

用法（x86 转换环境）：
  python -m face_landmark_core.convert_mediapipe --all                     # fp16 混合量化（默认）
  python -m face_landmark_core.convert_mediapipe --dtype i8 --dataset calib.txt
  python -m face_landmark_core.convert_mediapipe --check                   # 契约自检已转模型
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

EXPECTED: dict = {
    "face_detection_short_range": {
        "url": "https://storage.googleapis.com/mediapipe-assets/face_detection_short_range.tflite",
        "input": [1, 128, 128, 3],
        "input_name": "input",
        "output_names": ["regressors", "classificators"],
    },
    "face_landmark": {
        "url": "https://storage.googleapis.com/mediapipe-assets/face_landmark.tflite",
        "input": [1, 192, 192, 3],
        "input_name": "input_1",
        "output_names": ["conv2d_21", "conv2d_31"],
    },
}

MODELS_DIR = Path(__file__).resolve().parents[2] / "models"

# 与 mediapipe-rknn/models/<name>/convert.py --platform choices 对齐
PLATFORMS = ("rk3562", "rk3566", "rk3568", "rk3576", "rk3588")


def exp_input(name: str) -> list:
    return EXPECTED[name]["input"]


def parse_rknn_header(path: str | Path) -> dict:
    """从 .rknn 二进制提取头部 JSON 契约。

    fp16/非量化模型头：顶层对象键 = 输入/输出名，值 = {dtype, layout}。
    返回合并 dict（"input": {...}, 输出名: {...}）。
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
    """tflite → rknn。输入契约：float32 RGB [0,1]（调用方归一化）→ mean/std 恒等。

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
    """契约自检：rknn 头 JSON 与 EXPECTED 比对，返回契约 dict。"""
    workdir = Path(workdir or MODELS_DIR)
    rknn_path = workdir / f"{name}.rknn"
    if not rknn_path.exists():
        raise FileNotFoundError(f"model not converted yet: {rknn_path}")
    header = parse_rknn_header(rknn_path)
    exp = EXPECTED[name]
    missing = [n for n in exp["output_names"] if n not in header]
    if missing:
        raise AssertionError(f"{name}: missing outputs {missing} in {sorted(header)}")
    if exp["input_name"] not in header:
        raise AssertionError(f"{name}: missing input {exp['input_name']} in {sorted(header)}")
    return header


def main():
    parser = argparse.ArgumentParser(description="mediapipe tflite -> rknn (exp09, x86)")
    parser.add_argument("--all", action="store_true", help="convert all models")
    parser.add_argument("--name", default="", help="convert/check single model")
    parser.add_argument("--check", action="store_true", help="contract check only")
    parser.add_argument("--platform", default="rk3576", choices=PLATFORMS)
    parser.add_argument("--dtype", default="fp", choices=("fp", "i8"),
                        help="fp = fp16 混合量化（默认）; i8 = int8（需 --dataset）")
    parser.add_argument("--dataset", default=None, help="quantization dataset (txt)")
    args = parser.parse_args()

    names = [args.name] if args.name else list(EXPECTED)
    for name in names:
        if name not in EXPECTED:
            print(f"[convert] unknown model: {name}")
            return 2
        try:
            if args.check:
                h = check(name)
                print(f"[convert] {name} OK: in={exp_input(name)} out={sorted(EXPECTED[name]['output_names'])}")
            else:
                p = convert(name, platform=args.platform,
                            do_quantization=args.dtype == "i8", dataset=args.dataset)
                h = check(name)
                print(f"[convert] {name} -> {p} | "
                      f"in={exp_input(name)} dtype={h.get('input', {}).get('dtype')}")
        except (FileNotFoundError, RuntimeError, AssertionError, ValueError) as exc:
            print(f"[convert] {name} FAILED: {exc}")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
