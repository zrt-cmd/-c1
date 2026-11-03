"""ONNX -> RKNN 转换（板上 rknn-toolkit2 2.3.2，target rk3576）。

用法（板载 ros2_venv）：
    python src/convert_rknn.py \
        --onnx train/weights/best.onnx \
        --output models/defect_yolov8n_int8.rknn \
        --calib-dir <NEU-DET>/valid/images --calib-count 200 \
        --target rk3576

契约（TDD 断言目标）：
- build_calib_list(calib_dir, count) -> list[Path]：按字典序取前 count 张图
- convert(onnx, output, calib_list, target, quantize)：返回 0 表示成功
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import cv2

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def build_calib_list(calib_dir: str | Path, count: int) -> list[Path]:
    """取前 count 张图片（按文件名排序），返回绝对路径列表。"""
    d = Path(calib_dir)
    imgs = sorted(p for p in d.iterdir() if p.suffix.lower() in IMG_EXTS) if d.exists() else []
    return imgs[:count]


def _preprocess_calib(img_path: Path, imgsz: int) -> np.ndarray:
    """校准输入：letterbox(0 填充) + BGR->RGB，HWC uint8。"""
    from preprocess import letterbox

    img = cv2.imread(str(img_path))
    if img is None:
        raise FileNotFoundError(f"cannot read image: {img_path}")
    lb = letterbox(img, imgsz, color=(0, 0, 0))
    return lb.padded[..., ::-1].copy()  # RGB


def convert(
    onnx_path: str | Path,
    output_path: str | Path,
    calib_list: list[Path],
    target: str = "rk3576",
    quantize: bool = True,
    imgsz: int = 640,
    verbose: bool = False,
) -> int:
    """执行转换；成功返回 0。量化时使用校准图做 int8 校准。"""
    from rknn.api import RKNN  # 延迟导入：纯函数测试无需 rknn

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    rknn = RKNN(verbose=verbose)
    try:
        # 显式配置（默认值逐项核对 2.3.0 签名；勿用 toolkit1 的
        # quantized_dtype='asymmetric_quantized-8'——2.x 会经 **kwargs 静默忽略）
        if rknn.config(
            mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
            target_platform=target,
            quant_img_RGB2BGR=False,        # 输入已为 RGB（导出时固化），不再转色域
            quantized_dtype="w8a8",         # toolkit2 默认 int8 量化
            quantized_algorithm="normal",   # 默认
            quantized_method="channel",     # 2.3.0 默认即 channel（逐通道量化）
            optimization_level=3,           # 默认
            single_core_mode=True,          # 单核模型：省内存（用户指定 2026-08-05）
            model_pruning=True,             # 无损剪枝：省内存（用户指定 2026-08-05）
        ) != 0:
            raise RuntimeError("rknn.config failed")
        if rknn.load_onnx(model=str(onnx_path)) != 0:
            raise RuntimeError(f"load_onnx failed: {onnx_path}")

        dataset = ""
        if quantize:
            if not calib_list:
                raise ValueError("quantize=True 需要 --calib-dir/--calib-count")
            calib_txt = out.parent / f"{out.stem}_calib.txt"
            calib_txt.write_text("\n".join(str(p) for p in calib_list), encoding="utf-8")
            dataset = str(calib_txt)

        if rknn.build(do_quantization=quantize, dataset=dataset) != 0:
            raise RuntimeError("rknn.build failed")
        if rknn.export_rknn(str(out)) != 0:
            raise RuntimeError(f"export_rknn failed: {out}")
        return 0
    finally:
        rknn.release()


def main() -> int:
    ap = argparse.ArgumentParser(description="onnx -> rknn (rk3576)")
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--calib-dir", default=None, help="量化校准图目录（val/images）")
    ap.add_argument("--calib-count", type=int, default=200)
    ap.add_argument("--target", default="rk3576")
    ap.add_argument("--no-quantize", action="store_true", help="fp16/非量化转换")
    ap.add_argument("--imgsz", type=int, default=640)
    args = ap.parse_args()

    calib = build_calib_list(args.calib_dir, args.calib_count) if args.calib_dir else []
    return convert(
        args.onnx, args.output, calib,
        target=args.target, quantize=not args.no_quantize, imgsz=args.imgsz,
    )


if __name__ == "__main__":
    raise SystemExit(main())
