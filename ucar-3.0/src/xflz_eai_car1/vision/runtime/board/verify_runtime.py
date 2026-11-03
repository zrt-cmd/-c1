#!/usr/bin/env python3
"""Board runtime self-check for RK3576 (10.1.134.84).

Run on the board:
    source ~/ros2_venv/bin/activate
    python3 course_examples/runtime/board/verify_runtime.py

Each check is a (name, ok: bool|None, detail) triple; None = WARN (optional,
not blocking). Exit code 0 = all hard checks passed.
"""
import importlib.metadata as md
import os
import sys
import glob
from pathlib import Path

EXPECTED = {
    "numpy": "1.26.4",
    "opencv-python": "4.11.0.86",
    "onnx": "1.16.1",
    "onnxruntime": "1.23.2",
    "scipy": "1.15.3",
    "pillow": "12.2.0",
    "pyserial": "3.5",
    "pyyaml": "6.0.3",
    "rknn-toolkit2": "2.3.2",
    "rknn-toolkit-lite2": "2.3.2",
    "mjpeg-streamer": "2024.2.8",
    "sherpa-onnx": "1.13.4",
    "flask": "3.1.3",
    "flask-socketio": "5.6.1",
    "pupil-apriltags": "1.0.4.post11",
}

RESULTS = []


def check(name, ok, detail):
    RESULTS.append((name, ok, detail))
    tag = "PASS" if ok else ("WARN" if ok is None else "FAIL")
    print(f"[{tag}] {name}: {detail}")


def version_of(pkg):
    try:
        return md.version(pkg)
    except md.PackageNotFoundError:
        return None


def main():
    # --- Python runtime -------------------------------------------------
    py = sys.version_info
    check("python>=3.10", py >= (3, 10), f"{py.major}.{py.minor}.{py.micro}")
    check("venv active (ros2_venv)", "ros2_venv" in sys.prefix, sys.prefix)

    # --- Key packages ---------------------------------------------------
    for pkg, want in EXPECTED.items():
        got = version_of(pkg)
        check(f"pkg {pkg}=={want}", got == want, f"installed={got}")

    # rknn API importable (full toolkit, aarch64)
    try:
        from rknn.api import RKNN  # noqa: F401
        check("rknn.api.RKNN import", True, "ok")
    except Exception as exc:  # pragma: no cover
        check("rknn.api.RKNN import", False, str(exc))

    # --- Hardware (RK3576 board) ---------------------------------------
    model = Path("/proc/device-tree/model").read_text(errors="ignore").rstrip("\x00")
    check("SoC is RK3576", "rk3576" in model.lower(), model or "unknown")
    # RK3576 NPU devfreq 节点为 27700000.npu（fdab0000.npu 是 RK356x 的）
    npu_freq_paths = sorted(glob.glob("/sys/class/devfreq/*npu*/cur_freq"))
    npu_freq = Path(npu_freq_paths[0]) if npu_freq_paths else None
    check("NPU freq readable",
          npu_freq is not None,
          f"{npu_freq}: {npu_freq.read_text().strip()} MHz" if npu_freq else "missing")
    npu_load = Path("/sys/kernel/debug/rknpu/load")
    try:
        load_ok = bool(npu_load.exists())
    except PermissionError:
        load_ok = None  # debugfs 需 root：非 root 时 WARN（不阻塞）
    check("NPU load debugfs", load_ok,
          "ok" if load_ok else "needs root; run: sudo cat /sys/kernel/debug/rknpu/load")
    check("ROS_DOMAIN_ID set",
          bool(os.environ.get("ROS_DOMAIN_ID"))
          or "ROS_DOMAIN_ID=2" in Path("/home/iflytek/.bashrc").read_text(errors="ignore"),
          os.environ.get("ROS_DOMAIN_ID", "(unset; bashrc has ROS_DOMAIN_ID=2)"))
    cams = glob.glob("/dev/video*")
    check("camera devices >=1", len(cams) >= 1, f"{len(cams)}: {cams[:4]}{'...' if len(cams) > 4 else ''}")
    ttys = glob.glob("/dev/ttyUSB*")
    check("usb-serial devices >=1", len(ttys) >= 1, f"{len(ttys)}: {ttys}")

    # --- ROS2 -----------------------------------------------------------
    check("ROS2 Humble", Path("/opt/ros/humble/setup.bash").exists(), "/opt/ros/humble")
    check("car workspace built", Path("/home/iflytek/ros2_ws/install/setup.bash").exists(), "~/ros2_ws/install")

    # --- Optional (WARN only) -------------------------------------------
    try:
        import cv2
        check("opencv import", True, cv2.__version__)
    except Exception as exc:  # pragma: no cover
        check("opencv import", False, str(exc))
    try:
        import torch
        check("torch import", True, torch.__version__)
    except Exception as exc:  # pragma: no cover
        check("torch import", False, str(exc))
    check("ultralytics absent (by design)", version_of("ultralytics") is None,
          "training runs on PC; board does rknn inference only")

    # --- Summary --------------------------------------------------------
    hard = [r for r in RESULTS if r[1] is not None]
    failed = [r for r in hard if not r[1]]
    print(f"\n{len(hard) - len(failed)}/{len(hard)} hard checks passed"
          f" ({len([r for r in RESULTS if r[1] is None])} warn)")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
