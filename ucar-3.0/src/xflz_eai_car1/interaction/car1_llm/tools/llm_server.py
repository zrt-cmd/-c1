#!/usr/bin/env python3
"""
RKLLM Server launcher — starts the OpenAI-compatible chat API server.

Wraps the official rkllm_server_demo Flask server with automatic setup:
  - lib/librkllmrt.so symlink
  - Model path (Qwen3.5-0.8B W4A16 RKLLM) + Platform (RK3576)
  - Optional NPU/CPU/GPU/DDR frequency locking via --fix-freq

API: OpenAI-compatible /v1/chat/completions on http://0.0.0.0:8080

Usage:
  python3 llm_server.py                    # start server (no frequency tuning)
  python3 llm_server.py --fix-freq         # start server + lock clocks to max

The --fix-freq flag runs a script that disables CPU idle and locks
NPU / CPU / GPU / DDR to their maximum frequencies for best inference
performance. This step requires root (writes to /sys/) — you will be
prompted for sudo password. The server itself runs as the current user.
"""

import argparse
import os
import subprocess
import sys
import tempfile


# ---------------------------------------------------------------------------
# fix_freq: lock NPU/CPU/GPU/DDR to max frequency (requires root)
# ---------------------------------------------------------------------------

def run_fix_freq(platform, tools_dir, server_dir):
    """Run the bundled fix_freq script with sudo."""

    fix_freq_script = os.path.join(server_dir, f"fix_freq_{platform}.sh")

    if not os.path.exists(fix_freq_script):
        print(f"Warning: fix_freq script not found at {fix_freq_script}, skipping.")
        return False

    print("Locking clocks to max frequency (requires sudo)...")
    result = subprocess.run(["sudo", "bash", fix_freq_script])
    if result.returncode != 0:
        print("Warning: fix_freq failed (exit code: %d)." % result.returncode)
        print("  Continuing without frequency tuning — performance may be lower.")
        return False
    print("  Clocks locked.\n")
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="RKLLM Server launcher (OpenAI-compatible chat API)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 llm_server.py                     # start server
  python3 llm_server.py --fix-freq          # start + lock clocks to max (sudo prompt)
  python3 llm_server.py --model /path/to/model.rkllm
        """,
    )

    parser.add_argument("--model",
                        default="/home/iflytek/models/rkllm/Qwen3.5-0.8B_w4a16_g128_rk3576.rkllm",
                        help="Path to RKLLM model file")
    parser.add_argument("--platform", default="rk3576",
                        choices=["rk3588", "rk3576", "rv1126b", "rk3562"])
    parser.add_argument("--fix-freq", action="store_true",
                        help="Lock NPU/CPU/GPU/DDR to max frequency before starting "
                             "(will prompt for sudo password)")
    parser.add_argument("--lora-model", default=None)
    parser.add_argument("--prompt-cache", default=None)

    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f"Error: model file not found: {args.model}")
        sys.exit(1)

    # --- Locate the bundled llm_server directory (relative to this script) ---------
    tools_dir = os.path.dirname(os.path.abspath(__file__))
    server_dir = os.path.join(tools_dir, "..", "llm_server")
    server_dir = os.path.abspath(server_dir)
    server_script = os.path.join(server_dir, "flask_server.py")

    if not os.path.exists(server_script):
        print(f"Error: flask_server.py not found at {server_script}")
        sys.exit(1)

    # --- Setup lib/librkllmrt.so --------------------------------------------------
    lib_dir = os.path.join(server_dir, "lib")
    os.makedirs(lib_dir, exist_ok=True)

    lib_src = "/usr/lib/librkllmrt.so"
    lib_dst = os.path.join(lib_dir, "librkllmrt.so")
    if not os.path.exists(lib_dst):
        if os.path.exists(lib_src):
            os.symlink(lib_src, lib_dst)
            print(f"  Linked: {lib_dst} -> {lib_src}")
        else:
            print(f"Error: {lib_src} not found. Install the RKLLM runtime first.")
            sys.exit(1)

    # --- fix_freq: run with real sudo if requested, else provide no-op ------------
    fix_freq_script = os.path.join(server_dir, f"fix_freq_{args.platform}.sh")

    if args.fix_freq:
        run_fix_freq(args.platform, tools_dir, server_dir)
    else:
        if not os.path.exists(fix_freq_script):
            with open(fix_freq_script, "w") as f:
                f.write("#!/bin/bash\n# no-op — run with --fix-freq to lock clocks\n")
            os.chmod(fix_freq_script, 0o755)

    # --- Intercept the hardcoded "sudo bash fix_freq_*.sh" in flask_server.py -----
    fake_sudo_dir = tempfile.mkdtemp(prefix="llm_server_")
    fake_sudo = os.path.join(fake_sudo_dir, "sudo")
    with open(fake_sudo, "w") as f:
        f.write("#!/bin/bash\n")
        f.write("# Fake sudo — flaks_server.py hardcodes sudo; we provide a no-op.\n")
        f.write("exit 0\n")
    os.chmod(fake_sudo, 0o755)

    env = os.environ.copy()
    env["PATH"] = f"{fake_sudo_dir}:{env.get('PATH', '')}"

    # --- Launch the server (always use the venv Python, not sudo) ------------------
    venv_python = "/home/iflytek/ros2_venv/bin/python3"
    if not os.path.exists(venv_python):
        venv_python = sys.executable  # fallback

    cmd = [
        venv_python,
        server_script,
        "--rkllm_model_path", args.model,
        "--target_platform", args.platform,
    ]
    if args.lora_model:
        cmd.extend(["--lora_model_path", args.lora_model])
    if args.prompt_cache:
        cmd.extend(["--prompt_cache_path", args.prompt_cache])

    print("=" * 60)
    print("RKLLM Server")
    print("=" * 60)
    print(f"  Model:    {os.path.basename(args.model)}")
    print(f"  Platform: {args.platform}")
    print(f"  Fix freq: {'YES' if args.fix_freq else 'no (use --fix-freq to lock clocks)'}")
    print(f"  API:      http://0.0.0.0:8080/v1/chat/completions")
    print("=" * 60)
    print()
    print("Starting server... (Ctrl+C to stop)")
    print("Connect with: python3 llm_test.py")
    print()
    sys.stdout.flush()

    try:
        subprocess.run(cmd, cwd=server_dir, env=env)
    except KeyboardInterrupt:
        print("\nServer stopped.")
    finally:
        try:
            os.remove(fake_sudo)
            os.rmdir(fake_sudo_dir)
        except OSError:
            pass


if __name__ == "__main__":
    main()
