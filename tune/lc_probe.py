#!/usr/bin/env python3
"""Poll Nav2 lifecycle states to find the node that stalls activation."""
import subprocess
import time

NODES = [
    "/map_server",
    "/amcl",
    "/controller_server",
    "/planner_server",
    "/behavior_server",
    "/bt_navigator",
    "/velocity_smoother",
]

start = time.time()
while time.time() - start < 110:
    parts = []
    for n in NODES:
        try:
            out = subprocess.check_output(
                ["ros2", "lifecycle", "get", n],
                stderr=subprocess.DEVNULL, timeout=4).decode().strip()
            st = out.split()[0] if out else "?"
        except Exception:
            st = "-"
        parts.append(f"{n}={st}")
    print(f"[{time.time()-start:5.1f}] " + " ".join(parts), flush=True)
    time.sleep(5)
print("DONE")
