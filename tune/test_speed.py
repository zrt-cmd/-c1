#!/usr/bin/env python3
"""Command 0.1 m/s directly to Gazebo and sample ground-truth motion."""
import os
import re
import subprocess
import time

env = os.environ.copy()
env["GZ_PARTITION"] = os.environ.get("GZ_PARTITION", "d2")


def pose():
    out = subprocess.run(["gz", "model", "-m", "car1", "-p"],
                         capture_output=True, text=True, env=env).stdout
    m = re.search(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    r = re.search(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out[out.find("Pose"):])
    return [float(m.group(i)) for i in (1, 2, 3)] if m else None


start = None
for _ in range(60):
    start = pose()
    if start is not None:
        break
    time.sleep(2)
print("start", start)
for i in range(30):
    subprocess.run(["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
                    "-p", "linear: { x: 0.1 }"], capture_output=True, env=env)
    time.sleep(0.1)
    if i % 5 == 4:
        p = pose()
        print("t=%.1fs %s" % ((i + 1) * 0.1, p))
time.sleep(0.5)
print("end", pose())
