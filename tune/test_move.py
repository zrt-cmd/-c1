#!/usr/bin/env python3
"""Publish a forward velocity directly to Gazebo and check ground-truth motion."""
import os
import re
import subprocess
import time

env = os.environ.copy()
env["GZ_PARTITION"] = "movetest"


def pose():
    out = subprocess.run(["gz", "model", "-m", "car1", "-p"],
                         capture_output=True, text=True, env=env).stdout
    m = re.search(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    return [float(m.group(i)) for i in (1, 2, 3)] if m else None


start = None
for _ in range(45):
    start = pose()
    if start is not None:
        break
    time.sleep(2)
print("start", start)
if start is None:
    raise SystemExit("no model")

for _ in range(50):
    subprocess.run(["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
                    "-p", "linear: { x: 0.5 }"],
                   capture_output=True, env=env)
    time.sleep(0.1)
time.sleep(1.0)
end = pose()
print("end  ", end)
if end:
    print("moved %.3f m" % ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5)
