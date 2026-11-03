#!/usr/bin/env python3
"""Test forward and lateral motion of the mecanum sim (nav2 off)."""
import math
import os
import re
import subprocess
import time

env = os.environ.copy()
env["GZ_PARTITION"] = os.environ.get("GZ_PARTITION", "mec3")


def pose():
    out = subprocess.run(["gz", "model", "-m", "car1", "-p"],
                         capture_output=True, text=True, env=env).stdout
    blocks = re.findall(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    if len(blocks) < 2:
        return None
    return tuple(float(v) for v in blocks[0]) + (float(blocks[1][2]),)


def cmd(x, y, seconds=2.0):
    t0 = time.time()
    while time.time() - t0 < seconds:
        subprocess.run(["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
                        "-p", "linear: { x: %f, y: %f }" % (x, y)],
                       capture_output=True, env=env)
        time.sleep(0.05)


start = None
for _ in range(60):
    start = pose()
    if start is not None:
        break
    time.sleep(2)
print("start", start)

cmd(0.2, 0.0)
p = pose()
print("after +vx 0.2 :", p, "dx=%.3f dy=%.3f" % (p[0] - start[0], p[1] - start[1]))

time.sleep(0.5)
cmd(0.0, 0.2)
p2 = pose()
print("after +vy 0.2 :", p2, "dx=%.3f dy=%.3f" % (
    p2[0] - p[0], p2[1] - p[1]))

time.sleep(0.5)
cmd(-0.2, 0.0)
p3 = pose()
print("after -vx 0.2 :", p3, "dx=%.3f dy=%.3f" % (
    p3[0] - p2[0], p3[1] - p2[1]))
