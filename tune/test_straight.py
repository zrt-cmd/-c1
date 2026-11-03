#!/usr/bin/env python3
"""Command forward velocity and log x, y, yaw from Gazebo ground truth."""
import math
import os
import re
import subprocess
import time

env = os.environ.copy()
env["GZ_PARTITION"] = os.environ.get("GZ_PARTITION", "d4")


def pose():
    out = subprocess.run(["gz", "model", "-m", "car1", "-p"],
                         capture_output=True, text=True, env=env).stdout
    blocks = re.findall(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    if len(blocks) < 2:
        return None
    x, y, _ = (float(v) for v in blocks[0])
    _, _, yaw = (float(v) for v in blocks[1])
    return x, y, yaw


start = None
for _ in range(60):
    start = pose()
    if start is not None:
        break
    time.sleep(2)
print("start", start)

pub = subprocess.Popen(
    ["ros2", "topic", "pub", "-r", "20", "/cmd_vel",
     "geometry_msgs/msg/Twist", "{linear: {x: 0.15}}"],
    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

t0 = time.time()
prev = start
while time.time() - t0 < 5:
    time.sleep(0.5)
    p = pose()
    if p and prev:
        print("t=%.1f pos=(%.3f,%.3f) yaw=%+.1fdeg step=%.3f" % (
            time.time() - t0, p[0], p[1], math.degrees(p[2]),
            math.hypot(p[0] - prev[0], p[1] - prev[1])))
        prev = p

pub.terminate()
time.sleep(0.5)
print("end", pose())
