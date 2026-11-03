#!/usr/bin/env python3
"""Steady 20 Hz ROS velocity command; sample ground truth pose."""
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
    m = re.search(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    return [float(m.group(i)) for i in (1, 2, 3)] if m else None


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
last = start
while time.time() - t0 < 5:
    time.sleep(0.5)
    p = pose()
    if p and last:
        print("t=%.1f pos=(%.3f,%.3f) step=%.3f" % (
            time.time() - t0, p[0], p[1],
            math.hypot(p[0] - last[0], p[1] - last[1])))
    last = p

pub.terminate()
time.sleep(1)
print("end", pose())
