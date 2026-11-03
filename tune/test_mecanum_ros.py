#!/usr/bin/env python3
"""Steady 20 Hz ROS commands: forward, lateral, back."""
import math
import os
import re
import subprocess
import time

env = os.environ.copy()
env["GZ_PARTITION"] = os.environ.get("GZ_PARTITION", "vc2")


def pose():
    out = subprocess.run(["gz", "model", "-m", "car1", "-p"],
                         capture_output=True, text=True, env=env).stdout
    blocks = re.findall(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    if len(blocks) < 2:
        return None
    return tuple(float(v) for v in blocks[0]) + (float(blocks[1][2]),)


def move(x, y, seconds):
    cmd = ["ros2", "topic", "pub", "-r", "20", "/cmd_vel",
           "geometry_msgs/msg/Twist",
           "{linear: {x: %s, y: %s}}" % (x, y)]
    p = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    time.sleep(seconds)
    p.terminate()
    time.sleep(0.4)
    return pose()


start = None
for _ in range(60):
    start = pose()
    if start is not None:
        break
    time.sleep(2)
print("start", start)
time.sleep(5)  # let the robot settle on the ground before commanding
print("settled", pose())

p = move(0.15, 0.0, 3.0)
print("+vx3s:", p, "dx=%.3f dy=%.3f" % (p[0] - start[0], p[1] - start[1]))
q = move(0.0, 0.15, 3.0)
print("+vy3s:", q, "dx=%.3f dy=%.3f" % (q[0] - p[0], q[1] - p[1]))
