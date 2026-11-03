#!/usr/bin/env python3
"""Steady 20 Hz ROS command characterization: vx, vy, wz."""
import math
import os
import re
import subprocess
import time

env = os.environ.copy()


def pose():
    out = subprocess.run(["gz", "model", "-m", "car1", "-p"],
                         capture_output=True, text=True, env=env).stdout
    blocks = re.findall(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    if len(blocks) < 2:
        return None
    return (float(blocks[0][0]), float(blocks[0][1]), float(blocks[1][2]))


start = None
for _ in range(60):
    start = pose()
    if start is not None:
        break
    time.sleep(2)
if start is None:
    raise SystemExit("no model")
time.sleep(4)


def step(name, yaml, seconds=3.0):
    p0 = pose()
    pub = subprocess.Popen(
        ["ros2", "topic", "pub", "-r", "20", "/cmd_vel",
         "geometry_msgs/msg/Twist", yaml],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(seconds)
    pub.terminate()
    time.sleep(0.3)
    p1 = pose()
    print("%s: dx=%+.3f dy=%+.3f dyaw=%+.1f" % (
        name, p1[0] - p0[0], p1[1] - p0[1],
        math.degrees(p1[2] - p0[2])))


step("vx=+0.2", "{linear: {x: 0.2}}")
step("vy=+0.2", "{linear: {y: 0.2}}")
step("wz=+0.5", "{angular: {z: 0.5}}")
