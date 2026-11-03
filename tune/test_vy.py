#!/usr/bin/env python3
"""Pure lateral command test: report dx, dy, dyaw after vy=0.2 for 3 s."""
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
    return (float(blocks[0][0]), float(blocks[0][1]),
            float(blocks[1][0]), float(blocks[1][1]), float(blocks[1][2]))


start = None
for _ in range(60):
    start = pose()
    if start is not None:
        break
    time.sleep(2)
if start is None:
    raise SystemExit("no model")
time.sleep(4)
start = pose()
print("start", start)


def cmd(x, y, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        subprocess.run(["gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
                        "-p", "linear: { x: %f, y: %f }" % (x, y)],
                       capture_output=True, env=env)
        time.sleep(0.05)


VY = float(os.environ.get("VY", "0.2"))
cmd(0.0, VY, 3.0)
time.sleep(0.5)
end = pose()
print("after vy=0.2", end)
print("RESULT dx=%.3f dy=%.3f dyaw=%.1fdeg" % (
    end[0] - start[0], end[1] - start[1],
    math.degrees(end[4] - start[4])))
