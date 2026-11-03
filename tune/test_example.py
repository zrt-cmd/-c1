#!/usr/bin/env python3
"""Test whether gz-sim's official mecanum example actually strafes."""
import os
import re
import subprocess
import time

env = os.environ.copy()
env["GZ_PARTITION"] = os.environ.get("GZ_PARTITION", "ex1")


def pose():
    out = subprocess.run(["gz", "model", "-m", "vehicle_blue", "-p"],
                         capture_output=True, text=True, env=env).stdout
    m = re.search(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    return [float(m.group(i)) for i in (1, 2, 3)] if m else None


start = None
for _ in range(40):
    start = pose()
    if start is not None:
        break
    time.sleep(2)
print("start", start)
if start is None:
    raise SystemExit("no model")
time.sleep(3)


def cmd(msg, seconds=3.0):
    t0 = time.time()
    while time.time() - t0 < seconds:
        subprocess.run(["gz", "topic", "-t", "/model/vehicle_blue/cmd_vel",
                        "-m", "gz.msgs.Twist", "-p", msg],
                       capture_output=True, env=env)
        time.sleep(0.1)


cmd("linear: { y: 0.5 }")
time.sleep(0.5)
p = pose()
print("after vy=0.5", p, "dx=%.3f dy=%.3f" % (p[0] - start[0], p[1] - start[1]))

cmd("linear: { x: 0.5 }")
time.sleep(0.5)
q = pose()
print("after vx=0.5", q, "dx=%.3f dy=%.3f" % (q[0] - p[0], q[1] - p[1]))
