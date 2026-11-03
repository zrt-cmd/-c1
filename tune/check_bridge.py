#!/usr/bin/env python3
"""Check whether ROS /cmd_vel reaches the Gazebo topic."""
import os
import subprocess
import time

env = os.environ.copy()
env["GZ_PARTITION"] = "vc4"
env["ROS_DOMAIN_ID"] = "170"

echo = subprocess.Popen(
    ["gz", "topic", "-e", "-t", "/cmd_vel"],
    env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
time.sleep(3)
try:
    subprocess.run(["ros2", "topic", "pub", "-r", "10", "/cmd_vel",
                    "geometry_msgs/msg/Twist", "{linear: {x: 0.2}}"],
                   env=env, timeout=4, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
except subprocess.TimeoutExpired:
    pass
time.sleep(0.5)
echo.terminate()
out = echo.stdout.read().decode(errors="ignore")
print("echo length:", len(out))
print(out[:300])
