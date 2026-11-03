#!/usr/bin/env python3
"""Echo the Gazebo /cmd_vel topic while publishing vy from ROS."""
import os
import subprocess
import time

env = os.environ.copy()
echo = subprocess.Popen(["gz", "topic", "-e", "-t", "/cmd_vel"],
                        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
time.sleep(2)
pub = subprocess.Popen(
    ["ros2", "topic", "pub", "-r", "10", "/cmd_vel",
     "geometry_msgs/msg/Twist", "{linear: {y: 0.2}}"],
    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(2.5)
pub.terminate()
time.sleep(0.5)
echo.terminate()
out = echo.stdout.read().decode(errors="ignore")
print("gz received:", len(out), "bytes")
print("\n".join(out.splitlines()[:12]))
