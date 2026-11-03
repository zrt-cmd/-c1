#!/usr/bin/env python3
import math
import rclpy
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node('scan_inspect')


def cb(msg):
    vals = []
    for i, r in enumerate(msg.ranges):
        if math.isfinite(r) and r > 0.01:
            a = msg.angle_min + i * msg.angle_increment
            vals.append((r, a))
    vals.sort()
    print(f'valid={len(vals)} inf={len(msg.ranges) - len(vals)}')
    print('closest:')
    for r, a in vals[:8]:
        print(f'  {r:.3f} m at {math.degrees(a):7.1f} deg')
    print('samples:')
    for deg in (0, 45, 90, 135, 180, -135, -90, -45):
        i = int(round((math.radians(deg) - msg.angle_min) / msg.angle_increment))
        if 0 <= i < len(msg.ranges):
            print(f'  {deg:5d} deg -> {msg.ranges[i]:.3f}')
    rclpy.shutdown()


node.create_subscription(LaserScan, '/scan', cb, 10)
rclpy.spin(node)
