#!/usr/bin/env python3
"""Check lateral physical motion in an already running full Gazebo launch."""

import copy
import math
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node


state = {'time': None, 'pose': None}
node = Node()


def on_stats(message):
    state['time'] = message.sim_time.sec + message.sim_time.nsec * 1e-9


def on_poses(message):
    for pose in message.pose:
        if pose.name == 'car1':
            state['pose'] = copy.deepcopy(pose)


node.subscribe(WorldStatistics, '/world/math_room/stats', on_stats)
node.subscribe(Pose_V, '/world/math_room/pose/info', on_poses)
publisher = node.advertise('/cmd_vel', Twist)

deadline = time.monotonic() + 40
while (state['time'] is None or state['pose'] is None) and time.monotonic() < deadline:
    time.sleep(0.05)
if state['pose'] is None:
    raise RuntimeError('No robot pose from the full Gazebo launch')

start = copy.deepcopy(state['pose'])
initial_time = state['time']
command = Twist()
command.linear.y = 0.05
deadline = time.monotonic() + 30
try:
    while state['time'] - initial_time < 2.0:
        if time.monotonic() > deadline:
            raise RuntimeError('Simulation stopped advancing')
        publisher.publish(command)
        time.sleep(0.02)
finally:
    publisher.publish(Twist())
time.sleep(0.4)
end = state['pose']
dx = end.position.x - start.position.x
dy = end.position.y - start.position.y
q = start.orientation
heading = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
body_x = math.cos(heading) * dx + math.sin(heading) * dy
body_y = -math.sin(heading) * dx + math.cos(heading) * dy
print(f'body_x={body_x:.4f} body_y={body_y:.4f}', flush=True)
if abs(body_x) > 0.02 or not (0.07 < body_y < 0.13):
    raise RuntimeError('Full-launch lateral motion did not match the command')
