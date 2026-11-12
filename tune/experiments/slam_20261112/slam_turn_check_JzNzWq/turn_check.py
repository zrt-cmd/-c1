import json
import math
import sys
import time
from pathlib import Path
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

node = Node()
state = {}

def pose(msg):
    q = msg.orientation
    return (msg.position.x, msg.position.y,
            math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)))

def truth(msg):
    for p in msg.pose:
        if p.name == 'car1':
            state['truth'] = pose(p)
            break

def odom(msg):
    state['odom'] = pose(msg.pose)

node.subscribe(Pose_V, '/world/math_room/pose/info', truth)
node.subscribe(Odometry, '/odom', odom)
pub = node.advertise('/cmd_vel', Twist)
deadline = time.monotonic() + 35
while not {'truth', 'odom'} <= state.keys():
    if time.monotonic() >= deadline:
        raise RuntimeError('Missing Gazebo pose or odometry')
    time.sleep(.05)

def wrap(angle):
    return math.remainder(angle, 2*math.pi)

def send(v=0., w=0.):
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)

def settle():
    for _ in range(12):
        send()
        time.sleep(.025)
    return state.copy()

def turn(target, speed):
    deadline = time.monotonic()+25
    while True:
        err = wrap(target-state['truth'][2])
        if abs(err) < .006:
            return settle()
        if time.monotonic() >= deadline:
            raise RuntimeError('Turn timed out')
        send(w=math.copysign(min(speed,max(.07,3*abs(err))),err))
        time.sleep(.02)

def straight(distance):
    x,y,yaw = state['truth']
    deadline = time.monotonic()+30
    while True:
        px,py,heading = state['truth']
        progress = math.cos(yaw)*(px-x)+math.sin(yaw)*(py-y)
        err = distance-progress
        if abs(err) < .009:
            return settle()
        if time.monotonic() >= deadline:
            raise RuntimeError('Straight segment timed out')
        send(v=math.copysign(min(.25,max(.045,1.5*abs(err))),err),
             w=2*wrap(yaw-heading))
        time.sleep(.02)

def delta(a,b):
    return [b[0]-a[0],b[1]-a[1],math.degrees(wrap(b[2]-a[2]))]

def xy_difference(a,b):
    return math.hypot(a[0]-b[0],a[1]-b[1])

reports = []
initial = settle()
frame_offset = wrap(initial['truth'][2]-initial['odom'][2])
cosine,sine = math.cos(frame_offset),math.sin(frame_offset)
def align(odom_delta):
    x,y,z = odom_delta
    return [cosine*x-sine*y,sine*x+cosine*y,z]

try:
    for speed,sign in [(.7,1),(.7,-1),(.3,1),(.3,-1)]:
        before = settle()
        rotated = turn(before['truth'][2]+sign*math.pi/2,speed)
        after = straight(1.)
        t_turn = delta(before['truth'],rotated['truth'])
        o_turn = align(delta(before['odom'],rotated['odom']))
        t_straight = delta(rotated['truth'],after['truth'])
        o_straight = align(delta(rotated['odom'],after['odom']))
        report = dict(command_speed_rad_s=speed, direction='left' if sign>0 else 'right',
                      truth_turn=t_turn, odom_turn=o_turn,
                      yaw_error_deg=math.remainder(o_turn[2]-t_turn[2],360),
                      false_turn_translation_m=xy_difference(t_turn,o_turn),
                      truth_forward=t_straight, odom_forward=o_straight,
                      forward_vector_error_m=xy_difference(t_straight,o_straight))
        reports.append(report)
        Path(sys.argv[1]).write_text(json.dumps(reports,indent=2))
        print(json.dumps(report),flush=True)
        straight(-1.)
        turn(initial['truth'][2],speed)
finally:
    settle()
