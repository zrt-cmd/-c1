#!/usr/bin/env python3
"""Direct velocity demonstration along operator-selected waypoints; no Nav2."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument('--world', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--speed', type=float, default=.20)
parser.add_argument('--use-amcl', action='store_true', help='Control from map->base TF; truth is for measurement and stop guard only')
parser.add_argument('--max-waypoints', type=int, default=12)
args = parser.parse_args()
args.out.mkdir(parents=True, exist_ok=True)
route = [(0.15, 1.7), (-0.08, 1.7), (0.775, 1.7), (0.775, 1.15), (1.325, 1.15), (1.325, 1.725), (2.425, 1.725), (2.425, 0.625), (1.875, 0.625), (1.875, 1.18), (1.875, 0.625), (0.225, 0.625), (0.225, 1.125), (-0.625, 1.125), (-0.625, 1.73), (-0.625, 0.65), (-1.8, 0.65), (-1.8, 1.2), (-1.8, 1.58), (-2.27, 1.58), (-2.27, 0.65), (-1.0, 0.65), (-1.0, -0.24), (-2.22, -0.24), (-2.22, -1.5), (0.0, -1.5), (0.0, -2.85), (0.0, -1.5), (2.24, -1.5), (2.24, -0.22), (-1.0, -0.22), (-1.0, 0.65), (-0.625, 0.65)]
sweeps = set()
walls=[]
for model in ET.parse(args.world).findall('world/model'):
    box=model.find('link/collision/geometry/box/size')
    if box is not None:
        x,y,*_=map(float,model.findtext('pose').split())
        sx,sy,_=map(float,box.text.split())
        walls.append((model.get('name'),x,y,sx,sy))

def clearance(x,y,yaw=0):
    # Conservative rectangular body envelope including the wheel spheres.
    hx=.18*abs(math.cos(yaw))+.16*abs(math.sin(yaw))
    hy=.18*abs(math.sin(yaw))+.16*abs(math.cos(yaw))
    values=[]
    for name,cx,cy,sx,sy in walls:
        dx=abs(x-cx)-sx/2-hx; dy=abs(y-cy)-sy/2-hy
        d=math.hypot(max(dx,0),max(dy,0)) if max(dx,dy)>0 else max(dx,dy)
        values.append((d,name))
    return min(values)

for start,end in zip(route,route[1:]):
    for k in range(201):
        x=start[0]+(end[0]-start[0])*k/200
        y=start[1]+(end[1]-start[1])*k/200
        if clearance(x,y)[0]<.02:
            raise RuntimeError(f'Unsafe waypoint segment at {(x,y)}: {clearance(x,y)}')
(args.out/'route.json').write_text(json.dumps(route))

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.laserscan_pb2 import LaserScan
node=Node(); state={'t':None,'pose':None,'scan':None,'odom':None,'received':0}
def stats(m): state['t']=m.sim_time.sec+m.sim_time.nsec*1e-9
def poses(m):
    for p in m.pose:
        if p.name=='car1':
            q=p.orientation
            state['pose']=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            state['received']=time.monotonic()
def scan(m): state['scan']=min((v for v in m.ranges if math.isfinite(v) and v>0),default=None)
def odom(m): state['odom']=(m.pose.position.x,m.pose.position.y)
node.subscribe(WorldStatistics,'/world/math_room/stats',stats)
node.subscribe(Pose_V,'/world/math_room/pose/info',poses)
node.subscribe(Odometry,'/odom',odom)
node.subscribe(LaserScan,'/scan',scan)
publisher=node.advertise('/cmd_vel',Twist)
deadline=time.monotonic()+30
while (state['pose'] is None or state['t'] is None) and time.monotonic()<deadline: time.sleep(.05)
if state['pose'] is None: raise RuntimeError('No Gazebo robot pose')
if math.hypot(state['pose'][0]-.15,state['pose'][1]-1.7)>.05:
    raise RuntimeError('Robot is not at G; refusing to move')
start_time=state['t']; last_t=start_time; previous=(0,0); length=0; min_gap=100
status='running'; index=1; cmd=[0.,0.,0.]; last_log=-1
max_localization_error=0.
sweep_angle = None
swept = set()
if args.use_amcl:
    import rclpy
    from tf2_ros import Buffer, TransformListener, TransformException
    rclpy.init()
    ros_node=rclpy.create_node('reference_tracking_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    tf_buffer=Buffer(); tf_listener=TransformListener(tf_buffer,ros_node,spin_thread=True)
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        time.sleep(.01)
        if tf_buffer.can_transform('map','base_footprint',rclpy.time.Time()): break
    else: raise RuntimeError('No AMCL map->base transform')
fields=['sim_time','stage','x','y','yaw','cmd_vx','cmd_vy','cmd_wz','body_clearance','nearest_wall','scan_min','odom_x','odom_y','estimated_x','estimated_y','localization_error','tf_age_seconds']
try:
    with (args.out/'telemetry.csv').open('w',newline='') as f:
        writer=csv.writer(f); writer.writerow(fields)
        stage_start=state['t']; last_pose=state['pose']
        while index<len(route) and index<=args.max_waypoints:
            t=state['t']; x,y,yaw=state['pose']; dt=max(0,min(t-last_t,.2)); last_t=t
            if time.monotonic()-state['received']>2: raise RuntimeError('Pose stream stale')
            gap,wall=clearance(x,y,yaw); min_gap=min(min_gap,gap)
            if gap<.008: raise RuntimeError(f'Clearance guard stopped at {gap:.4f} m near {wall}')
            if abs(yaw)>.10 and sweep_angle is None: raise RuntimeError('Unexpected heading drift')
            if t-stage_start>90: raise RuntimeError('Waypoint timed out')
            cx,cy,cyaw=x,y,yaw
            tf_age=0.
            if args.use_amcl:
                stamped=tf_buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
                tf_age=ros_node.get_clock().now().nanoseconds/1e9-(stamped.header.stamp.sec+stamped.header.stamp.nanosec/1e9)
                if tf_age>.15: raise RuntimeError(f'TF is stale by {tf_age:.3f} simulation seconds')
                transform=stamped.transform
                cx,cy=transform.translation.x,transform.translation.y
                q=transform.rotation
                cyaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            loc_error=math.hypot(cx-x,cy-y); max_localization_error=max(max_localization_error,loc_error)
            gx,gy=route[index]; ex,ey=gx-cx,gy-cy; distance=math.hypot(ex,ey)
            if index in sweeps and index not in swept and (distance < .012 or sweep_angle is not None):
                if sweep_angle is None:
                    sweep_angle = 0.0
                    last_yaw = yaw
                sweep_angle += math.remainder(yaw-last_yaw, 2*math.pi)
                last_yaw = yaw
                remaining = 2*math.pi-sweep_angle
                if remaining > .018:
                    m = Twist()
                    m.angular.z = min(.7, max(.12, 2*remaining))
                    publisher.publish(m)
                    time.sleep(.025)
                    continue
                swept.add(index)
                sweep_angle = None
                publisher.publish(Twist())
            if distance<.012:
                print(json.dumps({'reached':index,'x':x,'y':y,'clearance':gap,'sim_time':t-start_time}),flush=True)
                index+=1; stage_start=t; publisher.publish(Twist()); cmd=[0.,0.,0.]
                continue
            speed=min(args.speed,1.6*distance)
            wx,wy=speed*ex/distance,speed*ey/distance
            target=[math.cos(cyaw)*wx+math.sin(cyaw)*wy,-math.sin(cyaw)*wx+math.cos(cyaw)*wy]
            for j in range(2): cmd[j]+=max(-.4*dt,min(.4*dt,target[j]-cmd[j]))
            cmd[2]=max(-.08,min(.08,-.8*cyaw))
            m=Twist(); m.linear.x,m.linear.y,m.angular.z=cmd; publisher.publish(m)
            length+=math.hypot(x-last_pose[0],y-last_pose[1]); last_pose=(x,y,yaw)
            if t-last_log>=.1:
                ox,oy=state['odom'] or (None,None)
                writer.writerow([t-start_time,index,x,y,yaw,*cmd,gap,wall,state['scan'],ox,oy,cx,cy,loc_error,tf_age]); f.flush(); last_log=t
            time.sleep(.025)
        status='survey_complete' if index == len(route) else 'partial'
except BaseException as exc:
    status=str(exc)
    raise
finally:
    for _ in range(5): publisher.publish(Twist()); time.sleep(.03)
    result={'status':status,'pose':state['pose'],'sim_seconds':state['t']-start_time,
            'path_length_m':length,'min_envelope_clearance_m':min_gap,'completed_waypoints':index-1,
            'pose_source':'amcl_tf' if args.use_amcl else 'gazebo_truth','max_localization_error_m':max_localization_error,
            'collision_measurement':'geometric envelope only; no physical contact sensor'}
    (args.out/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
    if args.use_amcl:
        tf_listener.executor.shutdown()
        tf_listener.dedicated_listener_thread.join(timeout=5)
        ros_node.destroy_node()
        rclpy.shutdown()
