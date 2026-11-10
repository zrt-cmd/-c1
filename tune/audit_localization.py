"""Stationary localization, TF freshness and scan-versus-wall audit."""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

p=argparse.ArgumentParser(); p.add_argument('--world',type=Path,required=True); p.add_argument('--out',type=Path,required=True); args=p.parse_args()
rclpy.init(); node=rclpy.create_node('localization_audit',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
buffer=Buffer(); listener=TransformListener(buffer,node)
state={}; gz=GzNode()
def truth(msg):
    for pose in msg.pose:
        if pose.name=='car1': state['truth']=(pose.position.x,pose.position.y,heading(pose.orientation))
def heading(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
gz.subscribe(Pose_V,'/world/math_room/pose/info',truth)
node.create_subscription(LaserScan,'/scan',lambda m:state.update(scan=m),qos_profile_sensor_data)
node.create_subscription(Odometry,'/odom',lambda m:state.update(odom=m),20)
node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:state.update(amcl=m),20)
deadline=time.monotonic()+8
while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
tf=buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
laser=buffer.lookup_transform('base_footprint','laser_link',rclpy.time.Time()).transform
x,y,yaw=state['truth']; lx=x+math.cos(yaw)*laser.translation.x-math.sin(yaw)*laser.translation.y; ly=y+math.sin(yaw)*laser.translation.x+math.cos(yaw)*laser.translation.y
walls=[]
for model in ET.parse(args.world).findall('world/model'):
    size=model.findtext('link/collision/geometry/box/size')
    if size:
        cx,cy,*_=map(float,model.findtext('pose').split()); sx,sy,_=map(float,size.split()); walls.append((cx-sx/2,cx+sx/2,cy-sy/2,cy+sy/2))
def expected(angle):
    dx,dy=math.cos(angle),math.sin(angle); best=8.
    for xmin,xmax,ymin,ymax in walls:
        lo,hi=0.,8.
        for origin,direction,lower,upper in ((lx,dx,xmin,xmax),(ly,dy,ymin,ymax)):
            if abs(direction)<1e-9:
                if not lower<=origin<=upper: hi=-1
            else:
                a,b=sorted(((lower-origin)/direction,(upper-origin)/direction)); lo=max(lo,a); hi=min(hi,b)
        if hi>=lo: best=min(best,lo)
    return best
scan=state['scan']; errors=[]; rays=[]
for i,distance in enumerate(scan.ranges):
    angle=scan.angle_min+i*scan.angle_increment
    target=expected(yaw+heading(laser.rotation)+angle)
    if math.isfinite(distance): errors.append(abs(distance-target))
    rays.append({'angle':angle,'measured':distance if math.isfinite(distance) else None,'expected':target})
errors.sort(); odom=state['odom']; now=node.get_clock().now().nanoseconds/1e9
report={'truth':state['truth'],'tf_pose':[tf.transform.translation.x,tf.transform.translation.y,heading(tf.transform.rotation)],
        'tf_age_seconds':now-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9),
        'tf_position_error_m':math.hypot(tf.transform.translation.x-x,tf.transform.translation.y-y),
        'odom':[odom.pose.pose.position.x,odom.pose.pose.position.y,heading(odom.pose.pose.orientation)],
        'spawn_offset_odom_error_m':math.hypot(odom.pose.pose.position.x+.15-x,odom.pose.pose.position.y+1.7-y),
        'laser_offset':[laser.translation.x,laser.translation.y,laser.translation.z],
        'scan_frame':scan.header.frame_id,'scan_residual_median_m':errors[len(errors)//2],
        'scan_residual_p95_m':errors[int(len(errors)*.95)],'rays':rays}
args.out.parent.mkdir(parents=True,exist_ok=True); args.out.write_text(json.dumps(report,indent=2)); print(json.dumps({k:v for k,v in report.items() if k!='rays'}))
node.destroy_node(); rclpy.shutdown()
