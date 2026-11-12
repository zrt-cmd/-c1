import math
from pathlib import Path
import json
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import rosbag2_py

out = Path(sys.argv[1])
directory = out / 'sensors'
metadata = rosbag2_py.Info().read_metadata(str(directory), '')
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id=metadata.storage_identifier),
            rosbag2_py.ConverterOptions('', ''))
topics = {'/truth_tf': TFMessage, '/odom': Odometry,
          '/wheel/pose': PoseWithCovarianceStamped, '/truth/pose': PoseWithCovarianceStamped}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
values = {topic: {} for topic in topics}

def record(table, stamp, position, q):
    t = stamp.sec + stamp.nanosec*1e-9
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    table[t] = [position.x, position.y, yaw]

while reader.has_next():
    topic, data, _ = reader.read_next()
    msg = deserialize_message(data, topics[topic])
    if topic == '/truth_tf':
        t = msg.transforms[0]
        record(values[topic],t.header.stamp,t.transform.translation,t.transform.rotation)
    else:
        p = msg.pose.pose
        record(values[topic],msg.header.stamp,p.position,p.orientation)

truth = np.array([[t,*p] for t,p in sorted(values['/truth_tf'].items())])
result = {}
for topic in ['/odom', '/wheel/pose', '/truth/pose']:
    poses = np.array([[t,*p] for t,p in sorted(values[topic].items())])
    keep = (poses[:,0] >= truth[0,0]) & (poses[:,0] <= truth[-1,0])
    poses = poses[keep]
    actual = np.column_stack([
        np.interp(poses[:,0],truth[:,0],truth[:,axis] if axis<3 else np.unwrap(truth[:,axis]))
        for axis in [1,2,3]])
    diff = poses[:,1:] - actual
    diff[:,2] = np.rad2deg(np.arctan2(np.sin(diff[:,2]),np.cos(diff[:,2])))
    xy = np.linalg.norm(diff[:,:2],axis=1)
    for label, mask in [('outbound',np.abs(actual[:,2]) < .15),
                        ('return',np.abs(actual[:,2]) > 3.0)]:
        if not mask.any():
            continue
        result[f'{topic}:{label}'] = {
            'samples': int(mask.sum()), 'position_median_m': float(np.median(xy[mask])),
            'position_p95_m': float(np.percentile(xy[mask],95)),
            'yaw_bias_deg': float(np.median(diff[mask,2])),
            'yaw_abs_p95_deg': float(np.percentile(np.abs(diff[mask,2]),95)),
            'last_xy_error_m': float(xy[mask][-1])}
    bad = (np.abs(actual[:,2]) > 3.) & (np.abs(diff[:,2]) > 5.)
    runs = []
    for block in np.split(np.flatnonzero(bad), np.where(np.diff(np.flatnonzero(bad)) != 1)[0]+1):
        if not len(block):
            continue
        start,end = int(block[0]),int(block[-1])
        runs.append(dict(start_sim_s=round(float(poses[start,0]),2),
                         end_sim_s=round(float(poses[end,0]),2),
                         start_world_xy=np.round(actual[start,:2]+[.15,1.70],3).tolist(),
                         end_world_xy=np.round(actual[end,:2]+[.15,1.70],3).tolist(),
                         peak_yaw_deg=round(float(np.max(np.abs(diff[block,2]))),2),
                         peak_position_cm=round(float(np.max(xy[block])*100),2)))
    result[f'{topic}:return_heading_error_runs'] = runs
print(json.dumps(result,indent=2))
(out/'pose_comparison.json').write_text(json.dumps(result,indent=2))
