from pathlib import Path
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
import rosbag2_py
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

directory = Path(sys.argv[1])/'sensors'
metadata = rosbag2_py.Info().read_metadata(str(directory), '')
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(directory),storage_id=metadata.storage_identifier),
            rosbag2_py.ConverterOptions('', ''))
types = {'/scan': LaserScan, '/truth_tf': TFMessage, '/tf_static': TFMessage,
         '/truth/pose': PoseWithCovarianceStamped, '/odom': Odometry}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
scans, static, truth, wheel = {}, {}, {}, {}
accepted = set()

def stamp(header):
    return header.stamp.sec*1_000_000_000 + header.stamp.nanosec

def record(table, header, pos, q):
    yaw = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]
    table[stamp(header)] = (pos.x,pos.y,yaw)

while reader.has_next():
    topic,data,_ = reader.read_next()
    msg = deserialize_message(data,types[topic])
    if topic == '/scan':
        scans[stamp(msg.header)] = msg
    elif topic == '/truth/pose':
        accepted.add(stamp(msg.header))
    elif topic == '/tf_static':
        for transform in msg.transforms:
            static[transform.child_frame_id] = transform
    elif topic == '/truth_tf':
        t = msg.transforms[0]
        record(truth,t.header,t.transform.translation,t.transform.rotation)
    elif topic == '/odom':
        p = msg.pose.pose
        record(wheel,msg.header,p.position,p.orientation)

times = sorted(accepted & scans.keys())
assert len(times)==len(accepted)
samples = {}
for label, series in [('truth',truth),('wheel',wheel)]:
    ticks = sorted(series)
    assert ticks[0] <= times[0] <= times[-1] <= ticks[-1]
    v = np.array([series[t] for t in ticks])
    samples[label] = np.column_stack([np.interp(times,ticks,v[:,0]),
                                     np.interp(times,ticks,v[:,1]),
                                     np.interp(times,ticks,np.unwrap(v[:,2]))])

rclpy.init()
node = rclpy.create_node('scan_matching_replay')
scan_pub = node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
static_pub = node.create_publisher(TFMessage,'/tf_static',
                     QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
dynamic = {label:node.create_publisher(TFMessage,f'/tf_{label}_replay',100)
           for label in samples}
import time
deadline = time.monotonic()+12
while scan_pub.get_subscription_count() < 1:
    if time.monotonic() > deadline:
        raise RuntimeError('SLAM must be subscribed before replay')
    rclpy.spin_once(node,timeout_sec=.05)
static_pub.publish(TFMessage(transforms=list(static.values())))
time.sleep(.5)
for i,t in enumerate(times):
    scan = scans[t]
    for label,values in samples.items():
        x,y,yaw=values[i]
        pose=TransformStamped()
        pose.header.stamp=scan.header.stamp
        pose.header.frame_id='odom'
        pose.child_frame_id='base_footprint'
        pose.transform.translation.x=float(x)
        pose.transform.translation.y=float(y)
        pose.transform.rotation.z=float(np.sin(yaw/2))
        pose.transform.rotation.w=float(np.cos(yaw/2))
        dynamic[label].publish(TFMessage(transforms=[pose]))
    time.sleep(.025)
    scan_pub.publish(scan)
    rclpy.spin_once(node,timeout_sec=.075)
print(f'Replayed {len(times)} timestamp-aligned scans',flush=True)
time.sleep(1)
node.destroy_node()
rclpy.shutdown()
