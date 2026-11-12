import json
from pathlib import Path
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from PIL import Image
from rclpy.serialization import deserialize_message
import rosbag2_py
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import yaml

out = Path(sys.argv[1])
resolution = float(sys.argv[2]) if len(sys.argv)>2 else .02
name = 'map_truth_direct' if resolution == .02 else 'map_truth_direct_1cm'
bag = out/'sensors'
metadata = rosbag2_py.Info().read_metadata(str(bag), '')
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata.storage_identifier),
            rosbag2_py.ConverterOptions('', ''))
types = {'/scan': LaserScan, '/truth_tf': TFMessage, '/tf_static': TFMessage,
         '/truth/pose': PoseWithCovarianceStamped}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
scan_frames = {}
accepted = set()
truth = {}
static_frames = {}

def stamp(header):
    return header.stamp.sec*1_000_000_000 + header.stamp.nanosec

def rotation(q):
    return Rotation.from_quat([q.x,q.y,q.z,q.w])

def xyz(v):
    return np.array([v.x,v.y,v.z])

while reader.has_next():
    topic, data, _ = reader.read_next()
    msg = deserialize_message(data, types[topic])
    if topic == '/scan':
        scan_frames[stamp(msg.header)] = msg
    elif topic == '/truth/pose':
        accepted.add(stamp(msg.header))
    elif topic == '/truth_tf':
        t = msg.transforms[0]
        truth[stamp(t.header)] = (*xyz(t.transform.translation)[:2],
                                  rotation(t.transform.rotation).as_euler('xyz')[2])
    else:
        for t in msg.transforms:
            static_frames[t.child_frame_id] = t

scan_times = sorted(accepted & scan_frames.keys())
assert len(scan_times) >= .97*len(accepted), (len(scan_times),len(accepted))
truth_times = np.array(sorted(truth),dtype=np.int64)
truth_samples = np.array([truth[t] for t in truth_times])
scan_times = [t for t in scan_times if truth_times[0] <= t <= truth_times[-1]]
xy_yaw = np.column_stack([np.interp(scan_times,truth_times,truth_samples[:,axis] if axis<2
                                   else np.unwrap(truth_samples[:,axis]))
                          for axis in range(3)])
frame = scan_frames[scan_times[0]].header.frame_id
offset = np.zeros(3)
heading = Rotation.identity()
while frame != 'base_footprint':
    t = static_frames[frame]
    r = rotation(t.transform.rotation)
    offset = r.apply(offset)+xyz(t.transform.translation)
    heading = r*heading
    frame = t.header.frame_id

origin = np.array([-10.,-12.])
size = int(np.ceil(20./resolution))
hits = np.zeros(size*size,dtype=np.int32)
misses = np.zeros_like(hits)
all_endpoints = []

def indices(points):
    ij = np.floor((points-origin)/resolution).astype(np.int32)
    valid = ((ij>=0)&(ij<size)).all(axis=-1)
    return ij[...,0][valid]+size*ij[...,1][valid]

for t,(x,y,yaw) in zip(scan_times,xy_yaw):
    msg = scan_frames[t]
    assert msg.header.frame_id == scan_frames[scan_times[0]].header.frame_id
    c,s = np.cos(yaw),np.sin(yaw)
    rot = np.array([[c,-s],[s,c]])
    sensor_xy = [x,y]+rot@offset[:2]
    ranges = np.asarray(msg.ranges,dtype=float)
    good = (ranges>=msg.range_min)&(ranges<=msg.range_max)&np.isfinite(ranges)
    usable = good | np.isposinf(ranges)
    angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
    directions = heading.apply(np.column_stack((np.cos(angles),np.sin(angles),
                                                np.zeros_like(angles))))[:,:2]@rot.T
    endpoints = sensor_xy+directions[good]*ranges[good,None]
    all_endpoints.append(endpoints)
    hit_ids = np.unique(indices(endpoints))
    length = np.where(good[usable],np.maximum(0.,ranges[usable]-resolution),
                      float(msg.range_max))
    distance = np.arange(0.,float(msg.range_max),resolution/2)
    ray_points = sensor_xy + (directions[usable,None,:]*distance[None,:,None])
    free_ids = np.unique(indices(ray_points[distance[None,:]<=length[:,None]]))
    misses[np.setdiff1d(free_ids,hit_ids,assume_unique=True)] += 1
    hits[hit_ids] += 1

log_odds = .85*hits - .4*misses
grid = np.full(len(hits),205,dtype=np.uint8)
grid[log_odds< -1.41]=254
grid[log_odds> .62]=0
grid = grid.reshape(size,size)
observed = np.argwhere(grid!=205)
y0,x0 = np.maximum(0,observed.min(axis=0)-2)
y1,x1 = np.minimum(size,observed.max(axis=0)+3)
Image.fromarray(np.flipud(grid[y0:y1,x0:x1])).save(out/f'{name}.pgm')
meta = dict(image=f'{name}.pgm', mode='trinary', resolution=resolution,
            origin=[float(origin[0]+x0*resolution),float(origin[1]+y0*resolution),0.],
            negate=0,occupied_thresh=.65,free_thresh=.196)
(out/f'{name}.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
np.save(out/f'{name}_endpoints.npy',np.concatenate(all_endpoints))
result = dict(accepted_scans=len(accepted), projected_scans=len(scan_times), laser_offset_m=offset.tolist(),
              angle_min_deg=float(np.rad2deg(scan_frames[scan_times[0]].angle_min)),
              angle_max_deg=float(np.rad2deg(scan_frames[scan_times[0]].angle_max)))
(out/f'{name}_source.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
