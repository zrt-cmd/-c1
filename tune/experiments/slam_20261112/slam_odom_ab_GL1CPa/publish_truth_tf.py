import math
import threading
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

rclpy.init()
ros = rclpy.create_node("gazebo_truth_odometry")
publisher = ros.create_publisher(TFMessage, "/truth_tf", 100)
transport = Node()
last = None
lock = threading.Lock()

def receive(msg):
    global last
    for p in msg.pose:
        if p.name != "car1":
            continue
        stamp = msg.header.stamp
        q = p.orientation
        heading = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.header.stamp.sec = stamp.sec
        t.header.stamp.nanosec = stamp.nsec
        t.transform.translation.x = p.position.x-.15
        t.transform.translation.y = p.position.y-1.70
        t.transform.rotation.z = math.sin(heading/2)
        t.transform.rotation.w = math.cos(heading/2)
        with lock:
            last = t
        break

def publish():
    with lock:
        transform = last
    if transform is not None:
        publisher.publish(TFMessage(transforms=[transform]))

transport.subscribe(Pose_V, "/world/math_room/pose/info", receive)
ros.create_timer(.015, publish)
try:
    rclpy.spin(ros)
except KeyboardInterrupt:
    pass
finally:
    ros.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
