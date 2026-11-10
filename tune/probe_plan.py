"""Request an independent ThetaStar path and save it for comparison."""
import argparse
import json
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose

parser=argparse.ArgumentParser()
parser.add_argument('--out',type=Path,required=True)
args=parser.parse_args()
rclpy.init()
node=rclpy.create_node('reference_path_probe',parameter_overrides=[
    rclpy.parameter.Parameter('use_sim_time',value=True)])
client=ActionClient(node,ComputePathToPose,'compute_path_to_pose')
if not client.wait_for_server(timeout_sec=60): raise RuntimeError('Planner unavailable')
goal=ComputePathToPose.Goal(); goal.planner_id='GridBased'; goal.use_start=True
for pose,x,y in ((goal.start,.15,1.7),(goal.goal,-1.8,1.2)):
    pose.header.frame_id='map'; pose.header.stamp=node.get_clock().now().to_msg()
    pose.pose.position.x=x; pose.pose.position.y=y; pose.pose.orientation.w=1.
future=client.send_goal_async(goal)
rclpy.spin_until_future_complete(node,future,timeout_sec=30)
handle=future.result()
if handle is None or not handle.accepted: raise RuntimeError('Planner rejected request')
future=handle.get_result_async(); rclpy.spin_until_future_complete(node,future,timeout_sec=30)
result=future.result()
if result is None: raise RuntimeError('Planner timeout')
data={'status':result.status,'error_code':result.result.error_code,
      'path':[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]}
args.out.parent.mkdir(parents=True,exist_ok=True)
args.out.write_text(json.dumps(data,indent=2)); print(json.dumps(data))
node.destroy_node(); rclpy.shutdown()
