#!/usr/bin/env python3
"""Initialize AMCL from the known G start, with an explicit covariance."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--settle-updates', type=int, default=30)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('initialize_known_start', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    latest = []
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', latest.append, 10)
    client = node.create_client(GetState, '/amcl/get_state')
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                if (future.done() and future.result().current_state.id == 3
                        and publisher.get_subscription_count() > 0
                        and node.get_clock().now().nanoseconds > 0):
                    break
        else:
            raise RuntimeError('AMCL not active or initialpose subscriber missing')

        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        # The vehicle is stationary at the known start. Allow for the bridge's
        # small TF publishing delay rather than requesting a future odom transform.
        message.header.stamp = (node.get_clock().now() - rclpy.duration.Duration(seconds=.05)).to_msg()
        message.pose.pose.position.x = .15
        message.pose.pose.position.y = 1.70
        message.pose.pose.orientation.w = 1.
        message.pose.covariance[0] = .0025
        message.pose.covariance[7] = .0025
        message.pose.covariance[35] = .01
        latest.clear()
        publisher.publish(message)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if latest:
                pose = latest[-1]
                # An estimate published after initialization can carry the earlier
                # scan's timestamp. Subsequent forced scans confirm new updates.
                if pose.header.frame_id == 'map':
                    first_estimate = [pose.pose.pose.position.x, pose.pose.pose.position.y]
                    # AMCL normally skips further stationary scans. Request distinct
                    # scan updates before starting, without moving or using truth.
                    no_motion = node.create_client(Empty, '/request_nomotion_update')
                    if args.settle_updates and not no_motion.wait_for_service(timeout_sec=10):
                        raise RuntimeError('AMCL stationary update service unavailable')
                    settled = []
                    for _ in range(args.settle_updates):
                        count = len(latest)
                        future = no_motion.call_async(Empty.Request())
                        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                        if not future.done():
                            raise RuntimeError('Stationary update request timed out')
                        update_deadline = time.monotonic() + 5
                        while len(latest) <= count and time.monotonic() < update_deadline:
                            rclpy.spin_once(node, timeout_sec=.05)
                        if len(latest) <= count:
                            raise RuntimeError('No new stationary scan estimate')
                        pose = latest[-1]
                        settled.append([pose.pose.pose.position.x, pose.pose.pose.position.y,
                                        *[pose.pose.covariance[i] for i in (0, 7, 35)]])
                    report = {'known_start': [.15, 1.7, 0.],
                              'requested_variances': [.0025, .0025, .01],
                              'first_estimate': first_estimate,
                              'stationary_estimates': settled,
                              'amcl_pose': [pose.pose.pose.position.x, pose.pose.pose.position.y],
                              'reported_variances': [pose.pose.covariance[i] for i in (0, 7, 35)]}
                    args.out.parent.mkdir(parents=True, exist_ok=True)
                    args.out.write_text(json.dumps(report, indent=2))
                    print(json.dumps(report), flush=True)
                    return
        raise RuntimeError('AMCL did not acknowledge initial pose with a fresh estimate')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
