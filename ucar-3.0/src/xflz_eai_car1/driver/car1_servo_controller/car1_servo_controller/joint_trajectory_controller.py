#!/usr/bin/env python3
# encoding: utf-8
"""FollowJointTrajectory action server for MoveIt2 integration."""
import time
import threading
from rclpy.action import ActionServer
from rclpy.duration import Duration
from rclpy.time import Time
from control_msgs.action import FollowJointTrajectory
from car1_servo_controller_msgs.msg import ServoPosition


class JointTrajectoryActionController:
    """Provides FollowJointTrajectory action for a group of joints."""

    def __init__(self, node, servo_manager, controller_name, action_name, controllers):
        """
        Args:
            node: Parent ROS2 node.
            servo_manager: ServoManager instance.
            controller_name: Controller identifier (e.g. 'arm_controller').
            action_name: Action server name (e.g. 'arm_controller/follow_joint_trajectory').
            controllers: List of JointPositionController instances.
        """
        self.node = node
        self.servo_manager = servo_manager
        self.controller_name = controller_name

        self.joint_names = []
        self.joint_to_controller = {}
        for c in controllers:
            self.joint_names.append(c.joint_name)
            self.joint_to_controller[c.joint_name] = c
        self.num_joints = len(self.joint_names)

        self.action_server = ActionServer(
            node,
            FollowJointTrajectory,
            action_name,
            self.execute_callback
        )
        self.node.get_logger().info(f'Trajectory action server started: {action_name}')

    def execute_callback(self, goal_handle):
        """Execute a joint trajectory goal."""
        traj = goal_handle.request.trajectory
        num_points = len(traj.points)

        if num_points == 0:
            self.node.get_logger().error('Incoming trajectory is empty')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        # Build joint index lookup
        lookup = []
        for joint in self.joint_names:
            if joint in traj.joint_names:
                lookup.append(traj.joint_names.index(joint))
            else:
                self.node.get_logger().error(f'Joint {joint} not found in trajectory')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                return result

        # Calculate segment durations
        durations = [0.0] * num_points
        durations[0] = (traj.points[0].time_from_start.sec +
                        traj.points[0].time_from_start.nanosec * 1e-9)
        for i in range(1, num_points):
            t_cur = (traj.points[i].time_from_start.sec +
                     traj.points[i].time_from_start.nanosec * 1e-9)
            t_prev = (traj.points[i-1].time_from_start.sec +
                      traj.points[i-1].time_from_start.nanosec * 1e-9)
            durations[i] = t_cur - t_prev

        # Execute each trajectory segment
        for seg in range(num_points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            # Skip zero-duration segments (typically the first point = current position)
            if durations[seg] < 0.001:
                continue

            # Build servo position commands
            positions = []
            for j, joint in enumerate(self.joint_names):
                desired_rad = traj.points[seg].positions[lookup[j]]
                controller = self.joint_to_controller[joint]
                pulse = controller.pos_rad_to_pulse(desired_rad)
                servo_pos = ServoPosition()
                servo_pos.id = controller.servo_id
                servo_pos.position = float(pulse)
                positions.append(servo_pos)

            self.servo_manager.set_position(durations[seg], positions)

            # Wait for segment to complete
            time.sleep(durations[seg])

        # Trajectory completed successfully
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.node.get_logger().info(
            f'{self.controller_name}: Trajectory execution completed'
        )
        return result
