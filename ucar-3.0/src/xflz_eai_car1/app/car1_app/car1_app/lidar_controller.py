"""
Lidar obstacle avoidance node for mecanum robot.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarControllerNode(Node):
    """ROS2 node for lidar-based obstacle avoidance with mecanum motion."""

    def __init__(self):
        super().__init__('lidar_controller_node')

        # Declare parameters
        self.declare_parameter('obstacle_threshold', 0.35)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)

        # Get parameters
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'controller/cmd_vel',
            10
        )

        self.get_logger().info('Lidar obstacle avoidance node started.')

    def _get_sector_min_range(
        self,
        ranges: list,
        angle_min: float,
        angle_increment: float,
        sector_start_deg: float,
        sector_end_deg: float
    ) -> float:
        """
        Get minimum range in a sector.

        Args:
            ranges: List of range values from LaserScan
            angle_min: Minimum angle of the scan (radians)
            angle_increment: Angular distance between measurements (radians)
            sector_start_deg: Start of sector in degrees
            sector_end_deg: End of sector in degrees

        Returns:
            Minimum range value in the sector
        """
        num_readings = len(ranges)
        min_range = float('inf')

        sector_start_rad = math.radians(sector_start_deg)
        sector_end_rad = math.radians(sector_end_deg)

        for i in range(num_readings):
            angle = angle_min + i * angle_increment

            # Normalize angle to [0, 2*pi)
            angle_normalized = angle % (2 * math.pi)

            # Normalize sector boundaries to [0, 2*pi)
            start_norm = sector_start_rad % (2 * math.pi)
            end_norm = sector_end_rad % (2 * math.pi)

            # Check if angle falls within sector
            in_sector = False
            if start_norm <= end_norm:
                in_sector = start_norm <= angle_normalized <= end_norm
            else:
                # Sector wraps around 0 degrees
                in_sector = (
                    angle_normalized >= start_norm or
                    angle_normalized <= end_norm
                )

            if in_sector:
                r = ranges[i]
                if not math.isinf(r) and not math.isnan(r) and r > 0.01:
                    min_range = min(min_range, r)

        return min_range

    def scan_callback(self, msg: LaserScan):
        """Process incoming lidar scan for obstacle avoidance."""
        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Divide scan into 3 sectors (degrees)
        # Front: 330-360 and 0-30 (wrapping around)
        # Left:  60-120
        # Right: 240-300
        front_min = self._get_sector_min_range(
            ranges, angle_min, angle_increment, 330.0, 30.0
        )
        left_min = self._get_sector_min_range(
            ranges, angle_min, angle_increment, 60.0, 120.0
        )
        right_min = self._get_sector_min_range(
            ranges, angle_min, angle_increment, 240.0, 300.0
        )

        twist = Twist()
        threshold = self.obstacle_threshold

        if front_min < threshold:
            # Obstacle in front: decide avoidance direction
            if left_min < threshold and right_min < threshold:
                # Obstacles on all sides: move backward
                twist.linear.x = -self.linear_speed
                self.get_logger().debug('Obstacle on all sides, moving backward.')
            elif left_min >= right_min:
                # More space on the left: strafe left (mecanum)
                twist.linear.y = self.linear_speed
                self.get_logger().debug('Obstacle ahead, strafing left.')
            else:
                # More space on the right: strafe right (mecanum)
                twist.linear.y = -self.linear_speed
                self.get_logger().debug('Obstacle ahead, strafing right.')
        else:
            # No front obstacle: move forward
            twist.linear.x = self.linear_speed

            # Gentle avoidance if obstacle on one side
            if left_min < threshold:
                twist.linear.y = -self.linear_speed * 0.5
                self.get_logger().debug('Obstacle on left, slight strafe right.')
            elif right_min < threshold:
                twist.linear.y = self.linear_speed * 0.5
                self.get_logger().debug('Obstacle on right, slight strafe left.')

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LidarControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
