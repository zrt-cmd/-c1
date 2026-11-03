#!/usr/bin/env python3
"""发布 /robot_description 话题（RViz RobotModel 从该话题读 URDF）。

bringup 的 robot_state_publisher / joint_state_publisher 只把 robot_description
作为参数读 URDF 发 TF，均不发布 /robot_description 话题；而 RViz 的 RobotModel
默认 Description Source=Topic（navigation.rviz / slam.rviz / car1.rviz），
从 /robot_description 话题读 URDF。无发布者 → 模型不显示。本节点补齐该话题。

用法：
    ros2 run car1_bringup robot_description_pub
"""
import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotDescriptionPub(Node):
    def __init__(self):
        super().__init__('robot_description_pub')
        urdf_file = os.path.expanduser(
            '~/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf')
        if not os.path.isfile(urdf_file):
            # 兜底：从 ament index 找（install 后）
            from ament_index_python.packages import get_package_share_directory
            try:
                urdf_file = os.path.join(
                    get_package_share_directory('car1_description'), 'urdf', 'car1.urdf')
            except Exception:
                pass
        try:
            with open(urdf_file, 'r') as f:
                self._desc = f.read()
        except Exception as e:
            self.get_logger().error(f'读取 URDF 失败: {e}')
            self._desc = ''
        # TRANSIENT_LOCAL：发布一次，后加入的订阅者（RViz 后启动）也能收到
        self._pub = self.create_publisher(
            String, '/robot_description', self._qos())
        self._timer = self.create_timer(2.0, self._publish)
        self.get_logger().info(
            f'robot_description_pub 就绪（{len(self._desc)} bytes），持续发布 /robot_description')

    @staticmethod
    def _qos():
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

    def _publish(self):
        if self._desc:
            msg = String()
            msg.data = self._desc
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
