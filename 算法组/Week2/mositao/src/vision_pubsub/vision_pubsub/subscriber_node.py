#!/usr/bin/env python3
"""订阅节点：订阅 /target_position，打印 detected / u / v / area。"""
import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.sub = self.create_subscription(
            TargetPosition, '/target_position', self.callback, 10)
        self.get_logger().info('已订阅 /target_position')

    def callback(self, msg):
        self.get_logger().info(
            '收到: detected=%s u=%.1f v=%.1f area=%.1f'
            % (msg.detected, msg.u, msg.v, msg.area))


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
