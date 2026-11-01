#!/usr/bin/env python3
"""任务1 订阅者：订阅 ball_pos，收到就打印 id / x / y。"""
import rclpy
from rclpy.node import Node

from ball_pos_pkg.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')

        # 话题名必须和发布者一字不差
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.callback, 10)

        self.get_logger().info('订阅者已启动，等待 /ball_pos ...')

    def callback(self, msg):
        """收到一条就打印一条。"""
        self.get_logger().info(f'收到 球{msg.id}: x={msg.x:.2f}, y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
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
