#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
任务1 · 订阅者
订阅 /ball_pos，收到一条就打印 id / x / y。
"""
import rclpy
from rclpy.node import Node

from cquick_ball.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.count = 0
        self.sub = self.create_subscription(
            BallPos, 'ball_pos', self.callback, 10)

    def callback(self, msg):
        self.count += 1
        self.get_logger().info(
            f'收到 [#{self.count}]: id={msg.id}, x={msg.x:.2f}, y={msg.y:.2f}')


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
