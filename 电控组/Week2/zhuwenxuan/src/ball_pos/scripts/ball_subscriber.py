#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 1 · 订阅端：接收 ball_pos 并打印 id / x / y"""
import rclpy
from rclpy.node import Node

from ball_pos.msg import BallPos


class BallSubscriber(Node):

    def __init__(self):
        super().__init__('ball_subscriber')
        self.count = 0
        # 话题名必须和发布端一字不差
        self.create_subscription(BallPos, 'ball_pos', self.callback, 10)
        self.get_logger().info('ball_subscriber 已启动，等待 /ball_pos ...')

    def callback(self, msg):
        self.count += 1
        self.get_logger().info(
            '收到第 %d 条  id=%d  x=%.2f  y=%.2f'
            % (self.count, msg.id, msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
