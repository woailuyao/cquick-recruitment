#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务1 · 订阅者：订阅 /ball_pos，把收到的 id / x / y 打印出来。"""

import rclpy
from rclpy.node import Node

from cquick_ball_msgs.msg import BallPos


class BallSubscriber(Node):

    def __init__(self):
        super().__init__('ball_subscriber')
        # 话题名必须和发布者一字不差：ball_pos
        self.subscription = self.create_subscription(
            BallPos, 'ball_pos', self.on_ball, 10)
        self.count = 0
        self.get_logger().info('订阅者已启动，等待 /ball_pos ...')

    def on_ball(self, msg):
        self.count += 1
        self.get_logger().info(
            f'收到第 {self.count} 条 -> id={msg.id}, x={msg.x:.1f}, y={msg.y:.1f}')


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
