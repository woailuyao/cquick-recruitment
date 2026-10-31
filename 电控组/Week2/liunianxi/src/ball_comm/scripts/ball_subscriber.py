#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务1 · 订阅者：订阅 /ball_pos，把 id / x / y 打印出来。"""
import rclpy
from rclpy.node import Node

from ball_comm.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.count = 0
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.on_msg, 10)
        self.get_logger().info('订阅者已启动：等待 /ball_pos ...')

    def on_msg(self, msg):
        self.count += 1
        self.get_logger().info('收到 id=%d  x=%.2f  y=%.2f  （第 %d 条）'
                               % (msg.id, msg.x, msg.y, self.count))


def main():
    rclpy.init()
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
