#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务一：订阅 /ball_pos，把收到的 id / x / y 打印出来。

改动很小，尽量保持原来的输出格式（截图能对得上）：
1. 启动时打一行提示，方便确认「节点起来了、在等话题」；
2. Ctrl+C 退出时不再打印 Traceback。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ball_pos_pkg.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')

        # 话题名必须和发布端一字不差
        self.subscription = self.create_subscription(
            BallPos, 'ball_pos', self.topic_callback, 10)

        self.get_logger().info('订阅者已启动，等待 /ball_pos ...')

    def topic_callback(self, msg):
        self.get_logger().info(
            f'收到: id={msg.id}, x={msg.x}, y={msg.y}')


def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
