#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 1 · 订阅端
订阅话题 /ball_pos（名字必须和发布端一字不差），收到就把 id / x / y 打印出来。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ball_pos_pkg.msg import BallPos

TOPIC_NAME = "ball_pos"
QUEUE_SIZE = 10


class BallPosSubscriber(Node):
    def __init__(self):
        super().__init__("ball_pos_subscriber")

        # 创建订阅者：消息类型、话题名、回调函数、队列长度
        self.subscription = self.create_subscription(
            BallPos, TOPIC_NAME, self.callback, QUEUE_SIZE
        )
        self.count = 0  # 收到第几条，方便看频率变化
        self.get_logger().info("订阅端启动：正在监听 /%s ..." % TOPIC_NAME)

    def callback(self, msg):
        # 每收到一条消息就进这里一次
        self.count += 1
        self.get_logger().info(
            "收到 #%d: id=%d  x=%.2f  y=%.2f" % (self.count, msg.id, msg.x, msg.y)
        )


def main(args=None):
    rclpy.init(args=args)
    node = BallPosSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # 避免信号路径下重复 shutdown
            rclpy.shutdown()


if __name__ == "__main__":
    main()
