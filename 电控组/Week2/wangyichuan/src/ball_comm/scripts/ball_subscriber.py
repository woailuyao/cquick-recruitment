#!/usr/bin/env python3
"""任务 1 · 订阅者：订阅 /ball_pos，把收到的每个球的 id / x / y 打印出来。"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ball_comm.msg import BallPos


class BallSubscriber(Node):
    """收到一条 BallPos 就打印一条。"""

    def __init__(self):
        super().__init__('ball_subscriber')

        # 话题名必须和发布者一字不差：ball_pos
        self.subscription = self.create_subscription(
            BallPos, 'ball_pos', self.listener_callback, 10
        )
        self.get_logger().info('ball_subscriber 已启动，正在监听 /ball_pos ...')

    def listener_callback(self, msg):
        self.get_logger().info(
            '收到 BallPos: id=%d x=%.2f y=%.2f' % (msg.id, msg.x, msg.y)
        )


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
