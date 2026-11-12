#!/usr/bin/env python3
"""任务 1 · 订阅者

订阅话题 /ball_pos，收到一个球就打印一次 id / x / y。
话题名必须和发布者一字不差，否则一条都收不到。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ball_tracker.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.declare_parameter('topic_name', 'ball_pos')
        topic = self.get_parameter('topic_name').value

        self.subscription = self.create_subscription(
            BallPos, topic, self.listener_callback, 10)
        self.get_logger().info(f'订阅者已启动：话题 /{topic}，等待球坐标...')

    def listener_callback(self, msg):
        self.get_logger().info(
            f'收到  球{msg.id}  x={msg.x:.2f}  y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
