#!/usr/bin/env python3
"""任务 1 - 订阅者：收到 BallPos 就打印"""

import rclpy
from rclpy.node import Node
from ball_week2.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.cb, 10)
        self.get_logger().info('订阅者已启动，等待 ball_pos……')

    def cb(self, msg):
        self.get_logger().info(f'收到 ← id={msg.id}  坐标=({msg.x:.2f}, {msg.y:.2f})')


def main():
    rclpy.init()
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
