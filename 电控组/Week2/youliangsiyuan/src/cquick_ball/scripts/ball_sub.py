#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务1 订阅者：订阅 ball_pos，打印 id/x/y。"""
import rclpy
from rclpy.node import Node
from cquick_ball.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_sub')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.cb, 10)
        self.get_logger().info('ball_sub 启动，等待 ball_pos ...')

    def cb(self, msg):
        self.get_logger().info(f'收到: id={msg.id} x={msg.x:.1f} y={msg.y:.1f}')


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