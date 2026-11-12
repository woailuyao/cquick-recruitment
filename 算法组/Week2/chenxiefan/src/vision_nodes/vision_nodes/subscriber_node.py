#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务 3 · 订阅节点

节点名  : target_subscriber
话  题  : /target_position
消息类型: vision_interfaces/msg/TargetPosition
功  能  : 订阅发布节点发出的目标位置，打印 detected / u / v / area
"""

import rclpy
from rclpy.node import Node

from vision_interfaces.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')

        self.subscription = self.create_subscription(
            TargetPosition,
            '/target_position',
            self.callback,
            10,
        )
        self.count = 0
        self.get_logger().info('已订阅 /target_position，等待消息...')

    def callback(self, msg):
        self.count += 1
        if msg.detected:
            self.get_logger().info(
                f'第 {self.count} 条  detected=True   u={msg.u:.1f}  v={msg.v:.1f}  area={msg.area:.1f}'
            )
        else:
            self.get_logger().info(
                f'第 {self.count} 条  detected=False  u={msg.u:.1f}  v={msg.v:.1f}  '
                f'area={msg.area:.1f}（未识别到目标）'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
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
