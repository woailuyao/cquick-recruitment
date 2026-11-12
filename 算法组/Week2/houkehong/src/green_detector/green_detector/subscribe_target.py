#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 3：订阅节点。订阅 /target_position，打印 detected / u / v / area。"""

from __future__ import annotations

from datetime import datetime

import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition

TOPIC = "/target_position"


class TargetSubscriber(Node):

    def __init__(self) -> None:
        super().__init__("target_subscriber")
        self.count = 0
        self.sub = self.create_subscription(TargetPosition, TOPIC, self.on_msg, 10)
        self.get_logger().info("订阅 %s，等待消息……" % TOPIC)

    def on_msg(self, msg: TargetPosition) -> None:
        self.count += 1
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if msg.detected:
            self.get_logger().info(
                "[#%02d %s] detected=%s u=%.2f v=%.2f area=%.2f"
                % (self.count, stamp, msg.detected, msg.u, msg.v, msg.area))
        else:
            # detected=false 时 u/v/area 是约定的默认值 0，不能当成“目标在左上角”
            self.get_logger().info(
                "[#%02d %s] detected=%s -> 未检测到目标（u=%.2f v=%.2f area=%.2f 为默认值）"
                % (self.count, stamp, msg.detected, msg.u, msg.v, msg.area))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("退出，共收到 %d 条消息" % node.count)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
