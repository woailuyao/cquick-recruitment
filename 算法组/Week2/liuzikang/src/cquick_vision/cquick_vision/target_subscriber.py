#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订阅节点：订阅 /target_position，打印收到的每一条消息，并统计够不够 10 条。

为什么要有它
------------
    发布节点只证明"我发出去了"，订阅节点才证明"对方真的收到了、
    而且字段能按消息定义解析出来"。两个都跑起来才算通信跑通。
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from cquick_interfaces.msg import TargetPosition

TOPIC_NAME = "/target_position"


class TargetSubscriber(Node):

    def __init__(self) -> None:
        super().__init__("target_subscriber")
        self.declare_parameter("expected_count", 10)

        self.count = 0
        self.detected_count = 0
        self.miss_count = 0
        self.reported = False

        self.subscription = self.create_subscription(
            TargetPosition, TOPIC_NAME, self.on_message, 10)

        expected = int(self.get_parameter("expected_count").value)
        self.get_logger().info(
            f"已订阅 {TOPIC_NAME}（类型 cquick_interfaces/msg/TargetPosition），"
            f"目标：至少收到 {expected} 条")

    def on_message(self, msg: TargetPosition) -> None:
        self.count += 1
        if msg.detected:
            self.detected_count += 1
            self.get_logger().info(
                f"#{self.count:03d}  detected=True   u={msg.u:8.2f}  v={msg.v:8.2f}  "
                f"area={msg.area:10.1f}")
        else:
            self.miss_count += 1
            self.get_logger().info(
                f"#{self.count:03d}  detected=False  u={msg.u:8.2f}  v={msg.v:8.2f}  "
                f"area={msg.area:10.1f}   <- 本帧没有目标，数值无效")

        # 常见错误：detected=False 却带着非 0 坐标，检查一下
        if not msg.detected and (msg.u != 0.0 or msg.v != 0.0 or msg.area != 0.0):
            self.get_logger().warning(
                "检测到异常：detected=False 但 u/v/area 不是 0，上层可能把 0 当成坐标用错！")

        expected = int(self.get_parameter("expected_count").value)
        if self.count == expected and not self.reported:
            self.reported = True
            self.get_logger().info(
                f"==== 已收到 {self.count} 条（检测到 {self.detected_count} 条，"
                f"未检测到 {self.miss_count} 条）—— 通信验证通过 ====")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
