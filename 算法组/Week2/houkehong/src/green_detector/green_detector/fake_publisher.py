#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 3 第 5 步：固定数据的假发布节点。

**只用于先验证“话题通不通”**，最终验收必须用 publish_target（真实检测结果）。
它把一小组写死的 (u, v, area) 循环发出去，其中还故意夹一条 detected=false，
用来验证“没检测到目标”这种帧在消息里是怎么表达的。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition

TOPIC = "/target_position"

# (detected, u, v, area) —— 约定：detected=false 时三个数一律为 0
FAKE_SAMPLES = [
    (True, 320.0, 240.0, 4800.0),
    (True, 400.0, 260.0, 5200.0),
    (True, 480.0, 280.0, 6100.0),
    (True, 512.0, 300.0, 7000.0),
    (True, 560.0, 320.0, 7400.0),
    (False, 0.0, 0.0, 0.0),
    (True, 600.0, 340.0, 7900.0),
    (True, 620.0, 350.0, 8100.0),
    (True, 640.0, 360.0, 8300.0),
    (True, 660.0, 370.0, 8500.0),
]


class FakePublisher(Node):

    def __init__(self) -> None:
        super().__init__("fake_target_publisher")
        self.declare_parameter("rate_hz", 2.0)
        self.pub = self.create_publisher(TargetPosition, TOPIC, 10)
        rate = max(float(self.get_parameter("rate_hz").value), 0.5)
        self.i = 0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().warn(
            "固定坐标测试模式，仅用于验证通信；正式验收请用 publish_target 节点")

    def on_timer(self) -> None:
        detected, u, v, area = FAKE_SAMPLES[self.i % len(FAKE_SAMPLES)]
        self.i += 1
        msg = TargetPosition()
        msg.detected, msg.u, msg.v, msg.area = detected, u, v, area
        self.pub.publish(msg)
        self.get_logger().info(
            "[#%02d] 发布 fixed: detected=%s u=%.1f v=%.1f area=%.1f"
            % (self.i, detected, u, v, area))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
