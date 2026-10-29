#!/usr/bin/env python3
"""订阅节点：订阅 /target_position，打印 detected / u / v / area。"""
import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition

TOPIC = "/target_position"


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__("target_subscriber")
        self.count = 0
        self.sub = self.create_subscription(TargetPosition, TOPIC, self.callback, 10)
        self.get_logger().info(f"订阅节点启动：等待 {TOPIC} ...")

    def callback(self, msg):
        self.count += 1
        if msg.detected:
            self.get_logger().info(
                f"[{self.count:02d}] detected=True   u={msg.u:.1f}  v={msg.v:.1f}  area={msg.area:.0f}"
            )
        else:
            # 判断有效性只看 detected，不看 u 是不是 0
            self.get_logger().info(
                f"[{self.count:02d}] detected=False  未检测到目标  (u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    try:
        rclpy.spin(node)          # 订阅端不 spin 就永远收不到消息
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
