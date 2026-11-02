# -*- coding: utf-8 -*-
"""订阅节点：收 /target_position 并把 detected / u / v / area 打印出来。"""

import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


class ListenerNode(Node):
    def __init__(self):
        super().__init__("target_listener")
        self.count = 0
        self.subscription = self.create_subscription(
            TargetPosition, "/target_position", self.on_msg, 10)
        self.get_logger().info("等待 %s 上的消息 ..." % self.subscription.topic_name)

    def on_msg(self, msg):
        self.count += 1
        if msg.detected:
            self.get_logger().info(
                "#%d detected=%s u=%.1f v=%.1f area=%.0f"
                % (self.count, msg.detected, msg.u, msg.v, msg.area))
        else:
            # 没检测到的时候 u/v/area 是 0，只是占位，不能当靶心坐标看
            self.get_logger().info(
                "#%d detected=%s (本帧没有目标, u/v/area 均为 0)"
                % (self.count, msg.detected))


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
