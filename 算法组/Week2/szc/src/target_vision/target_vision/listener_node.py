# -*- coding: utf-8 -*-
"""订阅节点：订阅 /target_position，打印 detected / u / v / area。

对应任务 3 的订阅端要求。
"""
import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


class ListenerNode(Node):

    def __init__(self):
        super().__init__('target_listener')
        self.count = 0
        self.create_subscription(
            TargetPosition, '/target_position', self.callback, 10)
        self.get_logger().info('订阅节点已启动，等待 /target_position ...')

    def callback(self, msg):
        self.count += 1
        if msg.detected:
            self.get_logger().info(
                '收到第 %d 条 | detected=%s u=%.1f v=%.1f area=%.0f'
                % (self.count, msg.detected, msg.u, msg.v, msg.area))
        else:
            # 明确区分「没检测到」和「目标在左上角」
            self.get_logger().info(
                '收到第 %d 条 | detected=%s u=%.1f v=%.1f area=%.0f  <-- 本帧无目标，坐标无效'
                % (self.count, msg.detected, msg.u, msg.v, msg.area))


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


if __name__ == '__main__':
    main()
