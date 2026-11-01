"""订阅节点：接收 /target_position 并打印 detected / u / v / area。"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.count = 0
        self.subscription = self.create_subscription(
            TargetPosition, '/target_position', self._on_message, 10)
        self.get_logger().info('订阅节点已启动: 等待 /target_position ...')

    def _on_message(self, msg):
        self.count += 1
        if msg.detected:
            self.get_logger().info(
                f'[收到第 {self.count} 条] detected=True '
                f'u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f}')
        else:
            self.get_logger().info(
                f'[收到第 {self.count} 条] detected=False '
                f'u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f} '
                f'-> 本帧未检测到目标')


def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
