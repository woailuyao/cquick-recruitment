import rclpy
from rclpy.node import Node

from mine_interfaces.msg import TargetPosition


class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.count = 0
        self.create_subscription(TargetPosition, '/target_position', self.cb, 10)
        self.get_logger().info('已订阅 /target_position，等待消息…')

    def cb(self, msg: TargetPosition):
        self.count += 1
        if msg.detected:
            self.get_logger().info(f'#{self.count:02d} u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f}')
        else:
            self.get_logger().warn(f'#{self.count:02d} 未检测到目标')
        if self.count == 10:
            self.get_logger().info('已收到 10 条，满足要求')


def main():
    rclpy.init()
    node = Subscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()