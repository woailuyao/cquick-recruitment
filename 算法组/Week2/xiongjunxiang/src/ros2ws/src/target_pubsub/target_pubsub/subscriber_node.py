import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.sub = self.create_subscription(
            TargetPosition,
            '/target_position',
            self.callback,
            10
        )

    def callback(self, msg: TargetPosition):
        # 只以 detected 判断有效性，不根据 u/v 是否为 0 推断
        self.get_logger().info(
            f'Received: detected={msg.detected}, '
            f'u={msg.u:.2f}, v={msg.v:.2f}, area={msg.area:.2f}'
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()