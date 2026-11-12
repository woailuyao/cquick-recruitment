import rclpy
from rclpy.node import Node

from cquick_interfaces.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__("target_subscriber")
        self.subscription = self.create_subscription(
            TargetPosition,
            "/target_position",
            self.callback,
            10,
        )
        self.count = 0

    def callback(self, msg):
        self.count += 1
        self.get_logger().info(
            f"recv {self.count}: detected={msg.detected}, "
            f"u={msg.u:.1f}, v={msg.v:.1f}, area={msg.area:.1f}"
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


if __name__ == "__main__":
    main()