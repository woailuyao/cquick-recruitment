import rclpy
from rclpy.node import Node

from cquick_vision_msgs.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("target_position_subscriber")

        self.subscription = self.create_subscription(
            TargetPosition,
            "/target_position",
            self.on_message,
            10,
        )

        self.received_count = 0

        self.get_logger().info(
            "目标位置订阅节点已启动，等待 /target_position"
        )

    def on_message(self, msg: TargetPosition) -> None:
        self.received_count += 1

        print(
            f"第 {self.received_count} 条: "
            f"detected={msg.detected}, "
            f"u={msg.u:.2f}, "
            f"v={msg.v:.2f}, "
            f"area={msg.area:.2f}",
            flush=True,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
