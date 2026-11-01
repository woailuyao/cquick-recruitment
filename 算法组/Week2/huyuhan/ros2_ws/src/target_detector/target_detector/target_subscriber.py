from datetime import datetime
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__("target_subscriber")
        self.count = 0
        self.subscription = self.create_subscription(
            TargetPosition,
            "/target_position",
            self.receive_target,
            10
        )
        print("姓名：胡钰涵 / Hu Yuhan", flush=True)
        print(
            "运行时间："
            + datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y-%m-%d %H:%M:%S"),
            flush=True
        )
        self.get_logger().info("等待接收 /target_position 消息")

    def receive_target(self, msg):
        self.count += 1
        self.get_logger().info(
            f"收到第{self.count}条：detected={msg.detected}, "
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
