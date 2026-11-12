import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__("target_subscriber")

        # 话题名和消息类型必须和发布端完全一致，差一个字就收不到
        self.sub = self.create_subscription(
            TargetPosition,          # 消息类型
            "/target_position",      # 话题名
            self.callback,           # 收到消息后调这个函数
            10,                      # 队列长度
        )

        self.get_logger().info("已订阅 /target_position，等待消息...")

    def callback(self, msg):
        # 发布端每发一条，这里就被调用一次
        if msg.detected:
            self.get_logger().info(
                f"收到: detected=True  u={msg.u:.1f}  v={msg.v:.1f}  area={msg.area:.0f}"
            )
        else:
            self.get_logger().warn("收到: detected=False（未检测到目标）")


def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    try:
        rclpy.spin(node)           # 一直挂着，等消息进来
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
