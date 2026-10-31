#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__("ball_subscriber")
        self.subscription = self.create_subscription(
            BallPos,
            "/ball_pos",
            self.callback,
            10
        )

    def callback(self, msg: BallPos):
        self.get_logger().info(f"收到球{msg.id}坐标: x={msg.x:.1f}, y={msg.y:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
