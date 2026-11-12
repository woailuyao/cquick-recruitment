#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_interfaces.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__("ball_sub_node")
        self.sub = self.create_subscription(
            BallPos,
            "/ball_pos",
            self.callback_func,
            10
        )

    def callback_func(self, msg):
        self.get_logger().info(f"收到小球：id={msg.id}, x={msg.x}, y={msg.y}")

def main(args=None):
    rclpy.init(args=None)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

