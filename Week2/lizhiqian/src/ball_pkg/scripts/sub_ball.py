#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallSubNode(Node):
    def __init__(self):
        super().__init__("ball_sub")
        self.sub = self.create_subscription(
            BallPos,
            "ball_pos",
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f"收到球 id={msg.id}, x={msg.x}, y={msg.y}")

def main():
    rclpy.init()
    node = BallSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()