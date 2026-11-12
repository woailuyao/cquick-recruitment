#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.sub = self.create_subscription(
            BallPos,
            '/ball_pos',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f"订阅收到：id={msg.id}, x={msg.x}, y={msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
