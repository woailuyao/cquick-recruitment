#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_ball_pkg.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.subscription = self.create_subscription(
            BallPos,
            'ball_pos',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'收到: ID={msg.id}, x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()