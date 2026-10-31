#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.subscription = self.create_subscription(
            BallPos, 'ball_pos', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'接收: 第 {msg.id} 个球, 坐标 ({msg.x}, {msg.y})')

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()