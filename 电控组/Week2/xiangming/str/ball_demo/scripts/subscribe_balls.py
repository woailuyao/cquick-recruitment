#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_demo.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'收到: 球{msg.id} x={msg.x} y={msg.y}')

def main():
    rclpy.init()
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
