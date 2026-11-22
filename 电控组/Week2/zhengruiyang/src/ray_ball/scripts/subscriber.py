#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ray_ball.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.create_subscription(BallPos, 'ball_pos', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f'收到球{msg.id}: x={msg.x}, y={msg.y}')

def main():
    rclpy.init()
    rclpy.spin(BallSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
