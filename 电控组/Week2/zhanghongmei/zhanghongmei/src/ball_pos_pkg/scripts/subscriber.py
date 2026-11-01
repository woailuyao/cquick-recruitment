#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos
class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.listener_callback, 10)
    def listener_callback(self, msg):
        self.get_logger().info(f'收到球: id={msg.id}, x={msg.x}, y={msg.y}')
def main():
    rclpy.init()
    rclpy.spin(BallSubscriber())
if __name__ == '__main__':
    main()
