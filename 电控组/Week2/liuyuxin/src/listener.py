#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_interfaces.msg import BallPos

class BallPosSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.subscription = self.create_subscription(
            BallPos,
            '/ball_pos',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'收到小球: id={msg.id}, x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = BallPosSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()