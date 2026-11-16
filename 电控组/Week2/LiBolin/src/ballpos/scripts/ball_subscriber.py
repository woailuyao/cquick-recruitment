#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ballpos.msg import BallPos

class BallPosSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.cb,10)
    def cb(self, msg):
        self.get_logger().info(f"收到 id={msg.id}, x={msg.x}, y={msg.y}")

def main():
    rclpy.init()
    node = BallPosSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

