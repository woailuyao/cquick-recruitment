#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_publisher.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_sub_node')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'收到 id={msg.id} x={msg.x} y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()