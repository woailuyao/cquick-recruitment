#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_comm.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f'recv id={msg.id} x={msg.x:.2f} y={msg.y:.2f}')


def main():
    rclpy.init()
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
