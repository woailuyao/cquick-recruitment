#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_comm.msg import BallPos


class BallSub(Node):
    def __init__(self):
        super().__init__('ball_sub')
        self.sub = self.create_subscription(
            BallPos, 'ball_pos', self.cb, 10)
        self.get_logger().info('订阅者启动，等待 /ball_pos')

    def cb(self, msg):
        self.get_logger().info(
            f'收到球 {msg.id}: x={msg.x:.2f}, y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = BallSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
