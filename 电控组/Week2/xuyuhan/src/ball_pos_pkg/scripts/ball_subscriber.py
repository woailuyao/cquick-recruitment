#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.balls = {}
        self.subscription = self.create_subscription(
            BallPos, 'ball_pos', self.on_msg, 10)
        self.get_logger().info('ball_subscriber 启动，等待 /ball_pos ...')

    def on_msg(self, msg):
        self.balls[msg.id] = (msg.x, msg.y)
        self.get_logger().info(
            f'收到 球{msg.id}: ({msg.x:.2f}, {msg.y:.2f})  '
            f'已收集 {len(self.balls)}/3')


def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()