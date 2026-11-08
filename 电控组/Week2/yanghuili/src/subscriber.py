#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')

        self.subscription_ = self.create_subscription(
            BallPos, 'ball_pos', self.listener_callback, 10)

        self.get_logger().info('Ball subscriber started, waiting for messages...')

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received: id={msg.id}, x={msg.x:.1f}, y={msg.y:.1f}'
        )


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
