#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_interfaces.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.sub = self.create_subscription(
            BallPos, 'ball_pos', self.callback, 10)
        self.get_logger().info('Ball subscriber started')

    def callback(self, msg):
        self.get_logger().info(
            f'Received: id={msg.id}, x={msg.x:.2f}, y={msg.y:.2f}')


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