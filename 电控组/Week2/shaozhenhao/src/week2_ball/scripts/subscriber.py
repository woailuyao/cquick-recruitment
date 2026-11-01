#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from week2_ball.msg import BallPos


class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.on_msg, 10)

    def on_msg(self, msg):
        self.get_logger().info(f'收到 id={msg.id} x={msg.x} y={msg.y}')


def main():
    rclpy.init()
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