#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.2, self.send)

    def send(self):
        balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        for bid, x, y in balls:
            msg = BallPos()
            msg.id = bid
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发了球{bid}: ({x}, {y})')


def main():
    rclpy.init()
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

