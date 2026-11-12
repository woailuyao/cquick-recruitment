#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_comm.msg import BallPos

BALLS = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.2, self.tick)

    def tick(self):
        for bid, x, y in BALLS:
            m = BallPos()
            m.id = bid
            m.x = x
            m.y = y
            self.pub.publish(m)
            self.get_logger().info(f'send id={m.id} x={m.x} y={m.y}')


def main():
    rclpy.init()
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
