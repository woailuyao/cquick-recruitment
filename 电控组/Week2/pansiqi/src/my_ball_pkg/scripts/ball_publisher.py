#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_ball_pkg.msg import BallPos


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 0.2s = 5Hz（最初是 0.5s = 2Hz，改频率后截的图）

    def timer_callback(self):
        balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        for ball_id, x, y in balls:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发球 {msg.id}: ({msg.x}, {msg.y})')


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()