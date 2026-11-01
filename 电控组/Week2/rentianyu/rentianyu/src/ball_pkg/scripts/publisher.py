#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

BALLS = [
    (1, 2.0, 3.0),
    (2, 5.5, 8.0),
    (3, 9.0, 4.0),
]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.declare_parameter('rate_hz', 2.0)
        self.rate = self.get_parameter('rate_hz').value
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_cb)
        self.get_logger().info(
            f'发布器已启动：频率 {self.rate} Hz，话题 /ball_pos')

    def timer_cb(self):
        for bid, x, y in BALLS:
            msg = BallPos()
            msg.id = bid
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发布 -> id={msg.id}, x={msg.x}, y={msg.y}')


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
