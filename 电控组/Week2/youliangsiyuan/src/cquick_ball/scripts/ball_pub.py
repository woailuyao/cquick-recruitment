#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cquick_ball.msg import BallPos


BALLS = [
    (1, 2.0, 3.0),
    (2, 5.5, 8.0),
    (3, 9.0, 4.0),
]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_pub')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.freq = 2.0                                          # 2 Hz
        self.timer = self.create_timer(1.0 / self.freq, self.timer_cb)
        self.get_logger().info('ball_pub 启动，2 Hz')

    def timer_cb(self):
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = float(x)
            msg.y = float(y)
            self.pub.publish(msg)
            self.get_logger().info(f'发布: id={msg.id} ({msg.x:.1f}, {msg.y:.1f})')


def main():
    rclpy.init()
    node = BallPublisher()
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
