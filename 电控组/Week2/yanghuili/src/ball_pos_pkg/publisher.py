#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')

        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)

        # 每 0.5 秒触发一次，即 2 Hz
        self.timer = self.create_timer(0.5, self.publish_all_balls)

        self.balls = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0),
        ]

        self.get_logger().info('Ball publisher started: 2 Hz')

    def publish_all_balls(self):
        for ball_id, x, y in self.balls:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y

            self.publisher_.publish(msg)
            self.get_logger().info(
                f'Sent: id={msg.id}, x={msg.x:.1f}, y={msg.y:.1f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
