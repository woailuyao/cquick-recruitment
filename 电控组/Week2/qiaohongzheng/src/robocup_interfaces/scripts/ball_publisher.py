#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robocup_interfaces.msg import BallPos


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')

        self.publisher_ = self.create_publisher(
            BallPos,
            'ball_pos',
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_balls
        )

    def publish_balls(self):
        balls = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0)
        ]

        for ball_id, x, y in balls:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y

            self.publisher_.publish(msg)

            self.get_logger().info(
                f'发布球 {ball_id}: x={x}, y={y}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = BallPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
