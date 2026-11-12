#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from week2_ball.msg import BallPos


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        
        self.declare_parameter('freq', 2.0)
        freq = self.get_parameter('freq').value
        self.timer = self.create_timer(1.0 / freq, self.tick)
        self.balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]

    def tick(self):
        for ball_id, x, y in self.balls:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发布 id={msg.id} x={msg.x} y={msg.y}')


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