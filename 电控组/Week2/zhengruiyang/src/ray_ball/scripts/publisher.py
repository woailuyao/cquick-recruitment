#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ray_ball.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.5, self.cb)   # 0.5s = 2Hz
        self.balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]

    def cb(self):
        for bid, x, y in self.balls:
            msg = BallPos()
            msg.id = bid
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发布球{bid}: x={x}, y={y}')

def main():
    rclpy.init()
    rclpy.spin(BallPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
