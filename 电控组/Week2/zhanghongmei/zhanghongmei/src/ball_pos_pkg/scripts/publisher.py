#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos
class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    def timer_callback(self):
        balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        for b_id, x, y in balls:
            msg = BallPos()
            msg.id, msg.x, msg.y = b_id, x, y
            self.pub.publish(msg)
            self.get_logger().info(f'发布球: id={msg.id}, x={msg.x}, y={msg.y}')
def main():
    rclpy.init()
    rclpy.spin(BallPublisher())
if __name__ == '__main__':
    main()
