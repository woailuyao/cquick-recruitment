#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        
        self.declare_parameter('freq', 2.0)
        freq = self.get_parameter('freq').value
        self.create_timer(1.0 / freq, self.timer_callback)

    def timer_callback(self):
        balls = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0),
        ]
        for bid, x, y in balls:
            msg = BallPos()
            msg.id = bid
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发送: id={bid}, x={x}, y={y}')

def main():
    rclpy.init()
    node = BallPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
