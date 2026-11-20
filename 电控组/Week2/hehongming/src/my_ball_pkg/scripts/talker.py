#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_ball_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        for ball in balls:
            msg = BallPos()
            msg.id = ball[0]
            msg.x = ball[1]
            msg.y = ball[2]
            self.publisher_.publish(msg)
            self.get_logger().info(f'发送: ID={msg.id}, x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()