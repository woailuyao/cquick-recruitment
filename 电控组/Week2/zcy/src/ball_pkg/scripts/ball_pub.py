#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

BALLS = {1: (2.0, 3.0), 2: (5.5, 8.0), 3: (9.0, 4.0)}

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.5, self.timer_callback) # 2Hz

    def timer_callback(self):
        for ball_id, (x, y) in BALLS.items():
            msg = BallPos()
            msg.id = ball_id
            msg.x = float(x)
            msg.y = float(y)
            self.pub.publish(msg)
            self.get_logger().info(f'发布 ball id={msg.id}, x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()