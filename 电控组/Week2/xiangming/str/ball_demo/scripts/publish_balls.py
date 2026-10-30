#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_demo.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.balls = [
            {'id': 1, 'x': 2.0, 'y': 3.0},
            {'id': 2, 'x': 5.5, 'y': 8.0},
            {'id': 3, 'x': 9.0, 'y': 4.0},
        ]
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        for b in self.balls:
            msg = BallPos()
            msg.id = b['id']
            msg.x = b['x']
            msg.y = b['y']
            self.pub.publish(msg)
            self.get_logger().info(f'发布: 球{b["id"]} x={b["x"]} y={b["y"]}')

def main():
    rclpy.init()
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
