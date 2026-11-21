#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cquick_week2.msg import BallPos

BALLS = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.2, self.tick)   

    def tick(self):
        for bid, x, y in BALLS:
            msg = BallPos()
            msg.id = bid
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发 id={msg.id} x={msg.x} y={msg.y}')

def main():
    rclpy.init()
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()