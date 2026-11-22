#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ballpos.msg import BallPos

class BallPosPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.2, self.timer_cb)
        self.ball_list = [
            {"id":1, "x":2.0, "y":3.0},
            {"id":2, "x":5.5, "y":8.0},
            {"id":3, "x":9.0, "y":4.0}
        ]
        self.idx = 0
    def timer_cb(self):
        msg = BallPos()
        b = self.ball_list[self.idx]
        msg.id = b["id"]
        msg.x = b["x"]
        msg.y = b["y"]
        self.pub.publish(msg)
        self.get_logger().info(f"发布 id={msg.id}, x={msg.x}, y={msg.y}")
        self.idx = (self.idx + 1) % len(self.ball_list)

def main():
    rclpy.init()
    node = BallPosPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

