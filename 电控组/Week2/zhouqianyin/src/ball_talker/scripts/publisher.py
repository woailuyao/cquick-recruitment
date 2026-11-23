#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_talker.msg import BallPos

class BallPub(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.ball_data = [
            {"id":1, "x":2.0, "y":3.0},
            {"id":2, "x":5.5, "y":8.0},
            {"id":3, "x":9.0, "y":4.0},
        ]

    def timer_callback(self):
        # 一个周期内依次发送全部3个球
        for data in self.ball_data:
            msg = BallPos()
            msg.id = data["id"]
            msg.x = data["x"]
            msg.y = data["y"]
            self.pub.publish(msg)
            self.get_logger().info(f"发布 id:{msg.id}, x:{msg.x}, y:{msg.y}")

def main():
    rclpy.init()
    node = BallPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
