#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_msg_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher_node')
        # 创建发布者，话题名 /ball_pos
        self.pub = self.create_publisher(BallPos, '/ball_pos', 10)
        # 定时器 2Hz → 0.5s触发一次
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # 固定球坐标，题目要求，不许修改
        self.ball_list = [
            {"id":1, "x":2.0, "y":3.0},
            {"id":2, "x":5.5, "y":8.0},
            {"id":3, "x":9.0, "y":4.0}
        ]

    def timer_callback(self):
        # 一轮依次发送3个球
        for ball in self.ball_list:
            msg = BallPos()
            msg.id = ball["id"]
            msg.x = ball["x"]
            msg.y = ball["y"]
            self.pub.publish(msg)
            self.get_logger().info(f"发布: id={msg.id}, x={msg.x}, y={msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

