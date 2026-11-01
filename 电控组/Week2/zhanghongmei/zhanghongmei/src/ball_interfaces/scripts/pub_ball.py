#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_interfaces.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_pub_node')
        self.pub = self.create_publisher(BallPos, "/ball_pos", 10)
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.ball_list = [
            {"id":1, "x":2.0, "y":3.0},
            {"id":2, "x":5.5, "y":8.0},
            {"id":3, "x":9.0, "y":4.0},
        ]

    def timer_callback(self):
        for ball in self.ball_list:
            msg = BallPos()
            msg.id = ball["id"]
            msg.x = ball["x"]
            msg.y = ball["y"]
            self.pub.publish(msg)
            self.get_logger().info(f"发布球 id={msg.id}, x={msg.x}, y={msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
