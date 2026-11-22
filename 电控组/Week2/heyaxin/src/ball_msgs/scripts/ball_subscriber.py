#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ball_msgs.msg import BallPos

import datetime


class BallSubscriber(Node):
    def __init__(self):
        super().__init__("ball_subscriber")
        self.sub = self.create_subscription(
            BallPos,
            "ball_pos",
            self.callback,
            10
        )

    def stamp(self):
        return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def callback(self, msg):
        self.get_logger().info(
            f"[heyaxin {self.stamp()}] 收到： id={msg.id}, x={msg.x}, y={msg.y}"
        )


def main():
    rclpy.init()
    node = BallSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
