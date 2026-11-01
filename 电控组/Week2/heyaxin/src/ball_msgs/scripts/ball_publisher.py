#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ball_msgs.msg import BallPos

import datetime


class BallPublisher(Node):
    def __init__(self):
        super().__init__("ball_publisher")
        self.pub = self.create_publisher(BallPos, "ball_pos", 10)
        timer_period = 0.5  # 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ball_list = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0),
        ]

    def stamp(self):
        # 每条日志都带上「姓名 + 实时时间戳」，截图天然满足防抄袭要求
        return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def timer_callback(self):
        # 每个周期把 3 个球依次发一遍（细则 1.3 第 5 步的要求）
        for ball_id, x, y in self.ball_list:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y

            self.pub.publish(msg)
            self.get_logger().info(
                f"[heyaxin {self.stamp()}] 发布： id={msg.id}, x={msg.x}, y={msg.y}"
            )


def main():
    rclpy.init()
    node = BallPublisher()
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
