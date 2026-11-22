#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
任务1 · 发布者
每个周期把 3 个球的坐标依次发一遍，并在本地打印发出去的内容。
发布频率通过 ROS 参数 publish_rate 调整（默认 2 Hz），不用改代码：
    ros2 run cquick_ball ball_publisher --ros-args -p publish_rate:=5.0
"""
import rclpy
from rclpy.node import Node

from cquick_ball.msg import BallPos


# 写死的三个球（x 为场地 x，y 为场地 y，单位：米/乌龟坐标单位）
BALLS = [
    (1, 2.0, 3.0),
    (2, 5.5, 8.0),
    (3, 9.0, 4.0),
]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')

        # 话题名：ball_pos（订阅端必须一字不差）
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)

        # 频率做成参数，方便用 ros2 param 现场改（加分项）
        self.declare_parameter('publish_rate', 2.0)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.get_logger().info(
            f'发布频率 publish_rate = {self.publish_rate} Hz')

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        # 一个周期内把 3 个球依次发出去
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = float(x)
            msg.y = float(y)
            self.pub.publish(msg)
            self.get_logger().info(
                f'发布: id={msg.id}, x={msg.x:.2f}, y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
