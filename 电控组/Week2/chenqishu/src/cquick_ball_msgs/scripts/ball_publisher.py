#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务1 · 发布者：每个周期把 3 个球的坐标依次发到话题 /ball_pos。"""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from cquick_ball_msgs.msg import BallPos

# (id, x, y) —— 细则里写死的三个值，一个都不许改
BALLS = [
    (1, 2.0, 3.0),   # 左下（我方半场）
    (2, 5.5, 8.0),   # 中上（对方半场）
    (3, 9.0, 4.0),   # 右下（边路）
]


class BallPublisher(Node):

    def __init__(self):
        super().__init__('ball_publisher')

        # 频率做成 ROS 参数：不用改代码，ros2 param set 就能调（加分项 +3）
        self.declare_parameter('publish_rate', 2.0)
        rate = float(self.get_parameter('publish_rate').value)

        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_three_balls)
        self.round = 0

        self.add_on_set_parameters_callback(self.on_parameter_change)
        self.get_logger().info(
            f'发布者已启动：话题=/ball_pos，频率={rate:.2f} Hz，每周期连发 3 条')

    def on_parameter_change(self, params):
        """参数被 ros2 param set 改了就重建定时器，实现「不改代码改频率」。"""
        for p in params:
            if p.name == 'publish_rate':
                if float(p.value) <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='publish_rate 必须 > 0')
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(
                    1.0 / float(p.value), self.publish_three_balls)
                self.get_logger().info(
                    f'publish_rate 已改为 {float(p.value):.2f} Hz')
        return SetParametersResult(successful=True)

    def publish_three_balls(self):
        self.round += 1
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'[第 {self.round} 轮] 发布 -> id={msg.id} x={msg.x:.1f} y={msg.y:.1f}')


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
