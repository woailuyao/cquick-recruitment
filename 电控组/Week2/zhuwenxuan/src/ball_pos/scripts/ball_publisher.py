#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 1 · 发布端：按固定频率循环发布 3 个球的坐标

用法：
    ros2 run ball_pos ball_publisher
    ros2 run ball_pos ball_publisher --ros-args -p freq:=5.0
    ros2 param set /ball_publisher freq 5.0     # 运行中动态改频率（加分项）
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from ball_pos.msg import BallPos

# 三个球的固定坐标（细则规定，不可修改）
BALLS = [
    (1, 2.0, 3.0),   # 左下（我方半场）
    (2, 5.5, 8.0),   # 中上（对方半场）
    (3, 9.0, 4.0),   # 右下（边路）
]


class BallPublisher(Node):

    def __init__(self):
        super().__init__('ball_publisher')

        # 频率做成参数，这样不用改代码就能调（加分项）
        self.declare_parameter('freq', 2.0)
        self.freq = float(self.get_parameter('freq').value)

        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)
        self.add_on_set_parameters_callback(self.param_callback)

        self.get_logger().info(
            'ball_publisher 已启动：话题=ball_pos，频率=%.1f Hz' % self.freq)
        self.get_logger().info(
            '提示：ros2 param set /ball_publisher freq 5.0 可以动态改频率')

    def param_callback(self, params):
        for p in params:
            if p.name == 'freq':
                if p.value <= 0.0:
                    return SetParametersResult(successful=False, reason='freq 必须大于 0')
                self.freq = float(p.value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)
                self.get_logger().info('>>> 频率已切换为 %.1f Hz <<<' % self.freq)
        return SetParametersResult(successful=True)

    def timer_callback(self):
        self.get_logger().info('-------- 发布一轮（3 个球） --------')
        for bid, x, y in BALLS:
            msg = BallPos()
            msg.id = int(bid)
            msg.x = float(x)
            msg.y = float(y)
            self.publisher_.publish(msg)
            self.get_logger().info(
                '发布 BallPos: id=%d, x=%.2f, y=%.2f' % (msg.id, msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
