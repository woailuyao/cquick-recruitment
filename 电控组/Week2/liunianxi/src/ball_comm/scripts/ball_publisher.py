#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务1 · 发布者：每周期把 3 个球（id=1/2/3）依次发一遍，默认 2 Hz。

频率做成了 ROS 参数，可以运行中改，不用动代码（加分项）：
    ros2 param set /ball_publisher freq 5.0
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from ball_comm.msg import BallPos

# 1.2 的固定坐标，一个都不改
BALLS = [
    (1, 2.0, 3.0),   # 左下（我方半场）
    (2, 5.5, 8.0),   # 中上（对方半场）
    (3, 9.0, 4.0),   # 右下（边路）
]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        # 参数：发布频率（Hz）。用固定 100 Hz 的定时器 + 自己做节拍，
        # 这样运行中改参数立刻生效，不用销毁/重建定时器（不会踩坑）。
        self.declare_parameter('freq', 2.0)
        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.freq = float(self.get_parameter('freq').value)
        self.last_pub = self.get_clock().now()
        self.add_on_set_parameters_callback(self.on_param_set)
        self.create_timer(0.01, self.tick)
        self.get_logger().info(
            '发布者已启动：话题 /ball_pos，频率 %.2f Hz，'
            '可用 ros2 param set /ball_publisher freq 5.0 改频率' % self.freq)

    def on_param_set(self, params):
        for p in params:
            if p.name == 'freq':
                if float(p.value) <= 0.0:
                    return SetParametersResult(successful=False, reason='freq 必须 > 0')
                self.freq = float(p.value)
                self.get_logger().info('频率已改为 %.2f Hz（每轮 %.3f s）' % (self.freq, 1.0 / self.freq))
        return SetParametersResult(successful=True)

    def tick(self):
        # 到点才发：每轮把 3 个球依次发一遍
        now = self.get_clock().now()
        if (now - self.last_pub).nanoseconds / 1e9 < 1.0 / self.freq:
            return
        self.last_pub = now
        for bid, x, y in BALLS:
            msg = BallPos()
            msg.id = bid
            msg.x = float(x)
            msg.y = float(y)
            self.pub.publish(msg)
            self.get_logger().info('发送 id=%d  x=%.1f  y=%.1f' % (bid, x, y))


def main():
    rclpy.init()
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
