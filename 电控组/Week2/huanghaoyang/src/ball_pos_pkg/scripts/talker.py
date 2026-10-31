#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务一：把 3 个球的固定坐标循环发布到 /ball_pos。

相对原版的改动：
1. 发布频率改成 ROS 参数 rate_hz，运行中就能调，不用改代码重编译：
       ros2 run ball_pos_pkg talker.py --ros-args -p rate_hz:=2.0
       ros2 param set /ball_publisher rate_hz 10.0      # 跑着的时候现场改
   这正好是任务一加分项「发布频率可用 ros2 param 调整」；
2. 加了 batch 参数：默认 False = 每个定时周期发 1 个球（id 1/2/3 轮流）；
   设 True = 每个周期连发 3 条。两种都满足「3 个球持续发布」，
   差别只在于评审看代码时你说的那句「一个周期发几条」；
3. 退出时清理干净，Ctrl+C 不再打印 Traceback。

三个坐标是考核写死的值，不要改。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult

from ball_pos_pkg.msg import BallPos


# (id, x, y) —— 考核规定的三个球，单位米
BALLS = (
    (1, 2.0, 3.0),   # 左下（我方半场）
    (2, 5.5, 8.0),   # 中上（对方半场）
    (3, 9.0, 4.0),   # 右下（边路）
)


class BallPublisher(Node):
    RATE_MIN = 0.2
    RATE_MAX = 20.0

    def __init__(self):
        super().__init__('ball_publisher')

        # 参数：频率 + 是否每周期整批发
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('batch', False)

        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)

        self.index = 0
        self.timer = None
        self._apply_rate(float(self.get_parameter('rate_hz').value))

        # 运行中 ros2 param set 改了 rate_hz，立刻重建定时器
        self.add_on_set_parameters_callback(self._on_set_parameters)

        rate = float(self.get_parameter('rate_hz').value)
        batch = bool(self.get_parameter('batch').value)
        self.get_logger().info(
            f'发布者已启动：话题 /ball_pos，{rate:.1f} Hz，'
            f'{"每周期连发 3 条" if batch else "每周期发 1 条（1→2→3 轮流）"}')

    # ---------------- 参数处理 ----------------
    def _apply_rate(self, rate_hz):
        rate_hz = max(self.RATE_MIN, min(self.RATE_MAX, float(rate_hz)))
        if self.timer is not None:
            self.destroy_timer(self.timer)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        return rate_hz

    def _on_set_parameters(self, params):
        for param in params:
            if param.name == 'rate_hz':
                try:
                    value = float(param.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason='rate_hz 必须是数字')
                if not (self.RATE_MIN <= value <= self.RATE_MAX):
                    return SetParametersResult(
                        successful=False,
                        reason=f'rate_hz 必须在 {self.RATE_MIN}~{self.RATE_MAX} 之间')
                self._apply_rate(value)
                self.get_logger().info(f'发布频率已改为 {value:.1f} Hz')
        return SetParametersResult(successful=True)

    # ---------------- 定时器回调 ----------------
    def _make_msg(self, ball_id, x, y):
        msg = BallPos()
        msg.id = int(ball_id)
        msg.x = float(x)
        msg.y = float(y)
        return msg

    def _publish_one(self):
        ball_id, x, y = BALLS[self.index]
        msg = self._make_msg(ball_id, x, y)
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布: id={msg.id}, x={msg.x}, y={msg.y}')
        self.index = (self.index + 1) % len(BALLS)

    def timer_callback(self):
        if bool(self.get_parameter('batch').value):
            # 一个周期内把 3 个球依次发完
            for ball_id, x, y in BALLS:
                msg = self._make_msg(ball_id, x, y)
                self.publisher_.publish(msg)
                self.get_logger().info(f'发布: id={msg.id}, x={msg.x}, y={msg.y}')
        else:
            self._publish_one()


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
