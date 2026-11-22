#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2.4 Action 客户端：发 goal，打印 feedback 和 result。

用法：
    ros2 run cquick_servo move_client 1 1700 1000   # id pwm duration
    ros2 run cquick_servo move_client 1 1500        # duration 省略 = 驱动默认值
    ros2 run cquick_servo move_client               # 交互模式，一行一条
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._goal_handle = None

    def wait_for_server(self, timeout=10.0):
        return self._client.wait_for_server(timeout_sec=timeout)

    # ------------------------------------------------------------------ 回调
    def _on_feedback(self, msg):
        fb = msg.feedback
        # 格式按考核文档：id=1 pwm=1620 progress=0.62
        self.get_logger().info(
            f'id={fb.servo_id} pwm={fb.current_pwm} progress={fb.progress:.2f}'
        )

    # ------------------------------------------------------------------ 发一条
    def send(self, sid, pwm, duration=0):
        goal = MoveServo.Goal()
        goal.command = ServoCommand(id=sid, pwm=pwm, duration_ms=duration)

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if handle is None:
            self.get_logger().error('发送 goal 失败（没有响应）')
            return False
        if not handle.accepted:
            self.get_logger().warn(
                f'goal rejected: id={sid} pwm={pwm}（pwm 超限 / ID 不在 servo_ids）'
            )
            return False

        self._goal_handle = handle
        self.get_logger().info(f'goal accepted: id={sid} pwm={pwm} duration={duration}ms')
        result_future = handle.get_result_async()

        try:
            while not result_future.done():
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().warn('Ctrl+C → 取消 goal')
            handle.cancel_goal_async()
            while not result_future.done():
                try:
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
                except KeyboardInterrupt:
                    break

        if not result_future.done():
            self.get_logger().warn('没等到 result')
            return False

        wrapped = result_future.result()
        result = wrapped.result
        status = STATUS.get(wrapped.status, str(wrapped.status))
        self.get_logger().info(
            f'result status={status} success={result.success} '
            f'actual_pwm={result.actual_pwm} message={result.message}'
        )
        return bool(result.success)


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    argv = remove_ros_args(args=sys.argv if args is None else args)[1:]

    try:
        if not node.wait_for_server(10.0):
            node.get_logger().error('move_servo 未就绪：先启动 servo_driver')
            return 1

        # 命令行模式：move_client <id> <pwm> [duration_ms]
        if len(argv) >= 2:
            try:
                sid, pwm = int(argv[0]), int(argv[1])
                duration = int(argv[2]) if len(argv) >= 3 else 0
            except ValueError:
                print('用法: move_client <id> <pwm> [duration_ms]', file=sys.stderr)
                return 2
            return 0 if node.send(sid, pwm, duration) else 3

        # 交互模式
        print('输入 "id pwm [duration_ms]" 回车下发；q 退出；Ctrl+C 取消当前动作')
        while True:
            try:
                line = input('> ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if line.lower() in ('q', 'quit', 'exit'):
                return 0
            parts = line.replace(',', ' ').split()
            if len(parts) not in (2, 3):
                print('格式: id pwm [duration_ms]')
                continue
            try:
                sid, pwm = int(parts[0]), int(parts[1])
                duration = int(parts[2]) if len(parts) == 3 else 0
            except ValueError:
                print('三个都必须是整数')
                continue
            node.send(sid, pwm, duration)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
