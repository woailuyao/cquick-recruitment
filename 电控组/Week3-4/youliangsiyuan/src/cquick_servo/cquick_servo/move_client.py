#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action 客户端：从键盘手打一条指令，发 goal，打印 feedback 和 result。

用法（先起驱动节点，再起这个）：
    ros2 run cquick_servo move_client
    # 提示符下输入（两种写法都行）：
    #   #1,1700,1000
    #   1 1700 1000
    #   回车（空行）  -> 当前舵机回安全区间中点
    #   q + 回车    -> 退出

它只做三件事：拼 goal、发 goal、把回来说的话打出来。
真正的控制逻辑一行都不在这里 —— 这是 action 该有的样子。
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo
from cquick_servo.servo_protocol import ProtocolError, parse_command_shorthand

ACTION_NAME = 'move_servo'


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')
        self.declare_parameter('action_name', ACTION_NAME)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)

        self._action_name = str(self.get_parameter('action_name').value)
        self._client = ActionClient(self, MoveServo, self._action_name)
        self._last_id = 0        # 只用来在 feedback 打印里带出当前操作的舵机号

        self.get_logger().info(f'等待 action server /{self._action_name} ...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f'10 秒内没等到 /{self._action_name}。'
                f'驱动节点起了吗？ros2 action list 看一眼。')
        self.get_logger().info('action server 已就绪')

    # ---------------------------------------------------------------- 回调
    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        # 细则 2.4 要求的打印格式
        print(f'  [feedback] id={self._last_id} pwm={fb.current_pwm} '
              f'progress={fb.progress:.2f}', flush=True)

    # -------------------------------------------------------------- 核心动作
    def send(self, servo_id, pwm, duration_ms):
        """发一条 goal，等到 result，全程把 feedback 打出来。"""
        self._last_id = int(servo_id)

        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration_ms)

        print(f'-> goal: id={goal.id} pwm={goal.pwm} duration_ms={goal.duration_ms}',
              flush=True)

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            # 被拒是正常的（超限 / id 不存在），原因在驱动节点的日志里
            print('<- goal 被拒绝（看驱动节点那侧的 warn 日志：pwm 超限或 id 不在 servo_ids）',
                  flush=True)
            return False

        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            print('\n收到 Ctrl+C -> 取消当前 goal', flush=True)
            goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, result_future)
            raise

        wrapped = result_future.result()
        if wrapped is None:
            print('<- 没拿到 result（驱动节点可能挂了）', flush=True)
            return False

        result = wrapped.result
        flag = 'OK' if result.success else 'FAIL'
        print(f'<- result [{flag}] final_pwm={result.final_pwm} '
              f'retry={result.retry_count} message={result.message}', flush=True)
        return bool(result.success)

    # ---------------------------------------------------------------- 交互
    def interactive(self):
        pwm_min = int(self.get_parameter('pwm_min').value)
        pwm_max = int(self.get_parameter('pwm_max').value)
        mid = (pwm_min + pwm_max) // 2
        default_duration = int(self.get_parameter('default_duration_ms').value)

        print('''
输入格式（两种都行）：
  #1,1700,1000        舵机 1 -> PWM 1700，用 1000 ms
  1 1700 1000         同上
  回车                当前舵机回 {}（安全区间中点）
  q                   退出
'''.format(mid), flush=True)

        current_id = 1
        while rclpy.ok():
            try:
                line = input(f'[{current_id}] #id,pwm,duration> ').strip()
            except EOFError:
                break

            if line in ('q', 'quit', 'exit'):
                break
            if line == '':
                self.send(current_id, mid, default_duration)
                continue

            try:
                servo_id, pwm, duration_ms = parse_command_shorthand(line)
            except ProtocolError as exc:
                print(f'  解析失败：{exc}', flush=True)
                continue

            current_id = servo_id
            try:
                self.send(servo_id, pwm, duration_ms)
            except ProtocolError as exc:
                print(f'  指令非法：{exc}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MoveClient()
    except RuntimeError as exc:
        print(f'[ERROR] {exc}', file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        node.interactive()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
