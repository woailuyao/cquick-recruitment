#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_client.py —— 手动发 goal 的 Action 客户端（考核 2.4）。

用法一（交互）：
    ros2 run cquick_servo move_client
    > 1 1700 1000          # id pwm duration_ms
    > 1 1300               # 不给 duration 就用节点默认值
    > q

用法二（一条命令演示正转/反转/回中，录屏用）：
    ros2 run cquick_servo move_client --ros-args -p auto_demo:=true
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo

DEMO_SEQUENCE = (1700, 1300, 1500)     # 正转 -> 反转 -> 回中位
DEMO_DURATION_MS = 1000


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')
        self.declare_parameter('auto_demo', False)
        self.declare_parameter('demo_id', 1)
        self._client = ActionClient(self, MoveServo, 'move_servo')

    def wait_for_server(self, timeout_sec=5.0) -> bool:
        if self._client.wait_for_server(timeout_sec=timeout_sec):
            return True
        self.get_logger().error('5 秒内没找到 /move_servo：驱动节点起了吗？')
        return False

    def _on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'feedback: current_pwm={feedback.current_pwm} progress={feedback.progress:.2f}')

    def send(self, servo_id: int, pwm: int, duration_ms: int) -> bool:
        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration_ms)

        # ① 异步发 goal，等「接不接受」
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝：看驱动节点日志里的 REJECT 原因')
            return False

        # ② 等 result；分片 spin，顺便给 Ctrl+C 一个处理机会
        result_future = goal_handle.get_result_async()
        try:
            while rclpy.ok() and not result_future.done():
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().warning('收到 Ctrl+C，取消当前 goal ...')
            goal_handle.cancel_goal_async()
            try:
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=2.0)
            except Exception as exc:        # 已经 shutdown / 取消后等不到 result
                self.get_logger().warning(f'取消后等 result 失败：{exc}')
                return False

        if not result_future.done():
            self.get_logger().error('等 result 超时')
            return False

        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error('没拿到 result')
            return False

        result = wrapped.result
        self.get_logger().info(
            f'result: success={result.success} final_pwm={result.final_pwm} '
            f'message={result.message}')
        return bool(result.success)


def main():
    rclpy.init()
    node = MoveClient()
    try:
        if not node.wait_for_server():
            return

        if bool(node.get_parameter('auto_demo').value):
            demo_id = int(node.get_parameter('demo_id').value)
            for pwm in DEMO_SEQUENCE:
                if not node.send(demo_id, pwm, DEMO_DURATION_MS):
                    break
            return

        while rclpy.ok():
            raw = input('输入 "id pwm [duration_ms]"（q 退出）> ').strip()
            if raw.lower() in ('q', 'quit', 'exit'):
                break
            parts = raw.split()
            if len(parts) not in (2, 3):
                print('格式：1 1700 1000  或  1 1700（用默认时长）')
                continue
            try:
                sid = int(parts[0])
                pwm = int(parts[1])
                duration = int(parts[2]) if len(parts) == 3 else 0
            except ValueError:
                print('id / pwm / duration 都必须是整数')
                continue
            node.send(sid, pwm, duration)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
