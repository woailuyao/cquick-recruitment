#!/usr/bin/env python3
"""取消演示：先把舵机摆到起点 → 发一条长动作 → 1.5 秒后取消（或随时按 Ctrl+C）。

预期结果：
    客户端  RESULT success=False message=canceled final_pwm≈1500
    驱动端  收到取消请求 / 取消：立即回中位 1500 并停止

用法：
    python3 ~/ros2_ws/src/cquick_servo/cquick_servo/cancel_demo.py [id] [pwm] [duration_ms] [start_pwm]

默认：id=1 目标=2050 时长=6000ms 起点=1200
先把舵机送到 1200，再往 2050 走，这样 feedback 会一路爬升，看得见它在动。

注意：`ros2 action send_goal` 命令行按 Ctrl+C 不会向服务端发取消请求，
      只是打断它自己的等待。验证 cancel 必须由客户端调用 cancel_goal_async()。
"""
import signal
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo

AUTO_CANCEL_SEC = 1.5
SIGINT = {'flag': False}


def _on_sigint(signum, frame):
    SIGINT['flag'] = True
    print('\n收到 Ctrl+C，准备取消 goal ...')


class CancelDemo(Node):
    def __init__(self):
        super().__init__('cancel_demo')
        self._client = ActionClient(self, MoveServo, 'move_servo')

    def _on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'feedback pwm={fb.current_pwm} progress={fb.progress:.2f}')

    def _send_and_wait(self, servo_id, pwm, duration_ms):
        """发一条 goal 并等它结束，返回 (accepted, result)"""
        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration_ms)

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, None

        result_future = handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return True, result_future.result().result

    def run(self, servo_id, pwm, duration_ms, start_pwm):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_servo 不可用（驱动节点起了吗？）')
            return False

        # ① 先摆到起点，让后面的长动作有可见的行程
        self.get_logger().info(f'先把舵机 1 号送到起点 {start_pwm}')
        ok, result = self._send_and_wait(servo_id, start_pwm, 900)
        if not ok:
            self.get_logger().error('起点 goal 被拒绝')
            return False
        self.get_logger().info(
            f'起点就位 success={result.success} final_pwm={result.final_pwm}')

        # ② 发长动作，中途取消
        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration_ms)

        self.get_logger().info(
            f'发送 goal id={goal.id} pwm={goal.pwm} dur={goal.duration_ms}')
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if handle is None or not handle.accepted:
            self.get_logger().error('goal 被拒绝')
            return False

        self.get_logger().info(
            f'goal accepted，{AUTO_CANCEL_SEC} 秒后自动取消（也可以直接按 Ctrl+C）')
        result_future = handle.get_result_async()

        cancelled = False
        deadline = time.time() + AUTO_CANCEL_SEC
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if not cancelled and (SIGINT['flag'] or time.time() >= deadline):
                self.get_logger().warning('发送取消请求 cancel_goal_async()')
                handle.cancel_goal_async()
                cancelled = True

        if not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)

        result = result_future.result().result
        self.get_logger().info(
            f'RESULT success={result.success} message={result.message} '
            f'final_pwm={result.final_pwm}')
        return not result.success


def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, _on_sigint)

    argv = [a for a in sys.argv[1:] if not a.startswith('-')]
    servo_id = int(argv[0]) if len(argv) > 0 else 1
    pwm = int(argv[1]) if len(argv) > 1 else 2050
    duration = int(argv[2]) if len(argv) > 2 else 6000
    start_pwm = int(argv[3]) if len(argv) > 3 else 1200

    node = CancelDemo()
    try:
        node.run(servo_id, pwm, duration, start_pwm)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
