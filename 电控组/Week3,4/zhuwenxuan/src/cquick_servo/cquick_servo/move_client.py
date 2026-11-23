#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cquick_servo Action 客户端（第 3/4 周 · 任务 2.4）

用法：
    ros2 run cquick_servo move_client --id 1 --pwm 1700
    ros2 run cquick_servo move_client --id 1 --pwm 1700 --duration 1200
    ros2 run cquick_servo move_client --id 1 --sweep        # 正转 → 反转 → 回中位
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, '/move_servo')

    def send_one(self, sid, pwm, duration):
        goal = MoveServo.Goal()
        goal.id = int(sid)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration)

        self.get_logger().info('send goal: id=%d pwm=%d duration=%d' % (sid, pwm, duration))
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal REJECTED: check servo_ids and pwm range')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error('no result received')
            return False
        result = wrapped.result
        self.get_logger().info('result: success=%s final_pwm=%d message=%s'
                               % (result.success, result.final_pwm, result.message))
        return bool(result.success)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info('feedback: pwm=%d progress=%.2f'
                               % (fb.current_pwm, fb.progress))


def main(args=None):
    parser = argparse.ArgumentParser(description='MoveServo 客户端')
    parser.add_argument('--id', type=int, default=1, help='舵机 ID')
    parser.add_argument('--pwm', type=int, default=1600, help='目标 PWM')
    parser.add_argument('--duration', type=int, default=800, help='用时 ms')
    parser.add_argument('--sweep', action='store_true', help='演示：正转 → 反转 → 回中位')
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = MoveClient()
    try:
        if not node._client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('/move_servo not available, is servo_driver running?')
            return 1

        if parsed.sweep:
            # 正转转出去、反转转回来、回中位 —— 正好是完成标准里那三项
            for pwm in (parsed.pwm, 3000 - parsed.pwm, (parsed.pwm + (3000 - parsed.pwm)) // 2):
                if not node.send_one(parsed.id, pwm, parsed.duration):
                    return 1
            return 0

        return 0 if node.send_one(parsed.id, parsed.pwm, parsed.duration) else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
