#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Action 客户端：发一条 MoveServo goal，打印 feedback 与 result。

用法：
    ros2 run cquick_servo move_client --id 1 --pwm 1700 --duration 1000
    ros2 run cquick_servo move_client --demo        # 正转 → 反转 → 回中位
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')
        self.client = ActionClient(self, MoveServo, 'move_servo')

    def send_one(self, sid, pwm, duration_ms, cancel_after=0.0):
        self.get_logger().info('等待 action server ……')
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('没有 action server（先 launch servo.launch.py）')
            return False

        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = duration_ms

        self.get_logger().info('发送 goal: id=%d pwm=%d duration=%d ms' % (sid, pwm, duration_ms))
        send_future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝（pwm 越界 / id 不在 servo_ids / duration 非法）')
            return False

        # 可选：运行中主动取消（演示 cancel 用）
        if cancel_after > 0.0:
            import time as _t
            self.get_logger().info('%.1f 秒后主动取消这个 goal ……' % cancel_after)
            t0 = _t.monotonic()
            while _t.monotonic() - t0 < cancel_after:
                rclpy.spin_once(self, timeout_sec=0.05)
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().warn('已发送取消请求')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            '结果: success=%s final_pwm=%d message=%s' % (
                result.success, result.final_pwm, result.message))
        return bool(result.success)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            'id=%d pwm=%d progress=%.2f' % (0, fb.current_pwm, fb.progress))


def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv[1:]
    sid, pwm, duration = 1, 1700, 1000
    demo = '--demo' in argv
    cancel_after = 0.0

    for i, a in enumerate(argv):
        if a == '--id' and i + 1 < len(argv):
            sid = int(argv[i + 1])
        elif a == '--pwm' and i + 1 < len(argv):
            pwm = int(argv[i + 1])
        elif a == '--duration' and i + 1 < len(argv):
            duration = int(argv[i + 1])
        elif a == '--cancel-after' and i + 1 < len(argv):
            cancel_after = float(argv[i + 1])

    node = MoveClient()
    try:
        if demo:
            for target, ms in ((1700, 1000), (1300, 1000), (1500, 800)):
                node.send_one(sid, target, ms)
        else:
            node.send_one(sid, pwm, duration, cancel_after)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
