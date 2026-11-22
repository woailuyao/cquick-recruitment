#!/usr/bin/env python3
"""MoveServo 交互式客户端。每行输入：<id> <pwm> <duration_ms>"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._last_id = 0

    def send(self, servo_id, pwm, duration_ms):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('move_servo 服务端未就绪')
            return False

        self._last_id = servo_id
        goal = MoveServo.Goal()
        goal.id = servo_id
        goal.pwm = pwm
        goal.duration_ms = duration_ms

        self.get_logger().info(
            f'发送 goal：id={servo_id} pwm={pwm} duration={duration_ms}')

        future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal 被拒绝')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result

        self.get_logger().info(
            f'结果：success={res.success} final_pwm={res.final_pwm} '
            f'message={res.message}')
        return res.success

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'id={self._last_id} pwm={fb.current_pwm} '
            f'progress={fb.progress:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    print('输入：<id> <pwm> <duration_ms>   例：0 1700 1000；回车退出')
    try:
        while rclpy.ok():
            try:
                line = input('> ').strip()
            except EOFError:
                break
            if not line:
                break
            parts = line.split()
            if len(parts) != 3:
                print('需要 3 个整数')
                continue
            try:
                sid, pwm, dur = (int(x) for x in parts)
            except ValueError:
                print('必须是整数')
                continue
            node.send(sid, pwm, dur)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
