#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')

        self.declare_parameter('servo_id', 1)
        self.declare_parameter('targets', [1200, 1800, 1500])
        self.declare_parameter('duration_ms', 1000)

        self.servo_id = int(self.get_parameter('servo_id').value)
        self.targets = [int(v) for v in self.get_parameter('targets').value]
        self.duration_ms = int(self.get_parameter('duration_ms').value)

        self._client = ActionClient(self, MoveServo, 'move_servo')
        self.get_logger().info(
            f'计划: id={self.servo_id} targets={self.targets} '
            f'duration={self.duration_ms}ms')

    def _feedback_cb(self, msg):
        feedback = msg.feedback
        self.get_logger().info(
            f'id={self.servo_id} pwm={feedback.current_pwm} '
            f'progress={feedback.progress:.2f}')

    def send_one(self, pwm):
        goal = MoveServo.Goal(
            id=self.servo_id, pwm=pwm, duration_ms=self.duration_ms)
        self.get_logger().info(
            f'--> goal id={self.servo_id} pwm={pwm} duration_ms={self.duration_ms}')

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('<-- goal 被拒绝（看服务端日志里的原因）')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()
        result = response.result

        if result.success:
            self.get_logger().info(
                f'<-- result SUCCESS final_pwm={result.final_pwm} {result.message}')
        else:
            self.get_logger().error(
                f'<-- result FAIL final_pwm={result.final_pwm} {result.message}')
        return result.success

    def run(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('10 秒内没等到 /move_servo 服务端')
            return False
        for pwm in self.targets:
            self.send_one(pwm)
        self.get_logger().info('全部目标执行完毕')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
