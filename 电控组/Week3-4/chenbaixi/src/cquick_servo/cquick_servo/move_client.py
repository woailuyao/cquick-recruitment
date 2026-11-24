#!/usr/bin/env python3
"""Action 客户端：从命令行读 id/pwm/duration，发 goal 并打印反馈与结果"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, 'move_servo')

    def send(self, servo_id, pwm, duration_ms) -> bool:
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action Server 没起来')
            return False

        goal = MoveServo.Goal()
        goal.id, goal.pwm, goal.duration_ms = servo_id, pwm, duration_ms
        self.get_logger().info(f'发送 goal: id={servo_id} pwm={pwm} dur={duration_ms}')

        future = self._client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal 被拒绝')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        self.get_logger().info(
            f'结果: success={res.success} final_pwm={res.final_pwm} message="{res.message}"')
        return res.success

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'反馈: current_pwm={fb.current_pwm} progress={fb.progress:.2f}')


def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv[1:]
    if len(argv) < 3:
        print('用法: ros2 run cquick_servo move_client <id> <pwm> <duration_ms>')
        return
    node = MoveClient()
    try:
        node.send(int(argv[0]), int(argv[1]), int(argv[2]))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
