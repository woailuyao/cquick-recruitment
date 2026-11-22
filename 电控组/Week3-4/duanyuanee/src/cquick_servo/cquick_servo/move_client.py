#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from cquick_servo_interfaces.action import MoveServo
import sys


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, '/servo/move')
        self.get_logger().info('MoveClient 启动，等待 Action Server...')
        self._client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info('Action Server 已连接!')

    def send_goal(self, servo_id, pwm, duration_ms):
        goal = MoveServo.Goal()
        goal.command.id = servo_id
        goal.command.pwm = pwm
        goal.command.duration_ms = duration_ms

        self.get_logger().info(
            f'发送 Goal: ID={servo_id}, PWM={pwm}, Duration={duration_ms}ms')

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 被拒绝!')
            return
        self.get_logger().info('Goal 被接受!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(
            f'反馈: 进度 {fb.progress:.1%}, 当前PWM={fb.current_pwm}')

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f'结果: success={result.success}, '
            f'final_pwm={result.final_pwm}, '
            f'message={result.message}')
        rclpy.shutdown()


def main():
    rclpy.init()
    client = MoveClient()

    # ━━ 从命令行参数读，默认 ID=1, PWM=1500, Duration=1000 ━━
    servo_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    pwm = int(sys.argv[2]) if len(sys.argv) > 2 else 1500
    duration = int(sys.argv[3]) if len(sys.argv) > 3 else 1000

    client.send_goal(servo_id, pwm, duration)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
