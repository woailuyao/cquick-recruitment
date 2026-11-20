#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo
import sys

class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._action_client = ActionClient(self, MoveServo, 'move_servo')

    def send_goal(self, servo_id, pwm, duration_ms):
        self.get_logger().info('等待 Action Server...')
        self._action_client.wait_for_server()

        goal_msg = MoveServo.Goal()
        goal_msg.id = servo_id
        goal_msg.pwm = pwm
        goal_msg.duration_ms = duration_ms

        self.get_logger().info(f'发送目标: ID={servo_id}, PWM={pwm}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝！')
            return
        self.get_logger().info('目标被接受，等待结果...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'执行结束: 成功={result.success}, 最终PWM={result.final_pwm}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'反馈: 当前PWM={feedback.current_pwm}, 进度={feedback.progress:.2f}')

def main(args=None):
    rclpy.init(args=args)
    client = MoveClient()
    # 默认发送 ID=1, PWM=1500, 1000ms
    client.send_goal(1, 1500, 1000)
    rclpy.spin(client)

if __name__ == '__main__':
    main()