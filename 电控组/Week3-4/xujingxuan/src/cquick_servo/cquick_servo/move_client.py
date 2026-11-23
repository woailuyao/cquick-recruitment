#!/usr/bin/env python3
"""CQuick 舵机 Action 客户端。

只发 goal，不碰底层函数。

用法：
  ros2 run cquick_servo move_client --id 1 --pwm 1700 --duration 800
  ros2 run cquick_servo move_client --id 1 --pwm 1700          # duration 用节点默认值
  ros2 run cquick_servo move_client --sweep                    # 1500 -> 1700 -> 1500 连打三条
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo

ACTION_NAME = 'move_servo'


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, ACTION_NAME)

    def send(self, servo_id, pwm, duration_ms):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('等不到 Action 服务端 %s，驱动节点起来了吗？' % ACTION_NAME)
            return False

        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration_ms)

        self.get_logger().info('发 goal：id=%d pwm=%d duration=%dms'
                               % (goal.id, goal.pwm, goal.duration_ms))
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝（超范围 / ID 不存在 / 参数刚被改小）')
            return False
        self.get_logger().info('goal 已接受，等结果…')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error('没拿到 result')
            return False

        result = wrapped.result
        if result.success:
            self.get_logger().info('成功：%s（最终 %d）' % (result.message, result.final_pwm))
            return True
        self.get_logger().warn('失败：%s（最终 %d）' % (result.message, result.final_pwm))
        return False

    def _on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # id 从 goal 里拿不到，这里只打驱动给的当前位置与进度
        self.get_logger().info('pwm=%d progress=%.2f'
                               % (feedback.current_pwm, feedback.progress))


def parse_args():
    parser = argparse.ArgumentParser(description='CQuick 舵机 Action 客户端')
    parser.add_argument('--id', type=int, default=0, help='舵机 ID（本机为 000）')
    parser.add_argument('--pwm', type=int, default=1500, help='目标 PWM')
    parser.add_argument('--duration', type=int, default=0,
                        help='动作时长 ms，0 表示用驱动节点的 default_duration_ms')
    parser.add_argument('--sweep', action='store_true', help='连打 1500 -> 1700 -> 1500')
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main(args=None):
    options = parse_args()
    rclpy.init(args=args)
    node = MoveClient()
    try:
        if options.sweep:
            for pwm in (1500, 1700, 1500):
                node.send(options.id, pwm, options.duration)
        else:
            node.send(options.id, options.pwm, options.duration)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
