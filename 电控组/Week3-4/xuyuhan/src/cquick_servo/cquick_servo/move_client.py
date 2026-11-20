#!/usr/bin/env python3
"""Action 客户端示例：发 goal、打印 feedback、打印 result。

用法：
    ros2 run cquick_servo move_client 1 1600 1000     # id=1 转到 1600，用时 1000ms
    ros2 run cquick_servo move_client                 # 不带参数：正转→反转→回中位
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._goal = None

    def wait_server(self, timeout=10.0):
        self.get_logger().info('等待 action server ...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('超时：move_servo 不可用（驱动节点起了吗？）')
            return False
        self.get_logger().info('已连接 move_servo')
        return True

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'feedback id={self._goal.id} pwm={fb.current_pwm} '
            f'progress={fb.progress:.2f}')

    def send_goal(self, servo_id, pwm, duration_ms, verbose=True):
        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration_ms)
        self._goal = goal

        self.get_logger().info(f'发送 goal id={goal.id} pwm={goal.pwm} dur={goal.duration_ms}')
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'goal 被拒绝（id={goal.id} pwm={goal.pwm}）')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if verbose:
            self.get_logger().info(
                f'result success={result.success} message={result.message} '
                f'final_pwm={result.final_pwm}')
        return bool(result.success)


def main(args=None):
    rclpy.init(args=args)
    argv = remove_ros_args(args=sys.argv)[1:]
    node = MoveClient()

    try:
        if not node.wait_server():
            return 1

        if len(argv) >= 2:
            servo_id = int(argv[0])
            pwm = int(argv[1])
            duration = int(argv[2]) if len(argv) > 2 else 800
            node.send_goal(servo_id, pwm, duration)
        else:
            node.get_logger().info('未给参数，执行演示：正转 → 反转 → 回中位')
            for pwm in (1900, 1100, 1500):
                node.send_goal(1, pwm, 1000)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
