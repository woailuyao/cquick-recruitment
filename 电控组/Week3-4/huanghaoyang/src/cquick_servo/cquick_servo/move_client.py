#!/usr/bin/env python3
"""Action 客户端。用法：

    ros2 run cquick_servo move_client 1 1700 1000
                                          ^  ^    ^
                                        ID  PWM  用时ms
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        # Action 名必须和服务端完全一致：move_servo
        self.client = ActionClient(self, MoveServo, 'move_servo')

    def send(self, sid, pwm, dur):
        # 等 server 上线，最多 5 秒。不等就发，会直接报 server not available
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('找不到 move_servo 服务端，先起 launch')
            return

        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = dur

        self.get_logger().info(f'发送 goal: id={sid} pwm={pwm} dur={dur}')
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error('goal 被拒绝（检查 ID 是否在 servo_ids、PWM 是否超限）')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        self.get_logger().info(
            f'result: success={res.success} final_pwm={res.final_pwm} msg={res.message}')

    def on_feedback(self, fb):
        f = fb.feedback
        self.get_logger().info(f'feedback: pwm={f.current_pwm} progress={f.progress:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()

    if len(sys.argv) >= 4:
        sid, pwm, dur = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
    else:
        sid, pwm, dur = 1, 1700, 1000

    node.send(sid, pwm, dur)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
