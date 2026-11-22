#!/usr/bin/env python3
"""Action 客户端：发 goal、打印 feedback、打印 result"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self.client = ActionClient(self, MoveServo, 'move_servo')
        self.last_id = 0

    def feedback_cb(self, msg):
        fb = msg.feedback
        print(f'  id={self.last_id} pwm={fb.current_pwm} progress={fb.progress:.2f}',
              flush=True)

    def send(self, sid, pwm, duration):
        self.last_id = int(sid)
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('找不到 action server（驱动节点没起来？）')
            return False

        goal = MoveServo.Goal()
        goal.command.id = int(sid)
        goal.command.pwm = int(pwm)
        goal.command.duration_ms = int(duration)

        print(f'发送 goal: id={sid} pwm={pwm} duration_ms={duration}', flush=True)
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            print('结果: goal 被拒绝（pwm 超限 或 ID 不存在）', flush=True)
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        print(f'结果: success={res.success} final_pwm={res.final_pwm} '
              f'message={res.message}', flush=True)
        return res.success


def main():
    rclpy.init()
    node = MoveClient()

    args = remove_ros_args(args=sys.argv)[1:]
    if len(args) >= 3:
        node.send(args[0], args[1], args[2])
    else:
        print('交互模式：输入 "id pwm duration_ms"，例如 "1 1700 1000"，q 退出')
        while True:
            try:
                line = input('goal> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if line.lower() in ('q', 'quit', 'exit'):
                break
            parts = line.split()
            if len(parts) != 3:
                print('  格式：id pwm duration_ms')
                continue
            node.send(parts[0], parts[1], parts[2])

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__&#8203;main__':
    main()
