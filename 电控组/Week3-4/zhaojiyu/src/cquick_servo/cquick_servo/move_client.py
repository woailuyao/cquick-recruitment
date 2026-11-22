#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cquick_servo 客户端：从参数读一串目标，按顺序发 goal，用 result 当同步信号

用法（每个目标写成 id:pwm:duration，空格分隔）：
    ros2 run cquick_servo move_client --ros-args \
        -p targets:="['1:1700:1000', '1:1300:1000', '1:1500:1000']"

设计要点：
  - 输入能指定 id 和 pwm；
  - 必须发 goal，不直接调底层函数；
  - 收到 feedback 打印 id / pwm / progress；
  - 收到 result 打印成功或失败原因；
  - 上一条的 result 回来才发下一条，不用 sleep 猜时间。
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')
        self.declare_parameter('targets', ['1:1700:1000', '1:1300:1000', '1:1500:1000'])
        self.declare_parameter('server_timeout_s', 10.0)
        self.declare_parameter('goal_timeout_s', 30.0)

        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._cur_id = 0
        self._fb_count = 0

    @staticmethod
    def parse_target(spec):
        parts = str(spec).replace('：', ':').split(':')
        if len(parts) < 2:
            raise ValueError('格式应为 id:pwm:duration，收到 %r' % spec)
        sid = int(parts[0])
        pwm = int(parts[1])
        dur = int(parts[2]) if len(parts) > 2 else 0
        return sid, pwm, dur

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._fb_count += 1
        self.get_logger().info('id=%d pwm=%d progress=%.2f'
                               % (self._cur_id, fb.current_pwm, fb.progress))

    def send_one(self, sid, pwm, dur):
        if not self._client.wait_for_server(
                timeout_sec=self.get_parameter('server_timeout_s').value):
            self.get_logger().error('Action 服务器没有出现')
            return False

        goal = MoveServo.Goal()
        goal.id = int(sid)
        goal.pwm = int(pwm)
        goal.duration_ms = int(dur)

        self._cur_id = int(sid)
        self._fb_count = 0
        self.get_logger().info('发 goal: id=%d pwm=%d duration_ms=%d' % (sid, pwm, dur))

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝（超限或 ID 不合法）')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future,
            timeout_sec=self.get_parameter('goal_timeout_s').value)

        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error('等 result 超时')
            return False

        res = wrapped.result
        if res.success:
            self.get_logger().info('结果: success=True final_pwm=%d（%d 条 feedback）message=%s'
                                   % (res.final_pwm, self._fb_count, res.message))
        else:
            self.get_logger().error('结果: success=False final_pwm=%d message=%s'
                                    % (res.final_pwm, res.message))
        return bool(res.success)

    def run_targets(self):
        targets = list(self.get_parameter('targets').value)
        ok_count = 0
        for i, spec in enumerate(targets, 1):
            try:
                sid, pwm, dur = self.parse_target(spec)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                continue
            self.get_logger().info('--- 第 %d/%d 步 ---' % (i, len(targets)))
            if self.send_one(sid, pwm, dur):
                ok_count += 1
        self.get_logger().info('全部完成: %d/%d 成功' % (ok_count, len(targets)))


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    try:
        node.run_targets()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
