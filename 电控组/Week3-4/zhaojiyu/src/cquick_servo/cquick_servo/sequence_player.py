#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sequence_player.py —— 动作组播放器

一套动作 = 一个 yaml，里面是若干 step：
  - step 内多个 goal 同时下发（多关节协同）
  - 上一个 step 的 result 全部回来，才发下一个（不用 sleep 猜时间）
  - 需要先后顺序就拆成两个 step

用法：
  ros2 run cquick_servo sequence_player --ros-args \
      -p action_file:=/绝对路径/stand_up.yaml -p repeat:=3
"""
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


def stamp():
    return time.strftime('%H:%M:%S') + '.%03d' % (int(time.time() * 1000) % 1000)


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')
        self.declare_parameter('action_file', '')
        self.declare_parameter('repeat', 1)
        self.declare_parameter('goal_timeout_s', 30.0)
        self.declare_parameter('server_timeout_s', 10.0)
        self.declare_parameter('gap_ms', 300.0)
        self._client = ActionClient(self, MoveServo, 'move_servo')

    def run_step(self, step_no, total, step):
        """一个 step 内的 move 逐个下发：上一条 result 回来，才发下一条。

        本机控制板同一时刻只能驱动一路动作（实测：两条 goal 并发时两路都不动），
        所以这里退化为逐个下发。yaml 里仍然可以把多个关节写进同一个 step，
        表达“这一步要它们一起做”，只是执行时按顺序落地。
        """
        name = step.get('name', 'step%d' % step_no)
        moves = step.get('moves', [])
        self.get_logger().info('=== 第 %d/%d 步：%s（%d 个 move，逐个下发）@%s'
                               % (step_no, total, name, len(moves), stamp()))

        t0 = time.monotonic()
        ok_all = True
        for m in moves:
            goal = MoveServo.Goal()
            goal.id = int(m['id'])
            goal.pwm = int(m['pwm'])
            goal.duration_ms = int(m.get('duration_ms', 0))

            fut = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            h = fut.result()
            if h is None or not h.accepted:
                self.get_logger().error('  id=%d 的 goal 被拒绝' % goal.id)
                ok_all = False
                continue
            self.get_logger().info('  接受 id=%d pwm=%d @%s'
                                   % (goal.id, goal.pwm, stamp()))

            rf = h.get_result_async()
            rclpy.spin_until_future_complete(
                self, rf, timeout_sec=self.get_parameter('goal_timeout_s').value)
            w = rf.result()
            if w is None:
                self.get_logger().error('  id=%d 等 result 超时' % goal.id)
                ok_all = False
                continue
            r = w.result
            self.get_logger().info('  完成 id=%d success=%s final_pwm=%d %s @%s'
                                   % (goal.id, r.success, r.final_pwm, r.message, stamp()))
            ok_all = ok_all and r.success

        self.get_logger().info('--- 第 %d 步结束，用时 %.0f ms ---'
                               % (step_no, (time.monotonic() - t0) * 1000))
        return ok_all

    def run_sequence(self, data):
        steps = data.get('steps') or []
        self.get_logger().info('动作组: %s（%d 步）'
                               % (data.get('name', '<未命名>'), len(steps)))
        for i, step in enumerate(steps, 1):
            self.run_step(i, len(steps), step)

    def main_loop(self):
        path = self.get_parameter('action_file').value
        if not path:
            self.get_logger().error('没有指定 action_file')
            return False
        with open(path, 'r', encoding='utf-8') as fh:
            data = yaml.safe_load(fh)

        if not self._client.wait_for_server(
                timeout_sec=self.get_parameter('server_timeout_s').value):
            self.get_logger().error('Action 服务端没出现')
            return False

        repeat = max(1, int(self.get_parameter('repeat').value))
        for k in range(1, repeat + 1):
            self.get_logger().info('########## 第 %d/%d 遍 ##########' % (k, repeat))
            self.run_sequence(data)
        self.get_logger().info('全部完成')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
