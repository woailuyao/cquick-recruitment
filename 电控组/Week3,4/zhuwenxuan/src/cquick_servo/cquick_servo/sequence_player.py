#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cquick_servo 动作组播放器（第 3/4 周 · 任务 3）

读 config/actions/ 下的一个 yaml，展开成一串 goal，按顺序下发给 /move_servo。
**每一步等所有舵机的 result 都回来了，才发下一步**（不是用 sleep 猜时间）。

用法：
    ros2 run cquick_servo sequence_player --list
    ros2 run cquick_servo sequence_player --action stand_up
    ros2 run cquick_servo sequence_player --action wave --dry-run
    ros2 run cquick_servo sequence_player --file /path/to/new_action.yaml

yaml 格式（一个文件 = 一套动作）：
    name: stand_up
    steps:
      - name: 双臂展开
        moves:
          - {id: 1, pwm: 1800, duration_ms: 900}
          - {id: 2, pwm: 1200, duration_ms: 900}
      - name: 收腿
        moves:
          - {id: 3, pwm: 1750, duration_ms: 1000}

语义：
  - 同一个 step 里的多条 move **同时下发**（并行）
  - step 与 step 之间**串行**：上一步的所有 result 回来才发下一步
  - 某个关节要先动，就单独给它开一个 step（不要靠调 duration_ms 硬凑先后）
"""
import argparse
import os
import sys

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


def actions_dir():
    """动作 yaml 所在目录：优先用 install 里的 share，找不到退回源码目录"""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('cquick_servo'),
                         'config', 'actions')
        if os.path.isdir(p):
            return p
    except Exception:  # noqa: BLE001
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'config', 'actions')


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')
        self._client = ActionClient(self, MoveServo, '/move_servo')

    # ---------- 一个 step：可能同时含多个舵机 ----------
    def do_step(self, step_name, moves, timeout_sec=25.0, tolerant=False):
        """把这一步的所有 goal 都发出去，等全部 result 回来才返回"""
        handles = []
        for m in moves:
            goal = MoveServo.Goal()
            goal.id = int(m['id'])
            goal.pwm = int(m['pwm'])
            goal.duration_ms = int(m.get('duration_ms', 800))

            self.get_logger().info(
                'step[%s] → id=%d pwm=%d duration=%d'
                % (step_name, goal.id, goal.pwm, goal.duration_ms))

            send_future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error('step[%s] id=%d 被拒绝（REJECT）'
                                        % (step_name, goal.id))
                return False
            handles.append((goal.id, handle))

        # 关键：全部 goal 发完，再统一等 result —— 不用 sleep 猜时间
        ok = True
        for sid, handle in handles:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future,
                                             timeout_sec=timeout_sec)
            wrapped = result_future.result()
            if wrapped is None:
                self.get_logger().error('step[%s] id=%d 等不到 result'
                                        % (step_name, sid))
                ok = False
                continue
            r = wrapped.result
            self.get_logger().info(
                'step[%s] id=%d 完成 success=%s final_pwm=%d message=%s'
                % (step_name, sid, r.success, r.final_pwm, r.message))
            if not r.success:
                ok = False
                if tolerant:
                    self.get_logger().warn('step[%s] id=%d 没到位，容错继续'
                                           % (step_name, sid))
        return ok

    # ---------- 播放整套动作 ----------
    def play(self, action_name, steps, dry_run=False, tolerant=False):
        self.get_logger().info('播放动作：%s（%d 个 step）'
                               % (action_name, len(steps)))
        if not dry_run and not self._client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('/move_servo 不可用，先把 servo_driver 起来')
            return False

        for idx, step in enumerate(steps, 1):
            step_name = step.get('name') or ('step%d' % idx)
            moves = step.get('moves') or []
            if not moves:
                self.get_logger().warn('step[%s] 没有 moves，跳过' % step_name)
                continue

            if dry_run:
                for m in moves:
                    self.get_logger().info(
                        '[dry-run] step[%s] id=%s pwm=%s duration=%s'
                        % (step_name, m.get('id'), m.get('pwm'),
                           m.get('duration_ms', 800)))
                continue

            if not self.do_step(step_name, moves, tolerant=tolerant) and not tolerant:
                self.get_logger().error('step[%s] 失败，动作中止' % step_name)
                return False

        self.get_logger().info('动作 %s 播放完成' % action_name)
        return True


def load_action(path):
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    steps = data.get('steps')
    if not isinstance(steps, list):
        raise ValueError('yaml 里必须有 steps: 列表')
    return data.get('name') or os.path.splitext(os.path.basename(path))[0], steps


def main(args=None):
    parser = argparse.ArgumentParser(description='动作组播放器（读 yaml 播动作）')
    parser.add_argument('--action', help='动作名，对应 config/actions/<name>.yaml')
    parser.add_argument('--file', help='直接指定 yaml 路径（覆盖 --action）')
    parser.add_argument('--list', action='store_true', help='列出可用的动作')
    parser.add_argument('--dry-run', action='store_true', help='只打印计划，不发 goal')
    parser.add_argument('--tolerant', action='store_true', help='某步没到位也继续')
    parsed, ros_args = parser.parse_known_args(args)

    d = actions_dir()
    available = sorted(os.path.splitext(f)[0] for f in os.listdir(d)
                       if f.endswith(('.yaml', '.yml'))) if os.path.isdir(d) else []

    if parsed.list or (not parsed.action and not parsed.file):
        print('动作目录：%s' % d)
        for name in available:
            print('  %-12s 对应 %s.yaml' % (name, name))
        return 0 if parsed.list else 2

    if parsed.file:
        path = os.path.abspath(os.path.expanduser(parsed.file))
    else:
        path = os.path.join(d, parsed.action + '.yaml')
    if not os.path.exists(path):
        print('找不到动作文件：%s（可用：%s）' % (path, ', '.join(available)),
              file=sys.stderr)
        return 2

    try:
        name, steps = load_action(path)
    except Exception as exc:  # noqa: BLE001
        print('解析 yaml 失败：%s' % exc, file=sys.stderr)
        return 2

    rclpy.init(args=ros_args)
    node = SequencePlayer()
    try:
        ok = node.play(name, steps, dry_run=parsed.dry_run,
                       tolerant=parsed.tolerant)
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
