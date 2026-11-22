#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cquick_servo 动作组播放器（第 3/4 周 · 任务 3.4）

一套动作 = 一个 yaml，播放器只负责「读 yaml → 展开成一串 goal → 按 result 同步打出去」。
**换动作只改 yaml，不动 python。**

用法：
    ros2 run cquick_servo sequence_player --list
    ros2 run cquick_servo sequence_player --action stand_up
    ros2 run cquick_servo sequence_player --action wave --loop
    ros2 run cquick_servo sequence_player --action /abs/path/to/my.yaml   # 现场给一套新动作

yaml 结构（字段名就是下面这些，改动作时按这个写）：
    name: stand_up
    loop: false                     # 这组动作跑完是否自动重来
    steps:
      - name: 前腿抬起
        joints:                     # 同一个 step 里的关节同时下发
          - {id: 1, pwm: 1700, duration_ms: 800}
          - {id: 2, pwm: 1300, duration_ms: 800}
      - name: 回中位                # step 之间串行：上一步 result 全回来才发下一步
        joints:
          - {id: 1, pwm: 1500, duration_ms: 800}
          - {id: 2, pwm: 1500, duration_ms: 800}

时序规则的落地方式：
  · step 内并行  —— 同一个 step 的 N 个 goal 一起 send_goal_async，然后一起等 result；
  · step 间串行  —— 上一步所有 result 都回来才进下一步，用 result 当同步信号，不用 sleep 猜；
  · step 内有先后 —— 想「先动 A 再动 B」就拆成两个 step，不要靠调 duration_ms 硬凑。
"""
import argparse
import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo

ACTIONS_SUBDIR = os.path.join('config', 'actions')


def _now():
    return time.strftime('%H:%M:%S') + '.%03d' % (int(time.time() * 1000) % 1000)


def _actions_dir():
    return os.path.join(get_package_share_directory('cquick_servo'), ACTIONS_SUBDIR)


def available_actions():
    try:
        files = sorted(f for f in os.listdir(_actions_dir()) if f.endswith('.yaml'))
    except OSError:
        return []
    return [os.path.splitext(f)[0] for f in files]


class SequencePlayer(Node):

    def __init__(self, action_name, loop):
        super().__init__('sequence_player')
        self._client = ActionClient(self, MoveServo, '/move_servo')
        self._executor = None
        self._path = self._resolve(action_name)
        self._seq = self._load(self._path)
        self._loop = bool(loop or self._seq.get('loop', False))
        self._stop = False

    # ---------------- yaml ----------------
    def _resolve(self, name):
        if os.path.isabs(name) or name.endswith('.yaml'):
            path = name if os.path.isabs(name) else os.path.abspath(name)
            if not os.path.exists(path):
                raise SystemExit('找不到动作文件：%s' % path)
            return path
        path = os.path.join(_actions_dir(), '%s.yaml' % name)
        if not os.path.exists(path):
            raise SystemExit('找不到动作 %r，可选：%s' % (name, available_actions()))
        return path

    def _load(self, path):
        import yaml
        with open(path, 'r', encoding='utf-8') as fh:
            seq = yaml.safe_load(fh) or {}
        steps = seq.get('steps') or []
        if not steps:
            raise SystemExit('%s 里没有 steps，无法播放' % path)
        for idx, step in enumerate(steps):
            joints = step.get('joints') or []
            if not joints:
                raise SystemExit('%s 第 %d 个 step 没有 joints' % (path, idx + 1))
            for j in joints:
                for key in ('id', 'pwm', 'duration_ms'):
                    if key not in j:
                        raise SystemExit('%s 第 %d 个 step 缺少字段 %s' % (path, idx + 1, key))
        return seq

    # ---------------- 执行 ----------------
    def _spin_until(self, futures, timeout):
        """pump 回调直到所有 future 完成或超时（不 sleep 硬等）。"""
        deadline = time.monotonic() + timeout
        while not all(f.done() for f in futures):
            self._executor.spin_once(timeout_sec=0.02)
            if time.monotonic() > deadline:
                return False
        return True

    def run(self):
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/move_servo 不可用，先起 servo_driver')
            return 1

        name = self._seq.get('name', os.path.basename(self._path))
        steps = self._seq['steps']
        self.get_logger().info('开始播放动作组 %r（%d 步，来自 %s，loop=%s）'
                               % (name, len(steps), self._path, self._loop))

        round_no = 0
        while not self._stop:
            round_no += 1
            t_round = time.monotonic()
            self.get_logger().info('=== 第 %d 遍 ===' % round_no)

            for idx, step in enumerate(steps, start=1):
                if self._stop:
                    break
                label = step.get('name') or ('step%d' % idx)
                joints = step['joints']

                # ① 同一个 step 的 goal 一起发（并行）
                send_futures, handles = [], []
                for j in joints:
                    goal = MoveServo.Goal()
                    goal.id = int(j['id'])
                    goal.pwm = int(j['pwm'])
                    goal.duration_ms = int(j.get('duration_ms', 0))
                    self.get_logger().info('[%s] step%d %r 下发 id=%d pwm=%d dur=%d'
                                           % (_now(), idx, label,
                                              goal.id, goal.pwm, goal.duration_ms))
                    send_futures.append(self._client.send_goal_async(goal))

                if not self._spin_until(send_futures, 5.0):
                    self.get_logger().error('step%d 发送超时' % idx)
                    return 1
                for fut in send_futures:
                    handle = fut.result()
                    if handle is None or not handle.accepted:
                        self.get_logger().error('step%d 有 goal 被 REJECT（检查 servo_ids / pwm 限位）' % idx)
                        return 1
                    handles.append(handle)

                # ② 等这一步所有 result 回来（串行同步点）
                result_futures = [h.get_result_async() for h in handles]
                budget = max(j.get('duration_ms', 0) for j in joints) / 1000.0 + 15.0
                if not self._spin_until(result_futures, budget):
                    self.get_logger().error('step%d 等 result 超时（%.1fs）' % (idx, budget))
                    return 1

                preempted = False
                for j, fut in zip(joints, result_futures):
                    wrapped = fut.result()
                    res = wrapped.result
                    self.get_logger().info(
                        'step[%s] id=%d 完成 success=%s final_pwm=%d message=%s'
                        % (label, int(j['id']), res.success, res.final_pwm, res.message))
                    if not res.success:
                        if 'preempted' in res.message:
                            preempted = True
                        else:
                            self.get_logger().warn('step[%s] id=%d 未成功，动作组中止'
                                                   % (label, int(j['id'])))
                            return 1

                if preempted:
                    # 被新 goal 抢占：立刻停，不要再用后续 step 把控制权抢回来
                    self.get_logger().warn('动作组 %s 被新 goal 抢占，停止播放' % name)
                    return 0

            dt = time.monotonic() - t_round
            self.get_logger().info('动作组 %s 播放完成（第 %d 遍，用时 %.2fs）' % (name, round_no, dt))
            if not self._loop:
                break
        return 0

    def request_stop(self):
        self._stop = True


def main(args=None):
    parser = argparse.ArgumentParser(description='动作组播放器（读 yaml → 发 goal 序列）')
    parser.add_argument('--action', default='stand_up', help='动作名（config/actions/<名>.yaml）或 yaml 绝对路径')
    parser.add_argument('--loop', action='store_true', help='循环播放（yaml 里 loop: true 也生效）')
    parser.add_argument('--list', action='store_true', help='列出可用的动作组')
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)

    if parsed.list:
        names = available_actions()
        print('可用动作组：')
        for n in names:
            print('  - %s' % n)
        rclpy.try_shutdown()
        return 0

    node = SequencePlayer(parsed.action, parsed.loop)
    try:
        return node.run()
    except KeyboardInterrupt:
        print('\n中断，停止播放')
        return 130
    finally:
        if node._executor is not None:
            node._executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
