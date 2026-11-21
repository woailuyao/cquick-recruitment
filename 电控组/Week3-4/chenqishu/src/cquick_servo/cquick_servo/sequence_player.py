#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动作组播放器：读 yaml → 展开成一串 goal → 按 result 同步顺序下发。

支持的 yaml 结构（两种都行）：

  # 写法一（简写：step 级 duration_ms + moves）
  motion_name: squat
  repeat: 3
  steps:
    - name: 蹲下
      duration_ms: 1000
      moves:
        - {id: 3, pwm: 866}
    - name: 保持蹲姿1秒
      hold_ms: 1000
      moves: []

  # 写法二（逐关节 duration_ms + joints）
  name: squat
  steps:
    - name: 蹲下
      joints:
        - {id: 3, pwm: 866, duration_ms: 1000}

设计要点（评审会问）：
  * 动作是【数据】不是代码：换动作只改 yaml
  * 同一 step 内多关节【并行】下发
  * step 之间【等所有 result 回来】才发下一个，不用 sleep 猜时间
  * hold_ms 是【动作本身的停顿】（保持姿势），不是同步手段 —— 同步永远靠 result
  * repeat 让整套动作重复 N 遍，方便验证「可重复」
"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo

DEFAULT_STEP_DURATION_MS = 800


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')

        self.declare_parameter('action_file', '')
        self.declare_parameter('dry_run', False)

        self.client = ActionClient(self, MoveServo, 'move_servo')

        path = self.get_parameter('action_file').value
        if not path:
            path = os.path.join(
                get_package_share_directory('cquick_servo'),
                'config', 'actions', 'squat.yaml')
        self.action_file = path

    # ------------------------------------------------------------------
    def load(self):
        with open(self.action_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def send_one(self, sid, pwm, duration_ms):
        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = duration_ms
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_result(self, goal_handle):
        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        return future.result().result

    def _hold(self, seconds):
        """显式的「保持姿势」停顿，不参与同步。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(0.02)

    # ------------------------------------------------------------------
    def play(self):
        data = self.load()
        self.get_logger().info('读取动作文件: %s' % self.action_file)

        name = data.get('name') or data.get('motion_name') or os.path.basename(self.action_file)
        steps = data.get('steps', [])
        repeat = int(data.get('repeat', 1) or 1)
        if not steps:
            self.get_logger().error('yaml 里没有 steps')
            return False

        self.get_logger().info(
            '动作名 = %s，共 %d 个 step，repeat = %d' % (name, len(steps), repeat))

        dry = bool(self.get_parameter('dry_run').value)

        # dry_run 只打印不发送，不需要 action server；真跑才等
        if not dry and not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('没有 action server，先 launch servo.launch.py')
            return False

        for rnd in range(1, repeat + 1):
            self.get_logger().info('********** 第 %d/%d 遍 **********' % (rnd, repeat))

            for idx, step in enumerate(steps, start=1):
                step_name = step.get('name', 'step%d' % idx)
                joints = step.get('joints') or step.get('moves') or []
                step_dur = int(step.get('duration_ms', DEFAULT_STEP_DURATION_MS))
                hold_ms = int(step.get('hold_ms', 0) or 0)

                self.get_logger().info(
                    '=== [%d/%d] %s：%d 个关节%s ===' % (
                        idx, len(steps), step_name, len(joints),
                        ('，保持 %d ms' % hold_ms) if hold_ms else ''))

                # 纯停顿步（没有关节要动）
                if not joints:
                    if dry:
                        self.get_logger().info('  [dry_run] 保持 %.1f 秒' % (hold_ms / 1000.0))
                    elif hold_ms > 0:
                        self._hold(hold_ms / 1000.0)
                    self.get_logger().info('=== [%d/%d] %s 完成 ===' % (idx, len(steps), step_name))
                    continue

                # ① 同一个 step 内的关节全部发出去（并行）
                handles = []
                for j in joints:
                    sid = int(j['id'])
                    pwm = int(j['pwm'])
                    dur = int(j.get('duration_ms', step_dur))
                    if dry:
                        self.get_logger().info(
                            '  [dry_run] id=%d pwm=%d dur=%d' % (sid, pwm, dur))
                        continue
                    gh = self.send_one(sid, pwm, dur)
                    if gh is None or not gh.accepted:
                        self.get_logger().error(
                            '  goal 被拒：id=%d pwm=%d（超限或 id 不在 servo_ids）' % (sid, pwm))
                        return False
                    handles.append((sid, gh))

                if dry:
                    self.get_logger().info('=== [%d/%d] %s 完成 ===' % (idx, len(steps), step_name))
                    continue

                # ② 等这个 step 的所有 result 回来（不用 sleep 猜时间）
                for sid, gh in handles:
                    res = self.wait_result(gh)
                    self.get_logger().info(
                        '  <- result 回来了：id=%d success=%s final=%d (%s)' % (
                            sid, res.success, res.final_pwm, res.message))
                    if not res.success:
                        self.get_logger().warn(
                            '  id=%d 没到位：%s（继续下一步）' % (sid, res.message))

                # ③ 关节走完之后如果还要保持姿势
                if hold_ms > 0:
                    self._hold(hold_ms / 1000.0)

                self.get_logger().info('=== [%d/%d] %s 完成 ===' % (idx, len(steps), step_name))

            self.get_logger().info('********** 第 %d/%d 遍 完成 **********' % (rnd, repeat))

        self.get_logger().info('动作组播放完毕（%s，%d 步 × %d 遍）' % (name, len(steps), repeat))
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    try:
        node.play()
    except KeyboardInterrupt:
        node.get_logger().warn('被 Ctrl+C 中断')
    except Exception as exc:                      # noqa: BLE001
        node.get_logger().error('播放失败: %s' % exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
