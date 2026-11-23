#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sequence_player.py —— 动作组播放器

把 yaml 里定义的「一套动作」展开成一串 Action goal，按 step 顺序打出去。

时序规则（这三条是任务 3 的硬要求）：
  1. 同一个 step 内的多条 move  → 同时下发（并发）
  2. step 与 step 之间          → 等上一步所有 result 都回来才发下一条（不 sleep）
  3. 需要先后                    → 在 yaml 里单独开一个 step

用法：
    ros2 run cquick_servo sequence_player --ros-args -p action_file:=stand.yaml
    ros2 run cquick_servo sequence_player --ros-args \
        -p action_file:=/abs/path/stand.yaml -p repeat:=3

参数：
    action_file     动作组文件（可以是绝对路径，也可以只写名字，会在包内 config/actions/ 找）
    repeat          重复遍数；<=0 时用 yaml 里的 repeat（默认 1）
    step_timeout_s  单个 step 等 result 的超时（默认 20 秒）
"""
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class ActionPlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self.declare_parameter('action_file', '')
        self.declare_parameter('repeat', 0)
        self.declare_parameter('step_timeout_s', 20.0)
        self.declare_parameter('step_gap_ms', 300)
        self.step_gap_s = int(self.get_parameter('step_gap_ms').value) / 1000.0
        self.client = ActionClient(self, MoveServo, 'move_servo')

    # ---------------- 配置文件 ----------------
    def _resolve_path(self, raw):
        """支持三种写法：绝对路径 / 相对路径 / 只给名字（去包里找）"""
        if not raw:
            raise ValueError('必须给 action_file 参数')
        if os.path.isabs(raw):
            if not os.path.exists(raw):
                raise FileNotFoundError(raw)
            return raw
        if os.path.exists(raw):
            return os.path.abspath(raw)
        name = raw if raw.endswith(('.yaml', '.yml')) else raw + '.yaml'
        share = get_package_share_directory('cquick_servo')
        cand = os.path.join(share, 'config', 'actions', name)
        if not os.path.exists(cand):
            raise FileNotFoundError(
                '找不到动作文件：%s（也试过 %s）' % (raw, cand))
        return cand

    def load(self, raw):
        path = self._resolve_path(raw)
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise ValueError('yaml 顶层必须是字典')
        steps = data.get('steps')
        if not isinstance(steps, list) or not steps:
            raise ValueError('steps 必须是非空列表')
        for i, step in enumerate(steps, 1):
            moves = step.get('moves') if isinstance(step, dict) else None
            if not isinstance(moves, list) or not moves:
                raise ValueError('第 %d 步缺少 moves 或为空' % i)
            for j, mv in enumerate(moves, 1):
                for key in ('id', 'pwm', 'duration_ms'):
                    if key not in mv:
                        raise ValueError(
                            '第 %d 步第 %d 条缺少字段 %s' % (i, j, key))
        self.get_logger().info(
            '动作组「%s」：%d 步，%s'
            % (data.get('name', os.path.basename(path)), len(steps),
               data.get('description', '')))
        return data, path

    # ---------------- 播放 ----------------
    def _run_step(self, idx, total, step):
        moves = step.get('moves', [])
        label = step.get('name', '')
        expect = max(mv['duration_ms'] for mv in moves) / 1000.0

        self.get_logger().info(
            '── 第 %d/%d 步 %s：%d 条指令同时下发（预计 %.2fs）'
            % (idx, total, ('「%s」' % label) if label else '', len(moves), expect))

        t0 = time.monotonic()

        # 1) 并发下发：先把所有 send_goal 请求都扔出去
        handles = []
        for mv in moves:
            goal = MoveServo.Goal()
            goal.id = int(mv['id'])
            goal.pwm = int(mv['pwm'])
            goal.duration_ms = int(mv['duration_ms'])
            fut = self.client.send_goal_async(goal)
            handles.append((mv, fut))

        # 2) 等每个 goal 被接受
        accepted = []
        for mv, fut in handles:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            gh = fut.result() if fut.done() else None
            if gh is None or not gh.accepted:
                self.get_logger().error(
                    '  指令被拒绝：id=%d pwm=%d' % (mv['id'], mv['pwm']))
                return False, time.monotonic() - t0
            accepted.append((mv, gh))

        # 3) 等所有 result（这是「用 result 同步」，不是 sleep）
        timeout = float(self.get_parameter('step_timeout_s').value)
        for mv, gh in accepted:
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
            if not rf.done():
                self.get_logger().error('  等 result 超时：id=%d' % mv['id'])
                return False, time.monotonic() - t0
            res = rf.result()
            r = res.result
            if res.status != GoalStatus.STATUS_SUCCEEDED or not r.success:
                self.get_logger().error(
                    '  失败：id=%d 状态=%d message=%s'
                    % (mv['id'], res.status, r.message))
                return False, time.monotonic() - t0
            self.get_logger().info(
                '  ok id=%d → %d（PRAD %d）' % (mv['id'], mv['pwm'], r.final_pwm))

        real = time.monotonic() - t0
        self.get_logger().info(
            '  本步实测 %.2fs / 预计 %.2fs（若串行下发应为 %.2fs）'
            % (real, expect, sum(mv['duration_ms'] for mv in moves) / 1000.0))
        return True, real

    def play(self, data, repeat):
        steps = data['steps']
        self.get_logger().info('等待 move_servo 动作服务器…')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('没有找到 /move_servo 动作服务器')
            return False

        # 串行总时长（假设不并发）与并发总时长，用来说明时序对不对
        serial = sum(sum(mv['duration_ms'] for mv in s['moves']) for s in steps) / 1000.0
        parallel = sum(max(mv['duration_ms'] for mv in s['moves']) for s in steps) / 1000.0
        self.get_logger().info(
            '开始播放：%d 遍；理论并行 %.2fs / 理论串行 %.2fs'
            % (repeat, parallel * repeat, serial * repeat))

        t_all = time.monotonic()
        for rep in range(repeat):
            if repeat > 1:
                self.get_logger().info('=== 第 %d/%d 遍 ===' % (rep + 1, repeat))
            for i, step in enumerate(steps, 1):
                ok, _ = self._run_step(i, len(steps), step)
                if not ok:
                    # 控制板偶发丢指令（实测：连跑多遍时，某一步的前一动作刚结束，
                    # 新目标被丢掉，报"未到位 差 +100 左右"）。重发一次通常就好。
                    self.get_logger().warn('  本步失败，重发一次（应对控制板偶发丢指令）')
                    time.sleep(0.5)
                    ok, _ = self._run_step(i, len(steps), step)
                if not ok:
                    self.get_logger().error('动作组中止')
                    return False
                # 步间保护间隔：不是用来同步（同步靠 result），
                # 而是给控制板留出退出"当前运动"的时间 ——
                # 实测 result 返回后 20ms 就发下一步，板子会丢掉新目标，动作直接不动。
                if self.step_gap_s > 0:
                    time.sleep(self.step_gap_s)
            # 遍与遍之间也留一次间隔（否则第 2 遍第 1 步容易丢指令）
            if repeat > 1 and rep < repeat - 1 and self.step_gap_s > 0:
                time.sleep(self.step_gap_s)

        total = time.monotonic() - t_all
        self.get_logger().info(
            '全部完成：%d 遍 × %d 步，实测 %.2fs（理论并行 %.2fs，理论串行 %.2fs）'
            % (repeat, len(steps), total, parallel * repeat, serial * repeat))
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ActionPlayer()
    ok = False
    try:
        data, path = node.load(node.get_parameter('action_file').value)
        repeat = int(node.get_parameter('repeat').value)
        if repeat <= 0:
            repeat = int(data.get('repeat', 1) or 1)
        node.get_logger().info('动作文件：%s（重复 %d 遍）' % (path, repeat))
        ok = node.play(data, repeat)
    except Exception as exc:  # noqa: BLE001 —— 配置文件错误要一句话讲清
        node.get_logger().error('播放失败：%s' % exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
