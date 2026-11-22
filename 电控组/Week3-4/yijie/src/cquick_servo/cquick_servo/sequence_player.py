#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sequence_player.py —— 读 yaml 动作组，展开成 goal 序列，用 result 同步逐步执行。

用法：
    ros2 run cquick_servo sequence_player --ros-args -p action_file:=stand_up.yaml
    ros2 run cquick_servo sequence_player --ros-args -p action_file:=wave.yaml -p repeat:=3
    ros2 run cquick_servo sequence_player --ros-args -p action_file:=/tmp/reviewer.yaml
    ros2 run cquick_servo sequence_player --ros-args -p action_file:=wave.yaml -p dry_run:=true

同步规则（考核任务 3 的核心）：
    一个 step 内的所有 goal **先全部发出去**，然后**等所有 result 回来**才进下一个 step。
    绝对不用 sleep 猜时间。
"""

import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo.sequence import SequenceError, load_sequence
from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')
        self.declare_parameter('action_file', 'stand_up.yaml')
        self.declare_parameter('repeat', 0)          # 0 表示用 yaml 里的 repeat
        self.declare_parameter('step_gap_ms', 100)   # 步间余量，不是用来凑时序的
        self.declare_parameter('stop_on_error', True)
        self.declare_parameter('dry_run', False)     # true 只打印计划，不连硬件
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._feedback_last = {}

    # ---------- 路径：带目录的当绝对路径，裸文件名去包的 share/config/actions 找 ----------
    def _resolve_action_file(self) -> str:
        raw = os.path.expanduser(str(self.get_parameter('action_file').value))
        if os.path.isabs(raw):
            return raw
        share = get_package_share_directory('cquick_servo')
        return os.path.join(share, 'config', 'actions', raw)

    # ---------- 等 future（带超时，防止永久挂住） ----------
    @staticmethod
    def _spin_until(node, future, timeout: float) -> bool:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        return future.done()

    def _on_feedback(self, feedback_msg, servo_id: int):
        now = time.monotonic()
        progress = feedback_msg.feedback.progress
        if progress < 1.0 and now - self._feedback_last.get(servo_id, 0.0) < 1.0:
            return
        self._feedback_last[servo_id] = now
        self.get_logger().info(
            f'    id={servo_id} feedback: pwm={feedback_msg.feedback.current_pwm} '
            f'progress={progress:.2f}')

    # ---------- 一个 step ----------
    def _play_step(self, index: int, step) -> bool:
        label = f'{index:02d}_{step.name}'
        self.get_logger().info(f'--- step {label}：{len(step.joints)} 个关节同时下发 ---')
        started = time.monotonic()

        # ① 先全部异步发出去：同一个 step 内是并行的
        pending = []
        for joint in step.joints:
            goal = MoveServo.Goal()
            goal.id = joint.servo_id
            goal.pwm = joint.pwm
            goal.duration_ms = joint.duration_ms
            future = self._client.send_goal_async(
                goal,
                feedback_callback=lambda msg, sid=joint.servo_id: self._on_feedback(msg, sid))
            pending.append((joint, future))

        # ② 逐个等「接受」
        handles = []
        for joint, future in pending:
            if not self._spin_until(self, future, timeout=5.0):
                self.get_logger().error(f'{label}: id={joint.servo_id} 发送超时')
                handles.append((joint, None))
                continue
            handle = future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(
                    f'{label}: id={joint.servo_id} pwm={joint.pwm} 被拒绝（看驱动日志 REJECT）')
                handles.append((joint, None))
                continue
            handles.append((joint, handle))

        # ③ 等所有 result 回来，才允许进下一个 step
        ok = True
        for joint, handle in handles:
            if handle is None:
                ok = False
                continue
            result_future = handle.get_result_async()
            budget = (joint.duration_ms / 1000.0 if joint.duration_ms else 5.0) + 5.0
            if not self._spin_until(self, result_future, timeout=budget):
                self.get_logger().error(f'{label}: id={joint.servo_id} 等 result 超时')
                ok = False
                continue
            wrapped = result_future.result()
            result = wrapped.result if wrapped is not None else None
            if result is None:
                ok = False
                continue
            self.get_logger().info(
                f'{label}: id={joint.servo_id} final={result.final_pwm} '
                f'success={result.success} msg={result.message}')
            ok = ok and bool(result.success)

        elapsed = (time.monotonic() - started) * 1000.0
        self.get_logger().info(
            f'=== step {label} 结束 ok={ok} 用时 {elapsed:.0f} ms @ {time.strftime("%H:%M:%S")} ===')
        time.sleep(int(self.get_parameter('step_gap_ms').value) / 1000.0)
        return ok

    # ---------- 整套动作 ----------
    def play(self) -> bool:
        path = self._resolve_action_file()
        try:
            sequence = load_sequence(path)
        except (SequenceError, OSError) as exc:
            self.get_logger().error(f'动作文件有问题：{exc}')
            return False

        repeat = int(self.get_parameter('repeat').value) or sequence.repeat
        self.get_logger().info(
            f'动作【{sequence.name}】共 {len(sequence.steps)} 步，重复 {repeat} 遍；文件：{path}')

        if bool(self.get_parameter('dry_run').value):
            for index, step in enumerate(sequence.steps, start=1):
                plan = ', '.join(
                    f'id={joint.servo_id} pwm={joint.pwm} '
                    f'{joint.duration_ms or "默认"}ms' for joint in step.joints)
                self.get_logger().info(f'  step {index:02d} {step.name}: {plan}')
            self.get_logger().info('dry_run=true：只打印计划，没有连舵机')
            return True

        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('5 秒内没找到 /move_servo：驱动节点起了吗？')
            return False

        stop_on_error = bool(self.get_parameter('stop_on_error').value)
        for round_index in range(1, repeat + 1):
            round_started = time.monotonic()
            for index, step in enumerate(sequence.steps, start=1):
                if not self._play_step(index, step):
                    self.get_logger().error(
                        f'第 {round_index} 遍的 step {index} 没成功，中止本次动作组')
                    if stop_on_error:
                        return False
            self.get_logger().info(
                f'>>> 第 {round_index}/{repeat} 遍结束，总耗时 '
                f'{(time.monotonic() - round_started) * 1000.0:.0f} ms')
        return True


def main():
    rclpy.init()
    node = SequencePlayer()
    try:
        node.play()
    except Exception as exc:            # 文件不存在 / yaml 写错 / 被 Ctrl+C
        node.get_logger().error(f'动作执行失败：{exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
