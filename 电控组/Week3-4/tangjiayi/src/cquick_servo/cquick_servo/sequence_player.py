#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sequence_player —— 动作组播放器（任务 3 的核心）。

一句话：**动作是数据，不是代码。**
  - 一套动作 = 一个 yaml（config/actions/*.yaml）
  - 播放器只做三件事：读 yaml -> 展开成 goal 序列 -> 按 result 同步地下发
  - 换动作只加一个 yaml，一行 python 都不用改

yaml 结构（字段含义见 README 第 9 节）：
    steps:
      - name: 01_两个关节同时抬
        settle_ms: 100            # 本 step 的 result 全回来后，再多等这么久
        joints:                   # 同一个 step 里的 joints 是「同时下发」
          - {id: 1, pwm: 1700, duration_ms: 700}
          - {id: 2, pwm: 1300, duration_ms: 700}

同步原则（本任务的硬要求）：
  - step 之内：所有 goal 一次性发出，**等全部 result 回来**才进下一步
  - 绝不 time.sleep 猜时间：舵机走完的时间不等于 duration_ms，会累积误差
  - 日志里打 result 的墙钟时间，评审要看「确实是收到 result 才发的下一条」
"""

from __future__ import annotations

import os
import time
from typing import Dict, List, Optional

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty

from cquick_servo_interfaces.action import MoveServo

ACTION_NAME = 'move_servo'
TOPIC_HEARTBEAT = '/servo/heartbeat'


class SequencePlayer(Node):
    def __init__(self) -> None:
        super().__init__('sequence_player')

        # 参数从 yaml 来；action_file 允许 launch 用绝对路径覆盖
        self.declare_parameter('action_file', '')
        self.declare_parameter('loop_times', 1)          # 0 = 一直循环
        self.declare_parameter('dry_run', False)         # true = 只打印展开结果，不动作
        self.declare_parameter('goal_timeout_s', 10.0)   # 单条 goal 最长等多久
        self.declare_parameter('heartbeat_period_ms', 200)

        self._client = ActionClient(self, MoveServo, ACTION_NAME)
        self._heartbeat_pub = self.create_publisher(Empty, TOPIC_HEARTBEAT, 10)
        period = max(0.05, int(self.get_parameter('heartbeat_period_ms').value) / 1000.0)
        self.create_timer(period, self._publish_heartbeat)

        self._preempted = False

    # ------------------------------------------------------------ 主流程
    def play(self) -> bool:
        path = str(self.get_parameter('action_file').value).strip()
        if not path:
            self.get_logger().error('没给 action_file，例：-p action_file:=$HOME/ros2_ws/src/.../stand_up.yaml')
            return False
        if not os.path.isfile(path):
            self.get_logger().error(f'动作文件不存在：{path}')
            return False

        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        steps = data.get('steps') or []
        if not steps:
            self.get_logger().error(f'{path} 里没有 steps')
            return False

        self.get_logger().info(
            f'载入动作组「{data.get("name", os.path.basename(path))}」：'
            f'{len(steps)} 个 step，action_file={path}')

        if bool(self.get_parameter('dry_run').value):
            self._print_plan(steps)
            return True

        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'等不到 Action Server /{ACTION_NAME}')
            return False

        loop_times = int(self.get_parameter('loop_times').value)
        round_index = 0
        while True:
            round_index += 1
            self.get_logger().info(f'===== 第 {round_index} 遍 =====')
            if not self._play_once(steps):
                return False
            if loop_times and round_index >= loop_times:
                break
        self.get_logger().info('动作组播放完毕')
        return True

    def _play_once(self, steps: List[Dict]) -> bool:
        for index, step in enumerate(steps, start=1):
            name = step.get('name') or f'step{index}'
            joints = step.get('joints') or []
            if not joints:
                self.get_logger().warn(f'{name} 没有 joints，跳过')
                continue

            self.get_logger().info(f'--- step {index}/{len(steps)} {name}：'
                                   f'{len(joints)} 个关节同时下发 ---')
            # ① 同一 step 的 goal 全部发出去（不同 ID 在驱动端是并发执行的）
            handles = []
            for joint in joints:
                handle = self._send(joint, name)
                if handle is None:
                    return False
                handles.append((joint, handle))

            # ② 等这一步所有 result 都回来，才允许进下一步
            for joint, handle in handles:
                outcome = self._wait_result(joint, handle, name)
                if outcome != 'ok':
                    self.get_logger().error(f'{name} 中止：{outcome}')
                    return False

            settle_ms = int(step.get('settle_ms') or 0)
            if settle_ms > 0:
                self.get_logger().info(f'{name} 全部 result 已回，再 settle {settle_ms}ms')
                time.sleep(settle_ms / 1000.0)
        return True

    # ------------------------------------------------------------ 单条 goal
    def _send(self, joint: Dict, step_name: str):
        goal = MoveServo.Goal()
        goal.id = int(joint['id'])
        goal.pwm = int(joint['pwm'])
        goal.duration_ms = int(joint.get('duration_ms') or 0)
        self.get_logger().info(
            f'[{self._stamp()}] 发 goal：{step_name} id={goal.id} '
            f'pwm={goal.pwm} duration={goal.duration_ms}ms')
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'goal 下发超时（{step_name} id={goal.id}）')
            return None
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'goal 被拒绝（{step_name} id={goal.id} pwm={goal.pwm}）'
                f'—— 看驱动节点日志里的 rejected 原因')
            return None
        return goal_handle

    def _wait_result(self, joint: Dict, goal_handle, step_name: str) -> str:
        timeout = float(self.get_parameter('goal_timeout_s').value)
        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            return f'等 result 超时（{step_name} id={joint["id"]}）'

        result = future.result().result
        status = future.result().status
        self.get_logger().info(
            f'[{self._stamp()}] 收到 result：{step_name} id={joint["id"]} '
            f'success={result.success} readback={result.pwm_readback} '
            f'status={status} message={result.message}')

        if result.success:
            return 'ok'
        if 'preempted' in result.message:
            # 有人（比如评审现场用 ros2 action send_goal）接管了：整套动作让位
            self._preempted = True
            return f'被抢占，整套动作中止（{result.message}）'
        return f'失败：{result.message}'

    # ------------------------------------------------------------ 杂项
    def _stamp(self) -> str:
        """墙钟时间 HH:MM:SS.mmm。评审要靠这个看「确实收到 result 才发的下一条」。"""
        now = self.get_clock().now().to_msg()
        wall = time.strftime('%H:%M:%S', time.localtime(now.sec))
        return f'{wall}.{now.nanosec // 1_000_000:03d}'

    def _publish_heartbeat(self) -> None:
        self._heartbeat_pub.publish(Empty())

    def _print_plan(self, steps: List[Dict]) -> None:
        total = 0
        for index, step in enumerate(steps, start=1):
            name = step.get('name') or f'step{index}'
            joints = step.get('joints') or []
            total += len(joints)
            self.get_logger().info(f'[dry_run] step {index} {name}（{len(joints)} 个关节）')
            for joint in joints:
                self.get_logger().info(
                    f'[dry_run]    id={joint["id"]} pwm={joint["pwm"]} '
                    f'duration={joint.get("duration_ms")}ms')
        self.get_logger().info(f'[dry_run] 合计 {len(steps)} 个 step / {total} 条 goal')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SequencePlayer()
    exit_code = 0
    try:
        exit_code = 0 if node.play() else 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
