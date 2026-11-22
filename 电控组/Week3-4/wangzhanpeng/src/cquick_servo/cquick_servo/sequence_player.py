#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sequence player for Task 3.

Read a YAML action file and play it through the /move_servo Action.
Each step sends all of its actions at once, then waits for every result
before moving to the next step (no fixed sleeps between steps).

YAML schema:
    name: stand_up
    default_duration_ms: 1000
    servos:
      1: {invert: false, pwm_min: 900, pwm_max: 2100}
      2: {invert: true}
    steps:
      - name: home_all
        actions:
          - {id: 1, pwm: 1500, duration_ms: 1500}
          - {id: 2, pwm: 1500, duration_ms: 1500}
      - name: joint1_up
        actions:
          - {id: 1, pwm: 1620, duration_ms: 1000}
"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')

        self.declare_parameter('action_file', '')
        self.declare_parameter('repeat', 1)
        self.declare_parameter('result_timeout_margin', 3.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)

        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._repeat = max(1, int(self.get_parameter('repeat').value))
        self._margin = max(1.0, float(
            self.get_parameter('result_timeout_margin').value))
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._pwm_min = int(self.get_parameter('pwm_min').value)
        self._pwm_max = int(self.get_parameter('pwm_max').value)
        self._active = []

    # ------------------------------------------------------------------
    def resolve_path(self):
        path = str(self.get_parameter('action_file').value).strip()
        if not path:
            share = get_package_share_directory('cquick_servo')
            path = os.path.join(share, 'config', 'actions', 'stand_up.yaml')
        return os.path.abspath(path)

    def load_file(self, path):
        with open(path, 'r', encoding='utf-8') as handle:
            doc = yaml.safe_load(handle)
        if not isinstance(doc, dict):
            raise ValueError('YAML 顶层必须是字典')

        steps = doc.get('steps')
        if not isinstance(steps, list) or not steps:
            raise ValueError('缺少 steps，或 steps 不是非空列表')

        default_duration = int(doc.get('default_duration_ms', 1000))
        servos = {}
        for key, cfg in (doc.get('servos') or {}).items():
            servos[int(key)] = cfg or {}

        prepared = []
        for step_index, step in enumerate(steps, 1):
            if not isinstance(step, dict):
                raise ValueError(f'第 {step_index} 个 step 必须是字典')
            actions = step.get('actions')
            if not isinstance(actions, list) or not actions:
                raise ValueError(f'第 {step_index} 个 step 的 actions 必须是非空列表')

            prepared_actions = []
            for action_index, action in enumerate(actions, 1):
                if (not isinstance(action, dict)
                        or 'id' not in action or 'pwm' not in action):
                    raise ValueError(
                        f'step {step_index} 第 {action_index} 条缺少 id 或 pwm')

                servo_id = int(action['id'])
                pwm = int(action['pwm'])
                duration = int(action.get('duration_ms', default_duration))
                cfg = servos.get(servo_id, {})
                low = int(cfg.get('pwm_min', self._pwm_min))
                high = int(cfg.get('pwm_max', self._pwm_max))

                if not low <= pwm <= high:
                    raise ValueError(
                        f'step {step_index} id={servo_id} pwm={pwm} '
                        f'超出 {low}~{high}')
                if duration < 1000:
                    self.get_logger().warn(
                        f'step {step_index} id={servo_id} duration_ms={duration} '
                        f'< 1000（本板 T<1000ms 不执行），按 1000 处理')
                    duration = 1000

                invert = bool(cfg.get('invert', False))
                prepared_actions.append({
                    'id': servo_id,
                    'pwm': 3000 - pwm if invert else pwm,
                    'raw_pwm': pwm,
                    'duration_ms': duration,
                    'invert': invert,
                })

            prepared.append({
                'name': step.get('name', f'step{step_index}'),
                'actions': prepared_actions,
            })

        return doc.get('name', os.path.basename(path)), prepared

    # ------------------------------------------------------------------
    def _wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done()

    def _cancel_all(self, handles):
        for _, goal_handle in handles:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._active = []

    def run_step(self, index, step):
        actions = step['actions']
        self.get_logger().info(
            f'--- step {index} [{step["name"]}] 下发 {len(actions)} 个舵机 ---')

        handles = []
        self._active = handles
        for action in actions:
            goal = MoveServo.Goal(
                id=action['id'],
                pwm=action['pwm'],
                duration_ms=action['duration_ms'])
            future = self._client.send_goal_async(goal)
            if not self._wait(future, 5.0):
                self.get_logger().error(
                    f'step {index}: id={action["id"]} 等待 goal 响应超时')
                self._cancel_all(handles)
                return False

            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(
                    f'step {index}: id={action["id"]} goal 被拒'
                    f'（检查 id 是否在 servo_ids、pwm 是否越界）')
                self._cancel_all(handles)
                return False

            handles.append((action, goal_handle))
            note = ' (invert)' if action['invert'] else ''
            self.get_logger().info(
                f'step {index}: id={action["id"]} pwm={action["pwm"]}{note} '
                f'T{action["duration_ms"]} accepted')

        longest = max(a['duration_ms'] for a in actions) / 1000.0
        for action, goal_handle in handles:
            result_future = goal_handle.get_result_async()
            if not self._wait(result_future, longest + self._margin):
                self.get_logger().error(
                    f'step {index}: id={action["id"]} 等 result 超时')
                self._cancel_all(handles)
                return False

            response = result_future.result()
            result = response.result
            stamp = time.strftime('%H:%M:%S')
            if result.success:
                self.get_logger().info(
                    f'step {index}: id={action["id"]} RESULT success '
                    f'final_pwm={result.final_pwm} ({result.message}) [{stamp}]')
            else:
                self.get_logger().error(
                    f'step {index}: id={action["id"]} RESULT fail '
                    f'final_pwm={result.final_pwm} ({result.message}) [{stamp}]')
                self._cancel_all(handles)
                return False

        self._active = []
        self.get_logger().info(f'step {index} 完成：全部 result 已收到')
        return True

    # ------------------------------------------------------------------
    def play(self, path):
        try:
            name, steps = self.load_file(path)
        except Exception as exc:
            self.get_logger().error(f'加载 {path} 失败: {exc}')
            return False

        self.get_logger().info(
            f'动作文件: {path}  name={name}  steps={len(steps)}  '
            f'repeat={self._repeat}')

        if self._dry_run:
            for index, step in enumerate(steps, 1):
                detail = ', '.join(
                    f'id={a["id"]} pwm={a["raw_pwm"]}'
                    f'{"(inv)" if a["invert"] else ""} T{a["duration_ms"]}'
                    for a in step['actions'])
                self.get_logger().info(
                    f'[dry-run] step {index} [{step["name"]}]: {detail}')
            return True

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('10 秒内没等到 /move_servo 服务端')
            return False

        for round_index in range(1, self._repeat + 1):
            self.get_logger().info(
                f'===== 第 {round_index}/{self._repeat} 遍 =====')
            for index, step in enumerate(steps, 1):
                if not self.run_step(index, step):
                    self.get_logger().error('序列中止')
                    return False

        self.get_logger().info('全部动作执行完毕')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    ok = False
    try:
        ok = node.play(node.resolve_path())
    except KeyboardInterrupt:
        node.get_logger().warn('用户中断，序列停止')
        node._cancel_all(node._active)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    main()
