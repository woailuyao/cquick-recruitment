#!/usr/bin/env python3
"""sequence_player.py —— 按 yaml 播放多舵机动作组，支持中途抢占。

用法：
    ros2 run cquick_servo sequence_player --ros-args \
        -p action_file:=$HOME/cquick_ws/src/cquick_servo/config/actions/stand_up.yaml

抢占：
    ros2 topic pub --once /sequence/preempt std_msgs/msg/Empty "{}"
"""
from __future__ import annotations

import os
import sys
import threading

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty

from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    """把 yaml 里的动作组逐个 step 发成 Action 目标。"""

    def __init__(self) -> None:
        super().__init__('sequence_player')

        self.declare_parameter('action_file', '')
        self.declare_parameter('sequence', 'stand_up')
        self.declare_parameter('repeat', 1)
        self.declare_parameter('strict', True)
        self.declare_parameter('step_timeout', 20.0)

        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)
        self._pending = 0
        self._preempted = False
        self._active = {}       # servo_id -> ClientGoalHandle
        self._failed = []       # [(servo_id, message)]

        self._cb_group = ReentrantCallbackGroup()
        self._client = ActionClient(
            self, MoveServo, 'move_servo', callback_group=self._cb_group)

        self.create_subscription(
            Empty, 'sequence/preempt', self._on_preempt, 10,
            callback_group=self._cb_group)

        self.get_logger().info('动作组播放器已启动')

    # ---------- 参数 ----------

    def _param(self, name: str):
        return self.get_parameter(name).value

    # ---------- 加载 ----------

    def load_sequence(self) -> dict:
        action_file = str(self._param('action_file') or '').strip()
        if action_file:
            path = os.path.expanduser(action_file)
        else:
            name = str(self._param('sequence') or 'stand_up')
            path = os.path.join(
                get_package_share_directory('cquick_servo'),
                'config', 'actions', f'{name}.yaml')

        if not os.path.isfile(path):
            raise FileNotFoundError(f'找不到动作文件：{path}')

        with open(path, 'r', encoding='utf-8') as fp:
            data = yaml.safe_load(fp) or {}

        data.setdefault('name', os.path.splitext(os.path.basename(path))[0])
        data.setdefault('steps', [])
        return data

    @staticmethod
    def _parse_joint(joint, step_duration=0) -> tuple:
        """支持 {id,pwm,duration_ms} 或 [id, pwm, duration_ms]。

        duration 三级回退：joint 自己的 > step 的 > MIN_DURATION_MS。
        """
        if isinstance(joint, dict):
            return (
                int(joint['id']),
                int(joint['pwm']),
                int(joint.get('duration_ms', 0) or step_duration or 1000),
            )
        if isinstance(joint, (list, tuple)) and len(joint) >= 3:
            return int(joint[0]), int(joint[1]), int(joint[2])
        raise ValueError(f'joint 格式不对: {joint!r}')

    # ---------- 抢占 ----------

    def _on_preempt(self, _msg) -> None:
        with self._lock:
            self._preempted = True
            handles = list(self._active.values())
            self._active.clear()
        for handle in handles:
            handle.cancel_goal_async()
        self.get_logger().warn(f'抢占：已取消 {len(handles)} 个未完成目标')
        with self._cond:
            self._cond.notify_all()

    # ---------- 回调 ----------

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f'feedback {fb.current_pwm} {fb.progress:.2f}')

    def _on_goal_response(self, future, servo_id: int) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f'#{servo_id:03d} 目标被拒绝')
            self._one_done(servo_id, False, 'goal rejected')
            return

        with self._lock:
            if self._preempted:
                goal_handle.cancel_goal_async()
            else:
                self._active[servo_id] = goal_handle

        goal_handle.get_result_async().add_done_callback(
            lambda fut, sid=servo_id: self._on_result(fut, sid))

    def _on_result(self, future, servo_id: int) -> None:
        resp = future.result()
        with self._lock:
            self._active.pop(servo_id, None)

        if resp.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'#{servo_id:03d} 已取消')
            self._one_done(servo_id, True, 'canceled')
            return

        result = resp.result
        if resp.status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self._one_done(servo_id, True, 'ok')
        else:
            self._one_done(servo_id, False, getattr(result, 'message', ''))

    def _one_done(self, servo_id: int, ok: bool, message: str) -> None:
        with self._cond:
            if not ok:
                self._failed.append((servo_id, message))
                self.get_logger().error(f'#{servo_id:03d} 失败：{message}')
            self._pending = max(0, self._pending - 1)
            self._cond.notify_all()

    # ---------- 播放 ----------

    def _wait_for_server(self, timeout_s: float = 5.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_s)

    def _run_step(self, index: int, step: dict) -> bool:
        name = step.get('name', f'step{index}')
        joints = step.get('joints') or []
        if not joints:
            self.get_logger().warn(f'[{index}] {name} 没有 joint，跳过')
            return True

        parsed = [self._parse_joint(j, int(step.get('duration_ms', 0) or 0)) for j in joints]
        self.get_logger().info(
            f'[{index}] {name}：{len(parsed)} 个舵机同时下发')

        with self._cond:
            if self._preempted:
                return False
            self._pending = len(parsed)
            self._failed = []

        for servo_id, pwm, duration_ms in parsed:
            goal = MoveServo.Goal()
            goal.id = servo_id
            goal.pwm = pwm
            goal.duration_ms = duration_ms

            future = self._client.send_goal_async(
                goal, feedback_callback=self._on_feedback)
            future.add_done_callback(
                lambda fut, sid=servo_id: self._on_goal_response(fut, sid))

        with self._cond:
            finished = self._cond.wait_for(
                lambda: self._pending == 0 or self._preempted,
                timeout=float(self._param('step_timeout')))

        if not finished:
            self.get_logger().error(f'[{index}] {name} 超时，强制进入下一步')
            with self._cond:
                self._pending = 0

        with self._lock:
            if self._preempted:
                return False
            failed = list(self._failed)

        if failed and bool(self._param('strict')):
            self.get_logger().error(f'[{index}] {name} 有失败，strict 模式中止整组')
            return False

        return True

    def play(self) -> bool:
        try:
            seq = self.load_sequence()
        except FileNotFoundError as exc:
            self.get_logger().error(str(exc))
            return False

        steps = seq.get('steps') or []
        if not steps:
            self.get_logger().error('动作组里没有任何 step')
            return False

        if not self._wait_for_server(5.0):
            self.get_logger().error('没有找到 move_servo 服务端')
            return False

        total = int(self._param('repeat') or 1)
        seq_name = seq.get('name')

        for r in range(1, total + 1):
            self.get_logger().info(
                f'=== 第 {r}/{total} 遍：{seq_name}（{len(steps)} 步）===')
            for i, step in enumerate(steps, 1):
                if not self._run_step(i, step):
                    if self._preempted:
                        self.get_logger().warn('已抢占，停止播放')
                    else:
                        self.get_logger().error('播放中止')
                    return False

        self.get_logger().info(f'动作组 {seq_name} 播放完成')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SequencePlayer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ok = False
    try:
        ok = node.play()
    except Exception as exc:            # noqa: BLE001
        node.get_logger().error(f'播放异常：{exc}')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
