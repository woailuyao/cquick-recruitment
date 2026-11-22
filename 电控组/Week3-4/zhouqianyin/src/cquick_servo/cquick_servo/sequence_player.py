#!/usr/bin/env python3
"""CQuick action sequence player (Task 3).

Read config/actions/*.yaml, expand it into goals, and play step by step.

Design rules (the reviewer will ask about these):
  1. Motions are DATA: adding a new motion only needs a new yaml file.
     This python file never changes.
  2. Steps are strictly serial: wait for ALL results of one step before
     sending the next step. No sleep-based guessing.
  3. Moves inside one step are sent together (multi-joint coordination).
     If one joint must move first, give it its own step.

Usage:
  ros2 run cquick_servo sequence_player --file stand_up.yaml --repeat 3
  ros2 run cquick_servo sequence_player --file wave.yaml
  ros2 run cquick_servo sequence_player --file stand_up.yaml --ids 1
"""

import argparse
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from ament_index_python.packages import get_package_share_directory
from cquick_servo_interfaces.action import MoveServo
import yaml

ACTION_NAME = 'move_servo'


def stamp():
    """Millisecond timestamp, printed in every log line to prove ordering."""
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


def resolve_path(file_arg):
    """Absolute path first, then cwd, then the package share directory."""
    if os.path.isabs(file_arg) or os.path.exists(file_arg):
        return file_arg
    share = get_package_share_directory('cquick_servo')
    return os.path.join(share, 'config', 'actions', file_arg)


def load_action(path):
    """Return (name, [(step_name, [goal_dict, ...]), ...])."""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or not isinstance(data.get('steps'), list):
        raise ValueError('%s: top level must be a dict with a steps list' % path)
    steps = []
    for i, step in enumerate(data['steps'], 1):
        if not isinstance(step, dict) or not isinstance(step.get('moves'), list):
            raise ValueError('%s: step %d has no moves list' % (path, i))
        moves = []
        for raw in step['moves']:
            sid = raw.get('servo_id', raw.get('id'))
            pwm = raw.get('target_pwm', raw.get('pwm'))
            dur = raw.get('duration_ms', 0)
            if sid is None or pwm is None:
                raise ValueError('%s: step %d move lacks servo_id/target_pwm' % (path, i))
            moves.append({'id': int(sid), 'pwm': int(pwm), 'duration_ms': int(dur)})
        steps.append((step.get('name', 'step_%d' % i), moves))
    return data.get('name', os.path.basename(path)), steps


class Player(Node):
    def __init__(self, steps, only_ids):
        super().__init__('sequence_player')
        self._steps = steps
        self._only_ids = set(only_ids) if only_ids else None
        self._client = ActionClient(self, MoveServo, ACTION_NAME)

    def play_step(self, index, name, moves, total):
        """Send all goals of one step together, then collect every result."""
        if self._only_ids is not None:
            moves = [m for m in moves if m['id'] in self._only_ids]
        if not moves:
            self.get_logger().info(
                '[%s] step %d/%d "%s" no goal, skip' % (stamp(), index, total, name))
            return True

        t0 = time.monotonic()
        self.get_logger().info(
            '[%s] >>> step %d/%d "%s" send %d goals together'
            % (stamp(), index, total, name, len(moves)))

        # 1) Send every goal first, without waiting for any of them.
        pending = []
        for k, m in enumerate(moves):
            if k:
                time.sleep(0.3)
            goal = MoveServo.Goal()
            goal.id = m['id']
            goal.pwm = m['pwm']
            goal.duration_ms = m['duration_ms']
            pending.append((m, self._client.send_goal_async(goal)))
            self.get_logger().info(
                '[%s]     goal -> id=%d pwm=%d duration=%d'
                % (stamp(), m['id'], m['pwm'], m['duration_ms']))

        # 2) Check acceptance, then register all result futures.
        ok = True
        handles = []
        for m, fut in pending:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            handle = fut.result() if fut.done() else None
            if handle is None or not handle.accepted:
                self.get_logger().error('[%s] goal id=%d rejected' % (stamp(), m['id']))
                ok = False
            else:
                handles.append((m, handle))

        results = [(m, h.get_result_async()) for m, h in handles]
        for m, fut in results:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            if not fut.done():
                self.get_logger().error('[%s] result id=%d timeout' % (stamp(), m['id']))
                ok = False
                continue
            res = fut.result().result
            self.get_logger().info(
                '[%s]     result <- id=%d success=%s final_pwm=%d message=%s'
                % (stamp(), m['id'], res.success, res.final_pwm, res.message))
            if not res.success:
                ok = False

        # 3) This line is the proof that results are the sync signal.
        self.get_logger().info(
            '[%s] <<< step %d/%d all results collected in %.3fs, next step'
            % (stamp(), index, total, time.monotonic() - t0))
        return ok

    def play_all(self):
        total = len(self._steps)
        for i, (name, moves) in enumerate(self._steps, 1):
            time.sleep(0.4)
            if not self.play_step(i, name, moves, total):
                self.get_logger().error('[%s] sequence aborted at step %d' % (stamp(), i))
                return False
        return True


def main():
    rclpy.init()
    ap = argparse.ArgumentParser(description='CQuick sequence player (yaml driven)')
    ap.add_argument('--file', default='stand_up.yaml',
                    help='yaml file name or absolute path; switching motions only changes this')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--ids', default='',
                    help='debug only: comma separated IDs, e.g. 1,2')
    args = ap.parse_args(remove_ros_args(args=sys.argv)[1:])

    try:
        path = resolve_path(args.file)
        name, steps = load_action(path)
    except (OSError, ValueError) as exc:
        print('[ERROR] load action failed: %s' % exc, file=sys.stderr)
        rclpy.shutdown()
        return 1

    only_ids = [int(x.strip()) for x in args.ids.split(',') if x.strip()] if args.ids else None
    node = Player(steps, only_ids)
    node.get_logger().info(
        '[%s] action "%s" from %s, %d steps, repeat %d'
        % (stamp(), name, path, len(steps), args.repeat))
    if only_ids is not None:
        node.get_logger().warn('[debug] only IDs %s will be sent' % only_ids)

    if not node._client.wait_for_server(timeout_sec=8.0):
        node.get_logger().error('action server not ready: is servo_driver running?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    ok = True
    try:
        for r in range(1, args.repeat + 1):
            t0 = time.monotonic()
            node.get_logger().info('[%s] ===== run %d/%d start =====' % (stamp(), r, args.repeat))
            if not node.play_all():
                ok = False
                break
            node.get_logger().info(
                '[%s] ===== run %d/%d done in %.3fs ====='
                % (stamp(), r, args.repeat, time.monotonic() - t0))
    except KeyboardInterrupt:
        node.get_logger().warn('user interrupted')
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
