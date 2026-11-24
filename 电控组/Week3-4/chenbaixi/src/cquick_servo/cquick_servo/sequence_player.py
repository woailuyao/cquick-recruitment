#!/usr/bin/env python3
"""动作组播放器：读 yaml -> 展开为 goal 序列 -> step 内并行、step 间用 result 同步"""

import os

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from cquick_servo_interfaces.action import MoveServo


def resolve_action_file(name, node):
    if os.path.isabs(name) and os.path.exists(name):
        return name
    if os.path.exists(name):
        return name
    try:
        share = get_package_share_directory('cquick_servo')
    except Exception:
        share = ''
    if share:
        p = os.path.join(share, 'config', 'actions', name)
        if os.path.exists(p):
            return p
    node.get_logger().error('找不到动作文件：' + name)
    return ''


class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self.declare_parameter('action_file', 'stand_up.yaml')
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('dry_run', False)
        self.client = ActionClient(self, MoveServo, 'move_servo')

    def run(self):
        path = resolve_action_file(self.get_parameter('action_file').value, self)
        if not path:
            return False
        with open(path, 'r', encoding='utf-8') as f:
            seq = yaml.safe_load(f)

        steps = seq.get('steps', [])
        name = seq.get('name', 'unnamed')
        dry = bool(self.get_parameter('dry_run').value)
        default_dur = int(seq.get('default_duration_ms',
                                  self.get_parameter('default_duration_ms').value))
        self.get_logger().info(
            "载入动作 '%s' 共 %d 个 step（%s） dry_run=%s"
            % (name, len(steps), path, dry))

        if not dry and not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action Server 没起来')
            return False

        for i, step in enumerate(steps, 1):
            sname = step.get('name', 'step%d' % i)
            cmds = step.get('commands', [])
            self.get_logger().info(
                '===== step %d/%d：%s（%d 条，同时下发）====='
                % (i, len(steps), sname, len(cmds)))
            handles = []
            for c in cmds:
                sid = int(c['id'])
                pwm = int(c['pwm'])
                dur = int(c.get('duration_ms', default_dur))
                if dry:
                    self.get_logger().info(
                        '[dry_run] #%03dP%04dT%04d!  (%s)' % (sid, pwm, dur, sname))
                    continue
                goal = MoveServo.Goal()
                goal.id = sid
                goal.pwm = pwm
                goal.duration_ms = dur
                fut = self.client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, fut)
                h = fut.result()
                if h is None or not h.accepted:
                    self.get_logger().error(
                        'goal 被拒：id=%d pwm=%d（%s）' % (sid, pwm, sname))
                    return False
                handles.append((sid, pwm, h))

            if dry:
                continue

            for sid, pwm, h in handles:
                rfut = h.get_result_async()
                rclpy.spin_until_future_complete(self, rfut)
                res = rfut.result().result
                t = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(
                    '[result] step=%s id=%d target=%d success=%s final_pwm=%d '
                    'msg="%s" t=%.3f'
                    % (sname, sid, pwm, res.success, res.final_pwm, res.message, t))

        self.get_logger().info('动作组播放完成')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()