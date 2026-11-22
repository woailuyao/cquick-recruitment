#!/usr/bin/env python3
"""动作组播放器：读 yaml -> 按 step 展开成 goal -> 全部 result 回来才进下一步"""
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


def ts():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')
        self.declare_parameter('action_file', '')
        self.declare_parameter('repeat', 1)
        self.declare_parameter('step_gap', 0.1)
        self._ac = ActionClient(self, MoveServo, 'move_servo')

    def _one_goal(self, spec, tag):
        g = MoveServo.Goal()
        g.id = int(spec['id'])
        g.pwm = int(spec['pwm'])
        g.duration_ms = int(spec.get('duration_ms', 0))
        self.get_logger().info('%s [%s] goal id=%d pwm=%d dur=%dms'
                               % (ts(), tag, g.id, g.pwm, g.duration_ms))
        fut = self._ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('%s [%s] goal 被拒绝 id=%d pwm=%d'
                                    % (ts(), tag, g.id, g.pwm))
            return None
        return gh

    def play(self):
        path = self.get_parameter('action_file').value
        if not path:
            self.get_logger().error('没给 action_file')
            return False
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        steps = data.get('steps', [])
        name = data.get('name', path)
        if not steps:
            self.get_logger().error('%s 里没有 steps' % path)
            return False
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server 不在线：先 ros2 launch cquick_servo servo.launch.py')
            return False

        self.get_logger().info('%s ===== 动作组 %s，%d 个 step =====' % (ts(), name, len(steps)))
        all_ok = True
        for i, step in enumerate(steps, 1):
            tag = '%d/%d %s' % (i, len(steps), step.get('name', 'step'))
            joints = step.get('joints', [])
            self.get_logger().info('%s ---- step %s 开始，%d 个关节同时下发 ----'
                                   % (ts(), tag, len(joints)))
            handles = []
            for spec in joints:
                gh = self._one_goal(spec, tag)
                if gh is None:
                    all_ok = False
                    break
                handles.append((spec, gh))
            if not handles:
                all_ok = False
                continue

            rfs = [(spec, gh.get_result_async()) for spec, gh in handles]
            while not all(rf.done() for _, rf in rfs):
                rclpy.spin_once(self, timeout_sec=0.05)

            for spec, rf in rfs:
                res = rf.result().result
                self.get_logger().info('%s [result] id=%d success=%s final_pwm=%d %s'
                                       % (ts(), int(spec['id']), res.success,
                                          res.final_pwm, res.message))
                if not res.success:
                    all_ok = False
            self.get_logger().info('%s ---- step %s 全部 result 回收完毕 ----' % (ts(), tag))
            time.sleep(float(self.get_parameter('step_gap').value))

        self.get_logger().info('%s ===== 动作组结束，全部成功=%s =====' % (ts(), all_ok))
        return all_ok


def main():
    rclpy.init()
    node = SequencePlayer()
    try:
        repeat = max(1, int(node.get_parameter('repeat').value))
        for k in range(repeat):
            node.get_logger().info('########## 第 %d/%d 遍 ##########' % (k + 1, repeat))
            node.play()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
