#!/usr/bin/env python3
"""动作组播放器：读 yaml → 展开成一串 goal → 按 step 顺序下发。

设计要点（对应考核细则 3.4）：
  1. 动作是「数据」：换一套动作只加一个 yaml，代码一行不改
  2. 同一 step 内的多个关节一起下发（并发），但必须**全部 result 回来**才进下一步
  3. 绝不用 sleep 猜时间 —— 同步信号是 Action 的 result
"""
import os
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self.declare_parameter('action_file', '')
        self.declare_parameter('start_step', 1)
        self.declare_parameter('wait_server_sec', 5.0)
        self.client = ActionClient(self, MoveServo, 'move_servo')

    # ---------- 读 yaml ----------
    def load(self, path):
        if not os.path.isfile(path):
            self.get_logger().error('动作文件不存在: %s' % path)
            return None
        with open(path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        if not isinstance(cfg, dict) or 'steps' not in cfg:
            self.get_logger().error('动作文件格式不对，必须有 steps 字段')
            return None
        return cfg

    # ---------- 发单个 goal，返回 result future ----------
    def _send_one(self, joint):
        goal = MoveServo.Goal()
        goal.id = int(joint['id'])
        goal.pwm = int(joint['pwm'])
        goal.duration_ms = int(joint.get('duration_ms', 800))

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('  id=%d 目标=%d **被拒绝**' % (goal.id, goal.pwm))
            return None
        self.get_logger().info('  id=%d 目标=%d 用时=%dms → 已接受'
                               % (goal.id, goal.pwm, goal.duration_ms))
        return handle.get_result_async()

    # ---------- 播放整套 ----------
    def play(self, cfg):
        name = cfg.get('name', 'unnamed')
        steps = cfg.get('steps', [])
        start = int(self.get_parameter('start_step').value)
        self.get_logger().info('=== 动作组「%s」共 %d 步，从第 %d 步开始 ==='
                               % (name, len(steps), start))

        if not self.client.wait_for_server(
                timeout_sec=float(self.get_parameter('wait_server_sec').value)):
            self.get_logger().error('等不到 /move_servo 服务，先启动 servo_driver')
            return False

        t_all = time.time()
        ok_all = True

        for idx, step in enumerate(steps, 1):
            if idx < start:
                continue
            joints = step.get('joints', [])
            label = step.get('name', 'step%d' % idx)
            self.get_logger().info('--- 第 %d/%d 步：%s（%d 个关节同时下发）'
                                   % (idx, len(steps), label, len(joints)))
            t_step = time.time()

            futures = []
            for joint in joints:
                fut = self._send_one(joint)
                if fut is not None:
                    futures.append((joint, fut))

            # 关键：这一步的所有 result 都回来，才允许进下一步
            for joint, fut in futures:
                rclpy.spin_until_future_complete(self, fut)
                wrapped = fut.result()
                stamp = time.time()
                if wrapped is None:
                    self.get_logger().error('  id=%s 没拿到 result' % joint['id'])
                    ok_all = False
                    continue
                res = wrapped.result
                self.get_logger().info(
                    '  [result] id=%d success=%s 实际=%d 耗时=%.2fs 完成时刻=%.3f msg=%s'
                    % (joint['id'], res.success, res.final_pwm,
                       stamp - t_step, stamp, res.message))
                if not res.success:
                    ok_all = False

            self.get_logger().info('--- 第 %d 步结束，用时 %.2fs' % (idx, time.time() - t_step))

        self.get_logger().info('=== 动作组「%s」播放完毕，总用时 %.2fs，%s ==='
                               % (name, time.time() - t_all,
                                  '全部到位' if ok_all else '有未到位'))
        return ok_all


def resolve_action_file(name):
    """允许只写文件名（如 stand.yaml），自动去包的 config/actions 下找。"""
    if os.path.isabs(name) and os.path.isfile(name):
        return name
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('cquick_servo')
        cand = os.path.join(share, 'config', 'actions', name)
        if os.path.isfile(cand):
            return cand
        # 也支持子目录写法
        cand2 = os.path.join(share, 'config', 'actions', os.path.basename(name))
        if os.path.isfile(cand2):
            return cand2
    except Exception:
        pass
    return name


def main():
    rclpy.init()
    node = SequencePlayer()
    raw = node.get_parameter('action_file').value
    if not raw:
        node.get_logger().error('必须给参数 action_file，例如：\n'
                                '  ros2 run cquick_servo sequence_player '
                                '--ros-args -p action_file:=stand.yaml')
        node.destroy_node()
        rclpy.shutdown()
        return
    path = resolve_action_file(raw)
    node.get_logger().info('动作文件: %s' % path)
    cfg = node.load(path)
    if cfg is not None:
        node.play(cfg)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
