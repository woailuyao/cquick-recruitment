#!/usr/bin/env python3
"""动作组播放器，支持抢占。

抢占逻辑：
  - 订阅 /servo/preempt，收到一个动作名就记下来
  - 每 0.1 秒检查一次标志，发现抢占立即放弃等待当前 result
  - 主循环切到新动作，从当前位置直接下发（舵机天然从当前位置平滑过去）
  - 旧 goal 不 cancel（cancel 会回中位），靠驱动的 cmd_seq 让它自动放弃校验
"""
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from cquick_servo_interfaces.action import MoveServo


class ActionPlayer(Node):
    def __init__(self):
        super().__init__('action_player')
        self.client = ActionClient(self, MoveServo, 'move_servo')
        self.preempt_name = None
        self.current_handle = None
        self.create_subscription(String, '/servo/preempt', self.on_preempt, 10)

    def on_preempt(self, msg):
        name = msg.data.strip()
        if name:
            self.get_logger().warn(f'收到抢占请求 -> {name}')
            self.preempt_name = name

    def play_step(self, step):
        step_name = step.get('name', 'unnamed')
        dur = step.get('duration_ms', 800)
        servos = step['servos']

        g = MoveServo.Goal()
        g.duration_ms = dur
        if len(servos) == 1:
            g.id = servos[0]['id']
            g.pwm = servos[0]['pwm']
            show = f'id={g.id} pwm={g.pwm}'
        else:
            g.ids = [x['id'] for x in servos]
            g.pwms = [x['pwm'] for x in servos]
            show = f'ids={list(g.ids)} pwms={list(g.pwms)}'

        self.get_logger().info(f'[SEND]   {time.time():.3f} step={step_name} {show} dur={dur}')

        fut = self.client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('[REJECT] goal 被拒绝')
            return 'fail'

        self.current_handle = handle
        rf = handle.get_result_async()

        # 边等边查抢占
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.preempt_name:
                self.get_logger().warn(f'抢占：放弃等待 step={step_name}')
                self.current_handle = None
                return 'preempt'

        self.current_handle = None
        res = rf.result().result
        self.get_logger().info(
            f'[RESULT] {time.time():.3f} success={res.success} '
            f'final={res.final_pwm} msg={res.message}')
        return 'done' if res.success else 'fail'

    def play_file(self, path):
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        name = data.get('name', os.path.basename(path))
        steps = data.get('steps', [])
        self.get_logger().info(f'开始播放: {name}，共 {len(steps)} 个 step')

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('找不到 move_servo 服务端')
            return 'fail'

        t0 = time.time()
        for i, step in enumerate(steps, 1):
            if self.preempt_name:
                return 'preempt'
            self.get_logger().info('')
            self.get_logger().info(f'===== step {i}/{len(steps)}: {step.get("name", "")} =====')
            st = self.play_step(step)
            if st != 'done':
                return st

        self.get_logger().info('')
        self.get_logger().info(f'整套动作完成，耗时 {time.time() - t0:.2f}s')
        return 'done'


def resolve_path(arg):
    if os.path.isabs(arg):
        return arg
    if arg.endswith('.yaml') and os.path.exists(arg):
        return os.path.abspath(arg)
    share = get_package_share_directory('cquick_servo')
    return os.path.join(share, 'config', 'actions', arg + '.yaml')


def main(args=None):
    rclpy.init(args=args)
    node = ActionPlayer()

    arg = sys.argv[1] if len(sys.argv) > 1 else 'sync_demo'
    path = resolve_path(arg)

    while True:
        node.get_logger().info(f'动作文件: {path}')
        status = node.play_file(path)
        if status == 'preempt' and node.preempt_name:
            name = node.preempt_name
            node.preempt_name = None
            path = resolve_path(name)
            node.get_logger().warn(f'切换到新动作: {name}')
            continue
        break

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
