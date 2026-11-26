#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐关节验证。用法：
  ros2 run cquick_servo joint_check
  python3 joint_check.py --ids 1,2 --delta 100
"""
import argparse
import sys
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo

ACTION_NAME = 'move_servo'
CENTER = 1500


def stamp():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


class JointChecker(Node):

    def __init__(self):
        super().__init__('joint_check')
        self._client = ActionClient(self, MoveServo, ACTION_NAME)

    def fetch_servo_ids(self):
        """从驱动节点读 servo_ids 参数；读不到返回 None。"""
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters, '/servo_driver/get_parameters')
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        req = GetParameters.Request()
        req.names = ['servo_ids']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None:
            return None
        vals = fut.result().values
        if not vals or not vals[0].integer_array_value:
            return None
        return list(vals[0].integer_array_value)

    def move(self, sid, pwm, duration_ms, tag):
        """发一条 goal 并等 result。返回 (ok, final_pwm, elapsed, message)。"""
        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = duration_ms

        t0 = time.monotonic()
        fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done():
            self.get_logger().error(f'[{stamp()}] id={sid} {tag} 等待接单超时')
            return False, 0, 0.0, 'wait_accepted_timeout'
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(
                f'[{stamp()}] id={sid} {tag} 被 rejected（ID 不在 servo_ids / pwm 越界？）')
            return False, 0, 0.0, 'rejected'

        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=30.0)
        elapsed = time.monotonic() - t0
        if not rfut.done():
            self.get_logger().error(f'[{stamp()}] id={sid} {tag} 等 result 超时')
            return False, 0, elapsed, 'result_timeout'

        res = rfut.result().result
        self.get_logger().info(
            f'[{stamp()}] id={sid} {tag:<10} pwm={pwm:<5} success={res.success} '
            f'final={res.final_pwm} 用时={elapsed:.2f}s msg={res.message}')
        return bool(res.success), int(res.final_pwm), elapsed, res.message

    def check_one(self, sid, delta, duration_ms, lo, hi):
        """一个舵机走四步，返回一行表格数据。"""
        plus = min(CENTER + delta, hi)
        minus = max(CENTER - delta, lo)
        self.get_logger().info(f'[{stamp()}] ===== 舵机 {sid} 开始（+{plus} / -{minus}）=====')

        rows = {}
        rows['center'] = self.move(sid, CENTER, duration_ms, '回中位')
        rows['plus'] = self.move(sid, plus, duration_ms, '正向')
        rows['minus'] = self.move(sid, minus, duration_ms, '反向')
        rows['back'] = self.move(sid, CENTER, duration_ms, '回中位')

        all_ok = all(r[0] for r in rows.values())
        return sid, plus, minus, rows, all_ok


def parse_args(argv):
    ap = argparse.ArgumentParser(description='整机逐关节验证（走 Action，等 result）')
    ap.add_argument('--ids', default='', help='逗号分隔，如 1,2,3；不填则读 servo_ids 参数')
    ap.add_argument('--delta', type=int, default=150, help='小幅正反转幅度（PWM），默认 150')
    ap.add_argument('--duration', type=int, default=1200, help='每步用时 ms，默认 1200')
    ap.add_argument('--min', type=int, default=900, help='安全下限，用来夹住 +/- 幅度')
    ap.add_argument('--max', type=int, default=2100, help='安全上限')
    ap.add_argument('--out', default='', help='把 markdown 表写到文件')
    return ap.parse_args(argv)


def main():
    args = parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = JointChecker()

    ids = [int(v) for v in args.ids.split(',') if v.strip()] if args.ids else node.fetch_servo_ids()
    if not ids:
        node.get_logger().error('拿不到舵机列表：驱动节点没起，或 servo_ids 参数为空')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.get_logger().info(f'[{stamp()}] 待验证舵机 {ids}，幅度 ±{args.delta}，每步 {args.duration}ms')
    if not node._client.wait_for_server(timeout_sec=8.0):
        node.get_logger().error('Action 服务未就绪（servo_driver 起了吗？）')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    results = []
    for sid in ids:
        results.append(node.check_one(sid, args.delta, args.duration, args.min, args.max))

    lines = [
        '',
        '## 逐关节验证（Action 逐个控制，参数来自 servo_ids）',
        '',
        f'时间：{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}　'
        f'幅度：±{args.delta}　每步：{args.duration} ms',
        '',
        '| ID | 回中位 final | 正向 final | 反向 final | 再回中位 final | 单步耗时 | 结论 |',
        '|---|---|---|---|---|---|---|',
    ]
    all_ok = True
    for sid, plus, minus, rows, ok in results:
        all_ok = all_ok and ok
        c, p, m, b = rows['center'], rows['plus'], rows['minus'], rows['back']
        avg = (c[2] + p[2] + m[2] + b[2]) / 4.0
        lines.append(
            f'| {sid} | {c[1]} | {p[1]} (+{plus}) | {m[1]} (-{minus}) | {b[1]} '
            f'| {avg:.2f}s | {"OK" if ok else "FAIL"} |')
    table = '\n'.join(lines)

    print(table)
    if args.out:
        with open(args.out, 'a', encoding='utf-8') as f:
            f.write(table + '\n')
        print(f'\n已追加到 {args.out}')

    if not all_ok:
        node.get_logger().error(f'[{stamp()}] 有舵机失败，看上面 message 定位（离线 / 越界 / 没到位）')
    else:
        node.get_logger().info(f'[{stamp()}] 全部 {len(ids)} 个舵机通过')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
