#!/usr/bin/env python3
"""Action 客户端：发 goal / 打 feedback / 打 result，支持 Ctrl+C 与 --cancel-after 取消"""
import argparse
import signal
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):

    def __init__(self):
        super().__init__('move_client')
        self._ac = ActionClient(self, MoveServo, 'move_servo')
        self._sid = 0

    def _fb(self, msg):
        f = msg.feedback
        self.get_logger().info('feedback: id=%d pwm=%d progress=%.2f'
                               % (self._sid, f.current_pwm, f.progress))

    def _cancel(self, gh, why):
        self.get_logger().warn('%s → 请求取消' % why)
        cf = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cf)

    def run(self, sid, pwm, dur, cancel_after=0.0):
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server 不在线，先 ros2 launch cquick_servo servo.launch.py')
            return False
        self._sid = sid
        g = MoveServo.Goal()
        g.id = sid
        g.pwm = pwm
        g.duration_ms = dur
        self.get_logger().info('发送 goal: id=%d pwm=%d duration=%dms' % (sid, pwm, dur))

        fut = self._ac.send_goal_async(g, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn('goal 被拒绝：pwm 越界 / id 不在 servo_ids / duration 非法')
            return False

        self.get_logger().info('goal 已接受，等待 result（Ctrl+C 可取消）')
        rf = gh.get_result_async()
        t0 = time.time()
        try:
            while not rf.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                if cancel_after > 0 and (time.time() - t0) >= cancel_after:
                    self._cancel(gh, '已过 %.1f 秒' % cancel_after)
                    cancel_after = 0.0
        except KeyboardInterrupt:
            self._cancel(gh, 'Ctrl+C')

        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        res = rf.result().result
        self.get_logger().info('result: success=%s final_pwm=%d message=%s'
                               % (res.success, res.final_pwm, res.message))
        return bool(res.success)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--id', type=int, default=1)
    ap.add_argument('--pwm', type=int, default=1500)
    ap.add_argument('--duration', type=int, default=1000)
    ap.add_argument('--cancel-after', type=float, default=0.0,
                    help='秒；>0 表示到点自动取消（录屏演示用）')
    a = ap.parse_args()

    rclpy.init()
    # rclpy 默认把 SIGINT 变成 context 关闭，这里恢复 Python 默认行为，Ctrl+C 才会抛 KeyboardInterrupt
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = MoveClient()
    try:
        ok = node.run(a.id, a.pwm, a.duration, a.cancel_after)
        node.get_logger().info('exit ok=%s' % ok)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
