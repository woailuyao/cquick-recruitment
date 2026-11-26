#!/usr/bin/env python3
"""用法：
    move_client --id 1 --pwm 1800 --duration 1000
    move_client --id 1 --sweep 1200,1800,1500 --duration 1200
    move_client --id 1 --pwm 2000 --duration 8000 --cancel-after 1.5
"""
import argparse
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo

ACTION_NAME = 'move_servo'


class MoveClient(Node):

    def __init__(self):
        super().__init__('servo_client')
        self._client = ActionClient(self, MoveServo, ACTION_NAME)
        self._active_id = 0

    def wait_for_server(self, timeout=5.0):
        self.get_logger().info(f'等待 Action 服务 {ACTION_NAME} ...')
        return self._client.wait_for_server(timeout_sec=timeout)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'feedback: id={self._active_id} pwm={fb.current_pwm} progress={fb.progress:.2f}')

    def send_goal(self, servo_id, pwm, duration_ms):
        goal = MoveServo.Goal()
        goal.id = servo_id
        goal.pwm = pwm
        goal.duration_ms = duration_ms
        self._active_id = servo_id
        self.get_logger().info(f'发送 goal: id={servo_id} pwm={pwm} duration={duration_ms}')

        send_future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝（pwm 越界 / ID 不在 servo_ids 中）')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'结果: 成功 final_pwm={result.final_pwm} message={result.message}')
        else:
            self.get_logger().warn(f'结果: 失败 final_pwm={result.final_pwm} message={result.message}')
        return result.success

    def send_goal_cancellable(self, servo_id, pwm, duration_ms, cancel_after):
        goal = MoveServo.Goal()
        goal.id = servo_id
        goal.pwm = pwm
        goal.duration_ms = duration_ms
        self._active_id = servo_id
        self.get_logger().info(
            f'发送长动作 goal: id={servo_id} pwm={pwm} duration={duration_ms}，'
            f'{cancel_after:.1f}s 后自动取消')

        send_future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝')
            return

        threading.Timer(cancel_after, lambda: self.cancel(goal_handle)).start()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f'结果: success={result.success} final_pwm={result.final_pwm} message={result.message}')

    def cancel(self, goal_handle):
        self.get_logger().warn('请求取消当前 goal ...')
        future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description='CQuick 舵机 Action 客户端')
    ap.add_argument('--id', type=int, default=1, help='舵机 ID')
    ap.add_argument('--pwm', type=int, default=1500, help='目标 PWM')
    ap.add_argument('--duration', type=int, default=1000, help='运动时间 ms')
    ap.add_argument('--sweep', type=str, default='',
                    help='逗号分隔 pwm 序列，如 1200,1800,1500（逐个等 result）')
    ap.add_argument('--cancel-after', type=float, default=0.0,
                    help='>0 时在该秒数后自动取消（演示取消）')
    return ap.parse_args(argv)


def main():
    rclpy.init()
    args = parse_args(remove_ros_args(args=sys.argv)[1:])
    node = MoveClient()
    if not node.wait_for_server():
        node.get_logger().error('Action 服务未就绪，退出')
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        if args.sweep:
            targets = [int(v.strip()) for v in args.sweep.split(',') if v.strip()]
            node.get_logger().info(f'动作序列: {targets}（逐个等 result 再发下一条）')
            for pwm in targets:
                node.send_goal(args.id, pwm, args.duration)
        elif args.cancel_after > 0:
            node.send_goal_cancellable(args.id, args.pwm, args.duration, args.cancel_after)
        else:
            node.send_goal(args.id, args.pwm, args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
