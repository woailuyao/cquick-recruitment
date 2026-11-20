#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_client —— Action Client 示例。

三种用法：
  1) 参数驱动发一条 goal（launch 里 demo:=true 走这条）
       ros2 run cquick_servo move_client --ros-args -p servo_id:=1 -p target_pwm:=1700
  2) 交互式：终端里敲 "id pwm [duration]"
       ros2 run cquick_servo move_client --ros-args -p interactive:=true
  3) 作为「心跳源」常驻：每 200ms 发一条 /servo/heartbeat，跑着它看门狗就不会跳闸，
     把它 Ctrl+C 掉，1 秒后驱动节点就会 PULK 卸力 —— 看门狗加分项就是这样演的。

注意它只发 goal，不直接碰串口，也不 import 任何串口库。
"""

from __future__ import annotations

import queue
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty

from cquick_servo_interfaces.action import MoveServo

ACTION_NAME = 'move_servo'
TOPIC_HEARTBEAT = '/servo/heartbeat'


class MoveClient(Node):
    def __init__(self) -> None:
        super().__init__('move_client')

        self.declare_parameter('servo_id', 1)
        self.declare_parameter('target_pwm', 1700)
        self.declare_parameter('duration_ms', 1000)
        self.declare_parameter('interactive', False)
        self.declare_parameter('heartbeat_period_ms', 200)
        self.declare_parameter('hold_after_goal', True)

        self._client = ActionClient(self, MoveServo, ACTION_NAME)
        self._heartbeat_pub = self.create_publisher(Empty, TOPIC_HEARTBEAT, 10)

        period = max(0.05, int(self.get_parameter('heartbeat_period_ms').value) / 1000.0)
        self.create_timer(period, self._publish_heartbeat)
        self._pending_id = int(self.get_parameter('servo_id').value)
        self._commands: 'queue.Queue[str]' = queue.Queue()

        if bool(self.get_parameter('interactive').value):
            self._start_stdin_reader()
            self.create_timer(0.1, self._drain_stdin)
            self.get_logger().info('交互模式：输入 "id pwm [duration_ms]"，例如 "1 1700 1000"')
        else:
            threading.Thread(target=self._send_initial_goal, daemon=True).start()

    # ------------------------------------------------------------ 心跳
    def _publish_heartbeat(self) -> None:
        self._heartbeat_pub.publish(Empty())

    # ------------------------------------------------------------ 发 goal
    def _send_initial_goal(self) -> None:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'等不到 Action Server /{ACTION_NAME}，先起 servo_driver')
            return
        self.send_goal(
            int(self.get_parameter('servo_id').value),
            int(self.get_parameter('target_pwm').value),
            int(self.get_parameter('duration_ms').value),
        )

    def send_goal(self, servo_id: int, pwm: int, duration_ms: int) -> None:
        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration_ms)
        self._pending_id = int(servo_id)
        self.get_logger().info(f'发 goal：id={servo_id} pwm={pwm} duration={duration_ms}ms')
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    # ------------------------------------------------------------ 回调
    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝（看驱动节点的日志，那里有原因）')
            return
        self.get_logger().info('goal 已接受，等 result')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        # 验收要求就是这个格式：id=1 pwm=1620 progress=0.62
        self.get_logger().info(
            f'feedback: id={self._pending_id} pwm={feedback.pwm_current} '
            f'progress={feedback.progress:.2f}')

    def _on_result(self, future) -> None:
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'result: 成功，readback={result.pwm_readback}（{result.message}）')
        else:
            self.get_logger().error(
                f'result: 失败，readback={result.pwm_readback}，原因：{result.message}')
        if not bool(self.get_parameter('hold_after_goal').value):
            self.get_logger().info('hold_after_goal=false，退出')
            rclpy.shutdown()

    # ------------------------------------------------------------ 交互输入
    def _start_stdin_reader(self) -> None:
        def reader() -> None:
            for line in sys.stdin:
                self._commands.put(line.strip())

        threading.Thread(target=reader, daemon=True).start()

    def _drain_stdin(self) -> None:
        while True:
            try:
                line = self._commands.get_nowait()
            except queue.Empty:
                return
            if not line:
                continue
            parts = line.replace(',', ' ').split()
            if len(parts) < 2:
                self.get_logger().warn(f'看不懂：{line!r}，格式 "id pwm [duration_ms]"')
                continue
            try:
                servo_id, pwm = int(parts[0]), int(parts[1])
                duration = int(parts[2]) if len(parts) > 2 else 1000
            except ValueError:
                self.get_logger().warn(f'参数不是整数：{line!r}')
                continue
            self.send_goal(servo_id, pwm, duration)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
