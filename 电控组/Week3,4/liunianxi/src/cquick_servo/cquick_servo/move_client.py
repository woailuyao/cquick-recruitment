"""Action 客户端：从命令行给一次 goal，打印 feedback 与 result。

用法：
  ros2 run cquick_servo move_client --id 1 --pwm 1700 --duration 800
  ros2 run cquick_servo move_client --id 1 --pwm 1700 --no-verify
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self) -> None:
        super().__init__("move_client")
        self._client = ActionClient(self, MoveServo, "move_servo")

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"id={self._req_id} pwm={fb.pwm_current} progress={fb.progress:.2f}"
        )

    def send(self, frame_id: int, pwm: int, duration_ms: int, verify: bool) -> int:
        self._req_id = frame_id
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("等不到 /move_servo 服务端（驱动节点没起来？）")
            return 2

        goal = MoveServo.Goal()
        goal.command.id = int(frame_id)
        goal.command.pwm = int(pwm)
        goal.command.duration_ms = int(duration_ms)
        goal.verify_on_finish = bool(verify)

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("goal rejected（超限 / ID 不在 servo_ids / duration 非法）")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        print(
            "result: success=%s prad_actual=%d message=%s"
            % (result.success, result.prad_actual, result.message)
        )
        return 0 if getattr(result, "success", False) else 1


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="给舵机发一条 MoveServo goal")
    parser.add_argument("--id", type=int, required=True, help="舵机 ID")
    parser.add_argument("--pwm", type=int, required=True, help="目标 PWM")
    parser.add_argument("--duration", type=int, default=800, help="用时 ms")
    parser.add_argument("--no-verify", action="store_true", help="不读 PRAD 校验")
    args = parser.parse_args(argv)

    rclpy.init()
    node = MoveClient()
    code = 1
    try:
        code = node.send(args.id, args.pwm, args.duration, not args.no_verify)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
