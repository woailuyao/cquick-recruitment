"""取消演示：发一个 5 秒的长动作，2 秒后取消，看 Result 是不是 canceled。

（这就是细则里「取消必须真的生效」的证据：取消后 Result.message == canceled，
 并且驱动会把舵机回中位 1500。）

  ros2 run cquick_servo cancel_demo -- --id 3 --pwm 2000 --duration 5000 --after 2
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class CancelDemo(Node):
    def __init__(self) -> None:
        super().__init__("cancel_demo")
        self._client = ActionClient(self, MoveServo, "move_servo")
        self._feedbacks = 0

    def _on_feedback(self, msg) -> None:
        self._feedbacks += 1
        fb = msg.feedback
        self.get_logger().info(f"feedback#{self._feedbacks} pwm={fb.pwm_current} progress={fb.progress:.2f}")

    def run(self, frame_id: int, pwm: int, duration_ms: int, cancel_after: float) -> int:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("等不到 /move_servo")
            return 2
        goal = MoveServo.Goal()
        goal.command.id = int(frame_id)
        goal.command.pwm = int(pwm)
        goal.command.duration_ms = int(duration_ms)
        goal.verify_on_finish = True

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("goal rejected")
            return 1

        deadline = time.monotonic() + cancel_after
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        print(f"{cancel_after:.1f}s 后请求取消…（已收到 {self._feedbacks} 条 feedback）")
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        result = wrapped.result if wrapped is not None else None
        print(
            "result: success=%s prad_actual=%d message=%s status=%s"
            % (
                result.success,
                result.prad_actual,
                result.message,
                wrapped.status if wrapped is not None else "?",
            )
        )
        return 0 if result is not None and result.message == "canceled" else 1


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="取消长动作演示")
    parser.add_argument("--id", type=int, default=3)
    parser.add_argument("--pwm", type=int, default=2000)
    parser.add_argument("--duration", type=int, default=5000)
    parser.add_argument("--after", type=float, default=2.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = CancelDemo()
    code = 1
    try:
        code = node.run(args.id, args.pwm, args.duration, args.after)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
