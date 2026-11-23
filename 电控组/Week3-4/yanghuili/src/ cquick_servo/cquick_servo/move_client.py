
"""Command-line Action client for a single CQuick servo movement."""

from __future__ import annotations

import argparse
import sys

from action_msgs.msg import GoalStatus
from cquick_servo_interfaces.action import MoveServo
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class MoveServoClient(Node):
    def __init__(self) -> None:
        super().__init__("move_servo_client")
        self.client = ActionClient(self, MoveServo, "/move_servo")

    def feedback_callback(self, message, servo_id: int) -> None:
        feedback = message.feedback
        self.get_logger().info(
            f"FEEDBACK id={servo_id}, "
            f"pwm={feedback.current_pwm}, "
            f"progress={feedback.progress:.2f}"
        )

    def send_and_wait(
        self,
        servo_id: int,
        pwm: int,
        duration_ms: int,
    ) -> bool:
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_servo is unavailable")
            return False

        goal = MoveServo.Goal()
        goal.id = int(servo_id)
        goal.pwm = int(pwm)
        goal.duration_ms = int(duration_ms)

        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=lambda message: self.feedback_callback(
                message,
                servo_id,
            ),
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "Goal rejected; check servo_ids and pwm limits"
            )
            return False

        self.get_logger().info(
            f"Goal accepted: {goal_handle.goal_id.uuid}"
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error("No result received")
            return False

        result = wrapped_result.result
        status_name = STATUS_NAMES.get(
            wrapped_result.status,
            "UNKNOWN",
        )
        self.get_logger().info(
            f"RESULT status={status_name}, "
            f"success={result.success}, "
            f"final_pwm={result.final_pwm}, "
            f"message={result.message}"
        )
        return (
            bool(result.success)
            and wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        )


def parse_arguments():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(
        description="Send one MoveServo Action goal."
    )
    parser.add_argument(
        "--id",
        dest="servo_id",
        type=int,
        required=True,
    )
    parser.add_argument("--pwm", type=int, required=True)
    parser.add_argument(
        "--duration",
        dest="duration_ms",
        type=int,
        default=1000,
    )
    parsed = parser.parse_args(args[1:])

    if not 1 <= parsed.servo_id <= 254:
        parser.error("--id must be in 1..254")
    if not 0 <= parsed.pwm <= 65535:
        parser.error("--pwm must be in 0..65535")
    if not 0 <= parsed.duration_ms <= 60000:
        parser.error("--duration must be in 0..60000")
    return parsed


def main(args=None) -> None:
    parsed = parse_arguments()
    rclpy.init(args=args)
    node = MoveServoClient()
    try:
        ok = node.send_and_wait(
            parsed.servo_id,
            parsed.pwm,
            parsed.duration_ms,
        )
        exit_code = 0 if ok else 1
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
