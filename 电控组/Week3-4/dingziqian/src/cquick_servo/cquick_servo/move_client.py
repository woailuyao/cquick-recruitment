import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__("move_client")
        self.client = ActionClient(self, MoveServo, "/servo/move")

    def send_goal(self, servo_id, pwm, duration_ms):
        self.client.wait_for_server()

        goal = MoveServo.Goal()
        goal.id = servo_id
        goal.pwm = pwm
        goal.duration_ms = duration_ms

        future = self.client.send_goal_async(
            goal, feedback_callback=self.on_feedback
        )
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("目标被拒绝")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"完成：success={result.success}, "
            f"final_pwm={result.final_pwm}, message={result.message}"
        )

    def on_feedback(self, message):
        feedback = message.feedback
        self.get_logger().info(
            f"反馈：id={feedback.id} "
            f"current_pwm={feedback.current_pwm} "
            f"progress={feedback.progress:.2f}"
        )


def main(args=None):
    parser = argparse.ArgumentParser(description="发送舵机 Action 目标")
    parser.add_argument("--id", type=int, required=True, dest="servo_id")
    parser.add_argument("--pwm", type=int, required=True)
    parser.add_argument("--duration-ms", type=int, default=0)
    options = parser.parse_args(remove_ros_args(args=args)[1:])

    if not 1 <= options.servo_id <= 254:
        parser.error("--id 必须在 1～254 之间")
    if not 0 <= options.pwm <= 65535:
        parser.error("--pwm 必须在 0～65535 之间")
    if not 0 <= options.duration_ms <= 65535:
        parser.error("--duration-ms 必须在 0～65535 之间")

    rclpy.init(args=args)
    node = MoveClient()
    try:
        node.send_goal(
            options.servo_id, options.pwm, options.duration_ms
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
