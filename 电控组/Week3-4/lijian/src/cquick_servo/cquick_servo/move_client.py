import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__("move_client")
        self._client = ActionClient(self, MoveServo, "move_servo")

    def send(self, servo_id, pwm, duration_ms):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("action server 不在线")
            return False

        goal = MoveServo.Goal()
        goal.id = servo_id
        goal.pwm = pwm
        goal.duration_ms = duration_ms

        self.get_logger().info(f"发送 goal: id={servo_id} pwm={pwm} duration={duration_ms}")
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("goal 被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"result: success={result.success} final_pwm={result.final_pwm} message={result.message}")
        return result.success

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"feedback: current_pwm={fb.current_pwm} progress={fb.progress:.2f}")


def main():
    rclpy.init()
    node = MoveClient()
    if len(sys.argv) >= 4:
        sid = int(sys.argv[1])
        pwm = int(sys.argv[2])
        dur = int(sys.argv[3])
    else:
        sid, pwm, dur = 1, 1700, 1000
    ok = node.send(sid, pwm, dur)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
