import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from cquick_servo_interfaces.action import MoveServo


class MoveClientNode(Node):
    def __init__(self):
        super().__init__('move_client')
        self._action_client = ActionClient(self, MoveServo, 'move_servo')

    def send_goal(self, servo_id, target_pwm, duration_ms=800):
        goal_msg = MoveServo.Goal()
        goal_msg.id = servo_id
        goal_msg.target_pwm = target_pwm
        goal_msg.duration_ms = duration_ms

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"id={feedback_msg.feedback.current_pwm and 1} pwm={feedback.current_pwm} progress={feedback.progress:.2f}"
        )

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal 被拒绝")
            return

        self.get_logger().info("Goal 已接受，开始执行")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"执行成功，最终位置：{result.final_pwm}")
        else:
            self.get_logger().info(f"执行失败：{result.error_msg}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = MoveClientNode()

    # 默认演示：0号舵机转到1500，用时1000ms
    client.send_goal(servo_id=0, target_pwm=1500, duration_ms=1000)

    rclpy.spin(client)


if __name__ == '__main__':
    main()

