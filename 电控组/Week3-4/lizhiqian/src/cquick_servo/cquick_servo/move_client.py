#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')

        self.declare_parameter('id', 1)
        self.declare_parameter('pwm', 1500)
        self.declare_parameter('duration_ms', 1000)

        self._client = ActionClient(self, MoveServo, 'move_servo')

    def send_goal(self):
        goal = MoveServo.Goal()
        goal.id = self.get_parameter('id').value
        goal.pwm = self.get_parameter('pwm').value
        goal.duration_ms = self.get_parameter('duration_ms').value

        self.get_logger().info(
            f'发送 goal: id={goal.id} pwm={goal.pwm} duration_ms={goal.duration_ms}'
        )

        self._client.wait_for_server()

        self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('goal 被拒绝')
            rclpy.shutdown()
            return

        self.get_logger().info('goal 已接受')
        goal_handle.get_result_async().add_done_callback(
            self.get_result_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'id={self.get_parameter("id").value} '
            f'pwm={feedback.current_pwm} progress={feedback.progress:.2f}'
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'result: success={result.success}, '
            f'message={result.message}, final_pwm={result.final_pwm}'
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
