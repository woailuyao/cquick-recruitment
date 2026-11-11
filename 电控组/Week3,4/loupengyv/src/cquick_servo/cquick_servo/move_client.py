#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self.declare_parameter('id', 1)
        self.declare_parameter('pwm', 1700)
        self.declare_parameter('duration_ms', 1000)
        self.client = ActionClient(self, MoveServo, 'move_servo')

    def feedback_cb(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            'feedback: id=%d pwm=%d progress=%.2f'
            % (self.sid, fb.current_pwm, fb.progress))

    def send_once(self):
        self.sid = self.get_parameter('id').value
        pwm = self.get_parameter('pwm').value
        dur = self.get_parameter('duration_ms').value

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('等不到 /move_servo 服务端，先用 launch 起来')
            return

        goal = MoveServo.Goal()
        goal.id = self.sid
        goal.pwm = pwm
        goal.duration_ms = dur

        self.get_logger().info(
            '发送 goal: id=%d pwm=%d duration_ms=%d' % (self.sid, pwm, dur))

        send_future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            '完成: success=%s final_pwm=%d message=%s'
            % (result.success, result.final_pwm, result.message))


def main():
    rclpy.init()
    node = MoveClient()
    try:
        node.send_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
