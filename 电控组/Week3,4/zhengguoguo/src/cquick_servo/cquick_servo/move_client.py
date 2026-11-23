#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo

class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._action_client = ActionClient(self, MoveServo, 'move_servo')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server found! Sending goal...')
        
        goal_msg = MoveServo.Goal()
        goal_msg.id = 1
        goal_msg.pwm = 1700
        goal_msg.duration_ms = 1000
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, msg={result.message}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: progress={feedback.progress:.2f}')

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
