import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


class MoveClientNode(Node):
    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, 'move_servo')

    def send_goal(self, servo_id, pwm, duration_ms):
        self._client.wait_for_server()
        goal = MoveServo.Goal()
        goal.id = servo_id
        goal.pwm = pwm
        goal.duration_ms = duration_ms

        self.get_logger().info(f'发送目标: id={servo_id}, pwm={pwm}')
        send_future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('目标被服务端拒绝！')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'执行结果: success={result.success}, final_pwm={result.final_pwm}, msg={result.message}')

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback -> current_pwm={fb.current_pwm}, progress={fb.progress:.2f}')


def main(args=None):
    rclpy.init(args=args)
    client = MoveClientNode()
    for target in [1800, 1200, 1500]:
        client.send_goal(1, target, 1000)
        time.sleep(0.5)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()