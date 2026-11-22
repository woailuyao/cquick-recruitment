#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self.current_id = 0
        self.get_logger().info('客户端已启动，输入格式: id pwm duration_ms（如 1 1700 800）')

    def send_goal(self, sid, pwm, duration_ms=0):
        self.current_id = sid
        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = duration_ms

        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server 没起来！')
            return

        self.get_logger().info(f'发送 goal: id={sid} pwm={pwm} duration={duration_ms}')
        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal 被拒绝了！')
            return
        self.get_logger().info('Goal 被接受，执行中...')
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'id={self.current_id} pwm={fb.current_pwm} progress={fb.progress:.2f}')

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(
            f'结果: success={result.success}, final_pwm={result.final_pwm}, message={result.message}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()

    import threading
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.daemon = True
    spinner.start()

    try:
        while rclpy.ok():
            try:
                line = input('> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if line.lower() in ('q', 'quit', 'exit'):
                break
            if not line:
                continue
            parts = line.split()
            try:
                sid = int(parts[0])
                pwm = int(parts[1])
                dur = int(parts[2]) if len(parts) > 2 else 0
                node.send_goal(sid, pwm, dur)
            except (ValueError, IndexError):
                print('格式错误，例：1 1700 800')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
