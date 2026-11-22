#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self.client = ActionClient(self, MoveServo, 'move_servo')
        self.client.wait_for_server()
        self.get_logger().info('客户端就绪。输入格式: <id> <pwm> <duration_ms>  回车发送')

    def send_goal(self, sid, pwm, dur):
        goal = MoveServo.Goal()
        goal.command.id = sid
        goal.command.pwm = pwm
        goal.command.duration_ms = dur
        self.get_logger().info(f'发送 goal: id={sid} pwm={pwm} t={dur}')
        self.future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        self.future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, msg):
        f = msg.feedback
        self.get_logger().info(
            f'反馈: current_pwm={f.current_pwm} progress={f.progress:.2f} status={f.status}')

    def goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('Goal 被拒绝')
            return
        self.get_logger().info('Goal 已接受，等待结果...')
        self.result_future = gh.get_result_async()
        self.result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        r = future.result().result
        self.get_logger().info(
            f'结果: success={r.success} code={r.error_code} msg={r.message}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            try:
                line = input('> ')
                parts = line.strip().split()
                if len(parts) == 3:
                    node.send_goal(int(parts[0]), int(parts[1]), int(parts[2]))
                else:
                    print('格式: id pwm duration_ms')
            except EOFError:
                break
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()