#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self.client = ActionClient(self, MoveServo, 'move_servo')

    def feedback_cb(self, msg):
        f = msg.feedback
        self.get_logger().info(f'feedback pwm={f.current_pwm} progress={f.progress:.2f}')

    def send(self, sid, pwm, dur):
        self.client.wait_for_server()
        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = dur

        fut = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('goal 被拒绝')
            return

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        res = rf.result().result
        self.get_logger().info(
            f'result success={res.success} final_pwm={res.final_pwm} msg={res.message}')


def main(args=None):
    rclpy.init(args=args)
    c = MoveClient()
    sid = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    pwm = int(sys.argv[2]) if len(sys.argv) > 2 else 1700
    dur = int(sys.argv[3]) if len(sys.argv) > 3 else 1500
    c.send(sid, pwm, dur)
    c.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()