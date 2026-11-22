#!/usr/bin/env python3
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    def __init__(self, yaml_path):
        super().__init__('sequence_player')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self.yaml_path = yaml_path
        self.pending = 0
        self.step_done = False

    def play(self):
        with open(self.yaml_path, 'r') as f:
            steps = yaml.safe_load(f).get('steps', [])
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server 没起来！')
            return

        for i, step in enumerate(steps):
            joints = step.get('joints', [])
            self.get_logger().info(f'=== step {i+1}: {step.get("name", "")} ===')
            self.pending = len(joints)
            self.step_done = False
            for j in joints:
                self._send_goal(j['id'], j['pwm'], j.get('duration_ms', 800))
            while not self.step_done:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(f'step {i+1} 完成 @ {time.time():.3f}')
        self.get_logger().info('动作组全部完成')

    def _send_goal(self, sid, pwm, duration):
        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = duration
        self._client.send_goal_async(goal).add_done_callback(
            lambda f, sid=sid: self._goal_cb(f, sid))

    def _goal_cb(self, future, sid):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f'id={sid} goal 被拒')
            self._one_done()
            return
        gh.get_result_async().add_done_callback(
            lambda f, sid=sid: self._result_cb(f, sid))

    def _result_cb(self, future, sid):
        r = future.result().result
        self.get_logger().info(f'id={sid} result: success={r.success}, msg={r.message}')
        self._one_done()

    def _one_done(self):
        self.pending -= 1
        if self.pending <= 0:
            self.step_done = True


def main(args=None):
    rclpy.init(args=args)
    import sys
    if len(sys.argv) < 2:
        print('用法: ros2 run cquick_servo sequence_player <yaml路径>')
        return
    node = SequencePlayer(sys.argv[1])
    node.play()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
