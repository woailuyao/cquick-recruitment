#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')

        self.declare_parameter('action_file', '')
        path = self.get_parameter('action_file').value
        if not path:
            self.get_logger().error('必须用 -p action_file:=xxx.yaml 指定动作文件')
            raise SystemExit(1)

        with open(path, 'r', encoding='utf-8') as f:
            self.data = yaml.safe_load(f)

        self.client = ActionClient(self, MoveServo, 'move_servo')
        self.client.wait_for_server()
        self.run()

    def feedback_cb(self, msg):
        f = msg.feedback
        self.get_logger().info(f'  feedback pwm={f.current_pwm} progress={f.progress:.2f}')

    def run(self):
        steps = self.data.get('steps', [])
        self.get_logger().info(
            f"动作组: {self.data.get('name', '未命名')}，共 {len(steps)} 步")

        for idx, step in enumerate(steps, 1):
            self.get_logger().info(f"--- 第 {idx} 步: {step.get('name', '')} ---")

            handles = []
            for g in step['goals']:
                goal = MoveServo.Goal()
                goal.id = int(g['id'])
                goal.pwm = int(g['pwm'])
                goal.duration_ms = int(g['duration_ms'])

                fut = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
                rclpy.spin_until_future_complete(self, fut)
                gh = fut.result()
                if gh is None or not gh.accepted:
                    self.get_logger().error(f"  被拒绝: id={goal.id} pwm={goal.pwm}")
                else:
                    handles.append(gh)

            # 关键：等本步所有 result 回来再进下一步（不用 sleep）
            for gh in handles:
                rf = gh.get_result_async()
                rclpy.spin_until_future_complete(self, rf)
                res = rf.result().result
                self.get_logger().info(
                    f'  result success={res.success} final={res.final_pwm} msg={res.message}')

        self.get_logger().info('动作组执行完毕')


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()