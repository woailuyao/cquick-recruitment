#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self._action_client = ActionClient(self, MoveServo, 'move_servo')
        self.steps = []
        self.current_step = 0

    def load_yaml(self, file_name):
        pkg_share = get_package_share_directory('cquick_servo')
        path = os.path.join(pkg_share, 'config', 'actions', file_name)
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            self.steps = data['steps']
            self.get_logger().info(f'成功加载动作组: {data["action_name"]}')
            self.play_next_step()

    def play_next_step(self):
        if self.current_step >= len(self.steps):
            self.get_logger().info('所有动作组执行完毕！')
            rclpy.shutdown()
            return

        step = self.steps[self.current_step]
        commands = step['commands']
        self.get_logger().info(f'--- 开始执行步骤: {step["step_name"]} (共 {len(commands)} 个关节) ---')

        # 记录这一步需要发送多少个目标，以及已经完成了多少个
        self.goals_to_wait = len(commands)
        self.goals_completed = 0

        for cmd in commands:
            goal_msg = MoveServo.Goal()
            goal_msg.id = cmd['id']
            goal_msg.pwm = cmd['pwm']
            goal_msg.duration_ms = cmd['duration_ms']

            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝！')
            self.goals_completed += 1
            self.check_step_completed()
            return
        
        # 这里非常重要：发完 Goal 之后，立马去拿 Result，不要阻塞
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info(f'收到 Result: 成功={result.success}')
        self.goals_completed += 1
        self.check_step_completed()

    def check_step_completed(self):
        # 如果当前 step 里的所有舵机都转完了，才进入下一个 step
        if self.goals_completed >= self.goals_to_wait:
            self.current_step += 1
            self.play_next_step()

def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    node._action_client.wait_for_server()
    
    # 读取 launch 传进来的 yaml 参数
    node.declare_parameter('yaml_file', 'stand_up.yaml')
    yaml_file = node.get_parameter('yaml_file').value
    node.load_yaml(yaml_file)
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()