import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from cquick_servo_interfaces.action import MoveServo

class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self._action_client = ActionClient(self, MoveServo, 'move_servo')
        self.get_logger().info('动作组播放器已启动，等待服务端...')
        self._action_client.wait_for_server()

        self.steps = []
        self.current_step_idx = 0
        self.current_action_idx = 0

    def load_and_play(self, file_path):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 解析 YAML 的层次结构
        for step_name, actions in data['stand_up']['steps'].items():
            self.steps.append((step_name, actions))
        
        self.get_logger().info(f'成功加载动作组，共 {len(self.steps)} 步。开始播放...')
        self.play_next_step()

    def play_next_step(self):
        if self.current_step_idx < len(self.steps):
            step_name, actions = self.steps[self.current_step_idx]
            self.get_logger().info(f'--- 开始执行 {step_name} ---')
            self.current_action_idx = 0
            self.send_next_action(actions)
        else:
            self.get_logger().info('🎉 所有动作组播放完毕！')
            rclpy.shutdown()

    def send_next_action(self, actions):
        if self.current_action_idx < len(actions):
            action = actions[self.current_action_idx]
            goal_msg = MoveServo.Goal()
            goal_msg.id = action['id']
            goal_msg.pwm = action['pwm']
            goal_msg.duration_ms = action['duration_ms']
            
            self.get_logger().info(f'  发送: id={goal_msg.id}, pwm={goal_msg.pwm}')
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            # 当前 step 发完了，进入下一个 step
            self.current_step_idx += 1
            self.play_next_step()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal 被拒绝')
            return
        # 等待 Result 回调
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # 收到 Result 后，立刻发当前 step 的下一个动作
        result = future.result().result
        self.get_logger().info(f'  完成: {result.success}')
        self.current_action_idx += 1
        _, actions = self.steps[self.current_step_idx]
        self.send_next_action(actions)

def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    
    pkg_dir = get_package_share_directory('cquick_servo')
    yaml_file = os.path.join(pkg_dir, 'config', 'actions', 'stand_up.yaml')
    
    if os.path.exists(yaml_file):
        node.load_and_play(yaml_file)
        rclpy.spin(node) # 必须加 spin，否则回调进不来，动作不会往下走
    else:
        node.get_logger().error(f'找不到动作文件: {yaml_file}')

if __name__ == '__main__':
    main()
