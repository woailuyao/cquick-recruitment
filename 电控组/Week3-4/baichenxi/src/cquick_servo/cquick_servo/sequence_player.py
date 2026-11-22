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
        self.get_logger().info('等待 Action Server 启动...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server 已连接！')

    def send_goal(self, servo_id, pwm, duration_ms):
        goal_msg = MoveServo.Goal()
        goal_msg.command.id = servo_id
        goal_msg.command.pwm = pwm
        goal_msg.command.duration_ms = duration_ms

        self.get_logger().info(f'发送指令: ID={servo_id}, PWM={pwm}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        return True

    def play_yaml(self, yaml_file):
        pkg_dir = get_package_share_directory('cquick_servo')
        yaml_path = os.path.join(pkg_dir, 'config', 'actions', yaml_file)
        
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        for step in data['steps']:
            self.get_logger().info('--- 开始新步骤 ---')
            for cmd in step:
                self.send_goal(cmd['id'], cmd['pwm'], cmd['duration_ms'])
            self.get_logger().info('--- 步骤结束 ---')

def main(args=None):
    rclpy.init(args=args)
    player = SequencePlayer()
    player.play_yaml('stand_up.yaml') 
    player.destroy_node()
    rclpy.shutdown()
