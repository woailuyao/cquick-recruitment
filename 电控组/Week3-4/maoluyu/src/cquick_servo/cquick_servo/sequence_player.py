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
        # 创建Action客户端，绑定/move_servo
        self._action_client = ActionClient(self, MoveServo, '/move_servo')
        self.sequence = []
        self.current_step_idx = 0
        self.get_logger().info("动作序列播放器启动")

    def load_yaml(self, yaml_name:str):
        # 从share/cquick_servo/config/actions/加载yaml
        pkg_share = get_package_share_directory("cquick_servo")
        yaml_path = os.path.join(pkg_share, "config", "actions", yaml_name)
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        self.sequence = data["action_sequence"]
        self.current_step_idx = 0
        self.get_logger().info(f"加载动作文件：{yaml_name}，一共{len(self.sequence)}步")

    def send_next_goal(self):
        if self.current_step_idx >= len(self.sequence):
            self.get_logger().info("✅ 全部动作序列执行完成！")
            return
        # 取出当前这一步参数
        step = self.sequence[self.current_step_idx]
        goal_msg = MoveServo.Goal()
        goal_msg.id = step["id"]
        goal_msg.pwm = step["pwm"]
        goal_msg.duration_ms = step["duration_ms"]

        self.get_logger().info(f"发送第{self.current_step_idx+1}步：id={step['id']}, pwm={step['pwm']}, time={step['duration_ms']}")
        # 等待服务上线
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ 当前动作被拒绝，序列终止")
            return
        # 等待动作执行结果
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        res = future.result().result
        if res.success:
            self.get_logger().info(f"✅ 第{self.current_step_idx+1}步执行成功，final_pwm={res.final_pwm}")
            self.current_step_idx += 1
            # 延时可选，防止舵机来不及稳定，也可以不加
            self.send_next_goal()
        else:
            self.get_logger().error(f"❌ 第{self.current_step_idx+1}步失败，序列停止：{res.message}")

def main(args=None):
    rclpy.init(args=args)
    player = SequencePlayer()
    # ========== 切换这里的文件名，就能换动作 stand_up.yaml / wave.yaml ==========
    player.load_yaml("stand_up.yaml")
    player.send_next_goal()
    rclpy.spin(player)
    player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
