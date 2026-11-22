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
        self.client = ActionClient(self, MoveServo, 'move_servo')
        
        self.get_logger().info("启动中，正在等服务端...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("启动失败：找不到服务端")
            return
        self.get_logger().info("连接服务端成功")

    # 反馈打印
    def my_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"{fb.progress} 目前位置{fb.current_pwm}")

    # 执行yaml
    def play_yaml(self, filepath): 
        self.get_logger().info("开始读取动作文件: " + filepath)
        
        # 读取filepath
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error("读 yaml 报错了: " + str(e))
            return

        # 分步执行1 2 3 
        steps = data.get('steps', [])
        
        for step in steps:
            step_num = step.get('step')
            joints = step.get('joints', [])
            self.get_logger().info("--- 开始执行 Step " + str(step_num) + " ---")
            
            futures = [] #存future
            # 把当前 step 里的所有舵机指令一起发出去
            for joint in joints:
                goal = MoveServo.Goal()
                goal.id = joint['id']
                goal.pwm = joint['pwm']
                goal.duration_ms = joint['duration_ms']
                
                self.get_logger().info("准备发送: 舵机" + str(goal.id) + ", 目标位置" + str(goal.pwm))
                
                future = self.client.send_goal_async(goal, feedback_callback=self.my_feedback_callback)
                futures.append(future)
            
            for future in futures:
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if not future.done():
                    self.get_logger().error("等太久了，服务端无应答！")
                    continue
                    
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().warn("指令被拒绝了！可能是 PWM 超限了。")
                    continue
                
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result().result
                
                if result.success:
                    self.get_logger().info("搞定！最终位置: " + str(result.final_pwm))
                else:
                    self.get_logger().warn("失败了！原因: " + result.message)

        self.get_logger().info("动作执行完毕！")

def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    
    # 使用 ROS 2 标准参数声明，设置默认的 yaml 路径
    pkg_dir = get_package_share_directory('cquick_servo')
    default_yaml = os.path.join(pkg_dir, 'config', 'actions', 'stand_up.yaml')
    node.declare_parameter('yaml_file', default_yaml)
    
    # 从参数服务器获取路径（这样 launch 文件里传的路径就会覆盖默认值）
    yaml_path = node.get_parameter('yaml_file').get_parameter_value().string_value

    node.play_yaml(yaml_path)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()