import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cquick_servo_interfaces.action import MoveServo
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import time


class SequencePlayerNode(Node):
    def __init__(self):
        super().__init__('sequence_player')

        # 1. 声明参数：动作文件名，启动时外部传入
        self.declare_parameter('action_file', 'reset.yaml')
        action_file_name = self.get_parameter('action_file').value

        # 2. 创建 Action 客户端，复用标准接口
        self._action_client = ActionClient(self, MoveServo, 'move_servo')
        self.get_logger().info("等待舵机 Action 服务端上线...")
        self._action_client.wait_for_server()
        self.get_logger().info("服务端连接成功")

        # 3. 加载动作 yaml 文件
        self.action_data = self._load_action_yaml(action_file_name)
        if not self.action_data:
            self.get_logger().error("动作文件加载失败，退出")
            return

        action_name = self.action_data.get('action_name', '未命名动作')
        step_count = len(self.action_data.get('steps', []))
        self.get_logger().info(f"加载动作成功：{action_name}，共 {step_count} 步")

        # 4. 自动开始播放
        self._play_sequence()

        # 5. 播放完成提示
        self.get_logger().info("动作组全部执行完毕")

    def _load_action_yaml(self, file_name):
        """加载动作 yaml，通过包路径查找，无硬编码相对路径"""
        try:
            pkg_share = get_package_share_directory('cquick_servo')
            file_path = os.path.join(pkg_share, 'config', 'actions', file_name)

            if not os.path.exists(file_path):
                self.get_logger().error(f"动作文件不存在: {file_path}")
                return None

            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            return data

        except Exception as e:
            self.get_logger().error(f"加载动作文件失败: {e}")
            return None

    def _play_sequence(self):
        """核心播放逻辑：步与步严格串行，靠 result 同步，失败立即终止"""
        steps = self.action_data.get('steps', [])
        total_step = len(steps)

        for idx, step in enumerate(steps):
            step_name = step.get('step_name', f'第{idx+1}步')
            joints = step.get('joints', [])

            # 打印 step 开始日志（带时间戳，验收留证）
            start_time = time.time()
            self.get_logger().info(
                f"[Step {idx+1}/{total_step}] 开始: {step_name} | 时间: {time.strftime('%H:%M:%S', time.localtime())}"
            )

            # 执行当前步：所有关节并行下发，等待全部返回 result
            all_success = self._execute_step(joints)

            # 打印 step 结束日志
            end_time = time.time()
            self.get_logger().info(
                f"[Step {idx+1}/{total_step}] 结束: {step_name} | 耗时: {end_time-start_time:.2f}s | 全部成功: {all_success}"
            )

            # 任意一步失败，立即终止整个动作组
            if not all_success:
                self.get_logger().error(f"动作组执行终止：第 {idx+1} 步存在关节执行异常")
                return

            # 步间极短间隔，避免总线瞬间拥堵
            time.sleep(0.05)

    def _execute_step(self, joints):
        """执行单步：所有关节并发发指令，等待全部返回结果，返回是否全部成功"""
        goal_futures = []

        # 第一轮：所有关节全部异步下发 goal，不等待，实现真正并行
        for joint in joints:
            sid = joint['id']
            target_pwm = joint['target_pwm']
            duration_ms = joint['duration_ms']

            goal_msg = MoveServo.Goal()
            goal_msg.id = sid
            goal_msg.target_pwm = target_pwm
            goal_msg.duration_ms = duration_ms

            future = self._action_client.send_goal_async(goal_msg)
            goal_futures.append((sid, future))

        # 第二轮：等待所有 goal 响应，收集 result 句柄
        result_futures = []
        for sid, goal_future in goal_futures:
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().warn(f"  舵机 ID={sid} 指令被服务端拒绝")
                return False

            result_future = goal_handle.get_result_async()
            result_futures.append((sid, result_future))

        # 第三轮：等待所有舵机执行完毕，校验结果
        all_success = True
        for sid, result_future in result_futures:
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            # 打印单关节完成日志（带时间戳，证明 result 同步）
            self.get_logger().info(
                f"  舵机 ID={sid} 完成 | success={result.success} | final_pwm={result.final_pwm} | 时间: {time.strftime('%H:%M:%S', time.localtime())}"
            )

            if not result.success:
                all_success = False

        return all_success


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayerNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

