import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from cquick_servo_interfaces.action import MoveServo


class SequencePlayerNode(Node):
    def __init__(self):
        super().__init__('sequence_player')
        pkg_share = get_package_share_directory('cquick_servo')
        default_yaml = os.path.join(pkg_share, 'config', 'actions', 'stand_neutral.yaml')

        self.declare_parameter('action_file', default_yaml)
        self.declare_parameter('repeat', 1)

        self._client = ActionClient(self, MoveServo, 'move_servo')

    def run_sequence(self):
        action_file = self.get_parameter('action_file').value
        repeat = self.get_parameter('repeat').value

        if not os.path.exists(action_file):
            self.get_logger().error(f'找不到动作文件: {action_file}')
            return

        with open(action_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        steps = data.get('steps', [])
        action_name = data.get('action_name', os.path.basename(action_file))

        self.get_logger().info('等待 /move_servo Action 服务端上线...')
        self._client.wait_for_server()

        for run_idx in range(1, repeat + 1):
            self.get_logger().info(f'=== 开始播放动作组 [{action_name}] 第 {run_idx}/{repeat} 遍 ===')
            for step_idx, step in enumerate(steps, start=1):
                joints = step.get('joints', [])
                self.get_logger().info(f'[Step {step_idx}] 并发下发 {len(joints)} 个关节目标')

                # 1. 同一步内多关节同时发送 Goal
                goal_futures = []
                for j in joints:
                    goal = MoveServo.Goal()
                    goal.id = int(j['id'])
                    goal.pwm = int(j['pwm'])
                    goal.duration_ms = int(j['duration_ms'])
                    send_future = self._client.send_goal_async(goal)
                    goal_futures.append((goal.id, send_future))

                # 2. 等待本步所有 Goal 被接收，收集 Result Future
                result_futures = []
                for sid, send_future in goal_futures:
                    rclpy.spin_until_future_complete(self, send_future)
                    goal_handle = send_future.result()
                    if not goal_handle or not goal_handle.accepted:
                        self.get_logger().error(f'[Step {step_idx}] 舵机 {sid} Goal 被拒绝，中止播放')
                        return
                    result_futures.append((sid, goal_handle.get_result_async()))

                # 3. 严格等待本步所有 Result 返回后，才进入下一步（无固定 sleep）
                for sid, res_future in result_futures:
                    rclpy.spin_until_future_complete(self, res_future)
                    res = res_future.result().result
                    self.get_logger().info(
                        f'[Step {step_idx} Result] id={sid} success={res.success} '
                        f'final_pwm={res.final_pwm} msg="{res.message}"'
                    )
                    if not res.success:
                        self.get_logger().warn(f'舵机 {sid} 执行异常: {res.message}')

        self.get_logger().info(f'=== 动作组 [{action_name}] 播放完成 ===')


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayerNode()
    try:
        node.run_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
