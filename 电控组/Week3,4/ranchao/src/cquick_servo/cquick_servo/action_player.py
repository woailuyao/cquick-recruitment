# 动作组播放器：读 yaml -> 展开成 goal 序列 -> 等上一个 result 回来再发下一个
# 考核要点：不用 sleep 猜时间，用 result 当同步信号；动作全在 yaml 里，换 yaml 不改代码
#
# 用法：
#   ros2 run cquick_servo action_player --ros-args \
#     -p actions_file:=<yaml 绝对路径> -p group_name:=lift -p loops:=3
import os
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient

from cquick_servo_interfaces.action import MoveServo


class ActionPlayer(Node):
    def __init__(self):
        super().__init__('action_player')
        # 三个参数都从命令行传：动作文件、组名、跑几遍
        self.declare_parameter('actions_file', '')
        self.declare_parameter('group_name', 'lift')
        self.declare_parameter('loops', 3)
        self._client = ActionClient(self, MoveServo, 'move_servo')

    def now(self):
        return time.strftime('%H:%M:%S', time.localtime())

    def play(self):
        file_path = self.get_parameter('actions_file').value
        group = self.get_parameter('group_name').value
        loops = int(self.get_parameter('loops').value)

        if not file_path or not os.path.isfile(file_path):
            self.get_logger().error(f'[{self.now()}] 找不到动作文件: {file_path!r}')
            return

        with open(file_path, 'r', encoding='utf-8-sig') as f:
            data = yaml.safe_load(f)

        actions = data.get('actions', [])
        self.get_logger().info(
            f'[{self.now()}] 加载动作组 [{group}]: {len(actions)} 个动作, 文件 {file_path}')

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'[{self.now()}] /move_servo 服务端没起来')
            return

        for loop in range(loops):
            self.get_logger().info(f'[{self.now()}] ===== 第 {loop + 1}/{loops} 遍 =====')
            for i, act in enumerate(actions, start=1):
                goal = MoveServo.Goal()
                goal.command.id = int(act['servo_id'])
                goal.command.pwm = int(act['pwm'])
                goal.command.duration_ms = int(act['duration_ms'])

                self.get_logger().info(
                    f'[{self.now()}] 第{loop + 1}遍 动作{i}: 发 goal '
                    f'id={goal.command.id} pwm={goal.command.pwm} '
                    f'duration_ms={goal.command.duration_ms}')

                # 关键：等上一个 result 回来才发下一个，不用 sleep 猜时间。
                # 舵机走完的时间不等于 duration_ms，固定 sleep 会累积误差，
                # 连跑三遍动作必然对不上；result 是服务端给的准确同步信号。
                send_future = self._client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, send_future)
                gh = send_future.result()
                if not gh.accepted:
                    self.get_logger().warn(
                        f'[{self.now()}] 第{loop + 1}遍 动作{i}: goal 被拒, 跳过')
                    continue

                result_future = gh.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result()
                self.get_logger().info(
                    f'[{self.now()}] 第{loop + 1}遍 动作{i}: result 回来 '
                    f'(status={result.status})')

            self.get_logger().info(f'[{self.now()}] 第 {loop + 1} 遍结束')

        self.get_logger().info(f'[{self.now()}] 动作组 [{group}] 共 {loops} 遍全部完成')


def main(args=None):
    rclpy.init(args=args)
    node = ActionPlayer()
    node.play()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
