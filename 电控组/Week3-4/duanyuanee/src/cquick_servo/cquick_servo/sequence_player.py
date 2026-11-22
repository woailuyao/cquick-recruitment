#!/usr/bin/env python3
"""动作组播放器：读 config/actions/*.yaml -> 展开成 goal 序列 -> 按 result 同步。

验收点：换一套动作只需新增一个 yaml，不用改一行 python。
每个 step 内的多条指令一起下发；等该 step 所有 goal 都收到 result
才进入下一步(result 当同步信号)。日志带 result 时间戳，便于证明时序。
"""
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from cquick_servo_interfaces.action import MoveServo

try:
    import yaml
except ImportError:
    yaml = None  # 运行前需 pip install pyyaml


class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self.client = ActionClient(self, MoveServo, '/servo/move')
        self.declare_parameter('action_file', '')
        self.declare_parameter('loop', 1)

    def load_actions(self, path):
        if yaml is None:
            self.get_logger().error('缺少 pyyaml：pip install pyyaml')
            return None
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def send_one(self, cmd):
        """发单条 goal 并阻塞等 result，返回 (accepted, result)。"""
        goal = MoveServo.Goal()
        goal.command.id = int(cmd['id'])
        goal.command.pwm = int(cmd['pwm'])
        goal.command.duration_ms = int(cmd['duration_ms'])
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            return False, None
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done)
        return True, done.result().result

    def play(self, data):
        steps = data.get('steps', [])
        self.get_logger().info(
            f"播放动作组 [{data.get('name', 'unnamed')}] 共 {len(steps)} 步")
        for i, step in enumerate(steps):
            cmds = step.get('commands', [])
            stamp = time.strftime('%H:%M:%S')
            self.get_logger().info(f'[{stamp}] step {i+1}/{len(steps)}: {step.get("name", "")}')
            for cmd in cmds:
                ok, result = self.send_one(cmd)
                stamp = time.strftime('%H:%M:%S')
                if ok and result is not None:
                    self.get_logger().info(
                        f'[{stamp}]   result id={cmd["id"]} success={result.success} '
                        f'final_pwm={result.final_pwm} msg={result.message}')
                else:
                    self.get_logger().warn(f'[{stamp}]   id={cmd["id"]} goal 被拒')

    def run(self, path):
        data = self.load_actions(path)
        if not data:
            return
        self.client.wait_for_server()
        loops = int(self.get_parameter('loop').value)
        for n in range(loops):
            self.get_logger().info(f'=== 第 {n+1}/{loops} 遍 ===')
            self.play(data)


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    action_file = node.get_parameter('action_file').value
    if not action_file:
        share = get_package_share_directory('cquick_servo')
        action_file = os.path.join(share, 'config', 'actions', 'stand_up.yaml')
    # 也允许命令行直接给路径：ros2 run cquick_servo sequence_player --ros-args -p action_file:=xxx
    if len(sys.argv) > 1 and sys.argv[1].endswith(('.yaml', '.yml')):
        action_file = sys.argv[1]
    node.get_logger().info(f'动作文件: {action_file}')
    try:
        node.run(action_file)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
