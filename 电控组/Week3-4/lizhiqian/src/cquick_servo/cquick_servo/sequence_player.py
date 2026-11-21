#!/usr/bin/env python3
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self.declare_parameter('action_file', 'all_up.yaml')
        self.declare_parameter('loop', 1)
        self.declare_parameter('goal_timeout', 15.0)
        self._client = ActionClient(self, MoveServo, 'move_servo')

    def _ts(self):
        now = time.time()
        return (time.strftime('%H:%M:%S', time.localtime(now))
                + '.{0:03d}'.format(int(now * 1000) % 1000))

    def _spin_until(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def play(self):
        loop = self.get_parameter('loop').value
        timeout = self.get_parameter('goal_timeout').value

        self.get_logger().info('等待 /move_servo 服务（最长 10 秒）...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('连不上 /move_servo，请先启动 servo_driver')
            return
        self.get_logger().info('已连接 /move_servo')

        file_name = self.get_parameter('action_file').value
        pkg_dir = get_package_share_directory('cquick_servo')
        path = os.path.join(pkg_dir, 'config', 'actions', file_name)
        self.get_logger().info('读取动作文件: {}'.format(path))

        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        steps = data['steps']
        self.get_logger().info(
            '动作 {}：{} 步，连跑 {} 遍'.format(file_name, len(steps), loop)
        )

        for lap in range(loop):
            self.get_logger().info(
                '===== 第 {}/{} 遍开始 [{}] ====='.format(lap + 1, loop, self._ts())
            )
            for i, step in enumerate(steps):
                commands = step['commands']
                self.get_logger().info(
                    '[step {}] 同时下发 {} 个 goal'.format(i + 1, len(commands))
                )

                futures = []
                for cmd in commands:
                    goal = MoveServo.Goal()
                    goal.id = int(cmd['id'])
                    goal.pwm = int(cmd['pwm'])
                    goal.duration_ms = int(cmd['duration_ms'])
                    futures.append((cmd, self._client.send_goal_async(goal)))

                for cmd, future in futures:
                    if not self._spin_until(future, timeout):
                        self.get_logger().error(
                            '[step {}] id={} goal 响应超时'.format(i + 1, cmd['id'])
                        )
                        continue
                    goal_handle = future.result()
                    if not goal_handle.accepted:
                        self.get_logger().warn(
                            '[step {}] id={} 被拒绝'.format(i + 1, cmd['id'])
                        )
                        continue
                    result_future = goal_handle.get_result_async()
                    if not self._spin_until(result_future, timeout):
                        self.get_logger().error(
                            '[step {}] id={} result 超时'.format(i + 1, cmd['id'])
                        )
                        continue
                    result = result_future.result().result
                    self.get_logger().info(
                        '[step {}] id={} result success={} message={} '
                        'final_pwm={} [{}]'.format(
                            i + 1, cmd['id'], result.success,
                            result.message, result.final_pwm, self._ts()
                        )
                    )
            self.get_logger().info(
                '===== 第 {}/{} 遍完成 [{}] ====='.format(lap + 1, loop, self._ts())
            )

        self.get_logger().info('整套动作播放完成')


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    node.play()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()