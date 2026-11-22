#!/usr/bin/env python3
"""动作组播放器：读 yaml，按 step 串行下发 goal，用 result 作为同步信号。"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


def _stamp():
    now = time.time()
    return (time.strftime('%H:%M:%S', time.localtime(now))
            + f'.{int(now * 1000) % 1000:03d}')


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self.declare_parameter('action_file', 'stand_up.yaml')
        self.declare_parameter('actions_dir', '')

    def resolve_path(self):
        name = self.get_parameter('action_file').value
        base = self.get_parameter('actions_dir').value
        if not base:
            base = os.path.join(
                get_package_share_directory('cquick_servo'),
                'config', 'actions')
        return os.path.join(base, name)

    def play(self, spec):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('move_servo 服务端未就绪')
            return False

        steps = spec.get('steps') or []
        self.get_logger().info(
            f'动作组 {spec.get("name", "")} 共 {len(steps)} 步')

        for idx, step in enumerate(steps, 1):
            joints = step.get('joints') or []
            self.get_logger().info(
                f'--- 第 {idx} 步 {step.get("name", "")}（{len(joints)} 关节）')

            futures = []
            for j in joints:
                goal = MoveServo.Goal()
                goal.id = int(j['id'])
                goal.pwm = int(j['pwm'])
                goal.duration_ms = int(j.get('duration_ms', 800))
                futures.append(self._client.send_goal_async(goal))

            handles = []
            for fut in futures:
                rclpy.spin_until_future_complete(self, fut)
                handle = fut.result()
                if handle is None or not handle.accepted:
                    self.get_logger().error(f'第 {idx} 步有 goal 被拒绝')
                    return False
                handles.append(handle)

            for j, handle in zip(joints, handles):
                res_fut = handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_fut)
                res = res_fut.result().result
                self.get_logger().info(
                    f'[{_stamp()}] result id={j["id"]} '
                    f'success={res.success} final={res.final_pwm} '
                    f'msg={res.message}')

        self.get_logger().info('动作组播放结束')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    path = ''
    try:
        path = node.resolve_path()
        node.get_logger().info(f'读取动作文件：{path}')
        with open(path, 'r', encoding='utf-8') as f:
            spec = yaml.safe_load(f)
        node.play(spec)
    except FileNotFoundError:
        node.get_logger().error(f'找不到动作文件：{path}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
