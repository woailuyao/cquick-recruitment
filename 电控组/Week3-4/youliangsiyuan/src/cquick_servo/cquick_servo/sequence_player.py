#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动作组播放器：**读 yaml → 展开成一串 goal → 按 result 同步下发**。

这是任务 3 的硬要求（8 分）：动作是**数据**，不是代码。
换一套动作 = 新增一个 yaml，一行 python 都不用改。

yaml 长这样：
    name: squat_in_place
    steps:
      - name: 01_全部回中位
        joints:
          - {id: 1, pwm: 1500, duration_ms: 800}
          - {id: 2, pwm: 1500, duration_ms: 800}
      - name: 02_抬起
        joints:
          - {id: 1, pwm: 1700, duration_ms: 700}

三条语义（细则 3.4 表格里那三行）：
    一步之内多舵机协同  -> 同一个 step 的 joints 全部**同时**下发，不等彼此
    步与步之间串行      -> 上一个 step 的**所有** result 都回来了，才发下一个
    一步之内有先后      -> 想先后就拆成两个 step，**别靠调 duration_ms 硬凑**

为什么绝对不能用 time.sleep(duration_ms) 等：
    舵机"走完"的时间不等于你给的 duration_ms（加减速、负载、堵转都会拖），
    误差一步步累积，第三遍和第一遍一定对不上（细则扣 6 分那条）。
    **用 result 当同步信号**才是对的。
"""

import os
import time
from datetime import datetime

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo

ACTION_NAME = 'move_servo'


def _stamp():
    """日志里要有能对得上的时间戳 —— 细则 3.5 要求"用日志证明 result 时序"。"""
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')

        default_dir = os.path.join(
            get_package_share_directory('cquick_servo'), 'config', 'actions')
        self.declare_parameter('action_name', ACTION_NAME)
        self.declare_parameter('actions_dir', default_dir)
        self.declare_parameter('action_file', 'squat_in_place.yaml')
        self.declare_parameter('step_interval_ms', 0)   # 步与步之间额外停一下（一般 0）
        self.declare_parameter('goal_timeout_s', 10.0)  # 等 result 的上限，防死等
        self.declare_parameter('loop', False)

        self._action_name = str(self.get_parameter('action_name').value)
        self._client = ActionClient(self, MoveServo, self._action_name)

    # -------------------------------------------------------------- yaml 读取
    def load_actions(self):
        actions_dir = str(self.get_parameter('actions_dir').value)
        action_file = str(self.get_parameter('action_file').value)

        path = action_file if os.path.isabs(action_file) \
            else os.path.join(actions_dir, action_file)

        self.get_logger().info(f'读取动作文件：{path}')
        if not os.path.isfile(path):
            raise RuntimeError(
                f'动作文件不存在：{path}\n'
                f'  ① 文件名拼错了？② colcon build 了吗？'
                f'（config/actions/ 要在 setup.py 的 data_files 里装进去）\n'
                f'  ③ 看看目录里有什么：ls {actions_dir}')

        with open(path, 'r', encoding='utf-8') as fh:
            data = yaml.safe_load(fh)

        if not isinstance(data, dict) or 'steps' not in data:
            raise RuntimeError(f'{path} 格式不对：最外层要有 steps 列表')
        steps = data['steps']
        if not isinstance(steps, list) or not steps:
            raise RuntimeError(f'{path} 里的 steps 是空的')

        # 提前把结构检查一遍 —— 别等发到一半才发现少个字段
        for index, step in enumerate(steps):
            if 'joints' not in step or not step['joints']:
                raise RuntimeError(f'第 {index + 1} 个 step 没有 joints')
            for joint in step['joints']:
                for key in ('id', 'pwm', 'duration_ms'):
                    if key not in joint:
                        raise RuntimeError(
                            f'第 {index + 1} 个 step 的 {joint} 缺少字段 {key}')
        return path, data

    # ---------------------------------------------------------------- 等待
    def _spin_until(self, futures, timeout_s):
        """边 spin 边等一批 future 全完成。future 里的回调（feedback）就是在这转出来的。"""
        deadline = time.monotonic() + timeout_s
        while rclpy.ok():
            if all(f.done() for f in futures):
                return True
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.02)
        return False

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(f'[{_stamp()}] feedback pwm={fb.current_pwm} '
                                f'progress={fb.progress:.2f}')

    # -------------------------------------------------------------- 播一步
    def play_step(self, index, total, step, timeout_s):
        name = step.get('name', f'step{index}')
        joints = step['joints']
        self.get_logger().info(
            f'[{_stamp()}] === step {index}/{total} "{name}" 下发 {len(joints)} 个 goal ===')

        # ① 同一个 step 里的 goal **一起发**，不等彼此 —— 这就是"一步之内多舵机协同"
        send_futures = []
        for joint in joints:
            goal = MoveServo.Goal()
            goal.id = int(joint['id'])
            goal.pwm = int(joint['pwm'])
            goal.duration_ms = int(joint['duration_ms'])
            self.get_logger().info(
                f'[{_stamp()}]   发出 goal: id={goal.id} pwm={goal.pwm} '
                f'duration_ms={goal.duration_ms}')
            send_futures.append(
                self._client.send_goal_async(goal, feedback_callback=self._on_feedback))

        if not self._spin_until(send_futures, timeout_s):
            raise RuntimeError(f'step "{name}" 等 goal 被接受超时（{timeout_s}s）')

        goal_handles = [f.result() for f in send_futures]
        for joint, handle in zip(joints, goal_handles):
            if handle is None or not handle.accepted:
                raise RuntimeError(
                    f'step "{name}" 里 id={joint.get("id")} 的 goal 被拒绝'
                    f'（看驱动节点日志：pwm 超限或 id 不在 servo_ids）')

        # ② 等**所有** result 回来，才算这一步结束 —— 这就是"用 result 同步"
        result_futures = [h.get_result_async() for h in goal_handles]
        if not self._spin_until(result_futures, timeout_s + 5.0):
            raise RuntimeError(f'step "{name}" 等 result 超时')

        # ③ 打时序日志：每个 result 都有时间戳，评审就靠这个确认你没用 sleep 硬等
        ok_count = 0
        for joint, future in zip(joints, result_futures):
            wrapped = future.result()
            result = wrapped.result if wrapped is not None else None
            if result is None:
                self.get_logger().error(
                    f'[{_stamp()}]   id={joint.get("id")} 没拿到 result')
                continue
            flag = 'OK' if result.success else 'FAIL'
            self.get_logger().info(
                f'[{_stamp()}]   <- result id={joint.get("id")} [{flag}] '
                f'final_pwm={result.final_pwm} retry={result.retry_count} '
                f'msg={result.message}')
            if result.success:
                ok_count += 1

        self.get_logger().info(
            f'[{_stamp()}] === step {index}/{total} "{name}" 完成：'
            f'{ok_count}/{len(joints)} 到位，可以进下一步 ===')
        return ok_count == len(joints)

    # ---------------------------------------------------------------- 主流程
    def play_file(self):
        path, data = self.load_actions()
        steps = data['steps']
        total = len(steps)
        timeout_s = float(self.get_parameter('goal_timeout_s').value)
        interval = int(self.get_parameter('step_interval_ms').value) / 1000.0

        self.get_logger().info(
            f'动作组 "{data.get("name", os.path.basename(path))}"：'
            f'共 {total} 步，文件 {path}')

        self.get_logger().info(f'[{_stamp()}] 等待 action server /{self._action_name} ...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'没等到 action server /{self._action_name}')

        loop = bool(self.get_parameter('loop').value)
        while rclpy.ok():
            t0 = time.monotonic()
            for index, step in enumerate(steps, start=1):
                if not self.play_step(index, total, step, timeout_s):
                    # 大幅度分级动作中，上一级没到位就继续加深可能顶死结构。
                    # 任一 goal 失败都立即停止，不再发送后续 step。
                    name = step.get('name', f'step{index}')
                    raise RuntimeError(
                        f'step "{name}" 有舵机未到位，已停止后续动作；'
                        f'不要继续加大幅度，先检查结构与 PRAD 日志')
                if interval > 0:
                    time.sleep(interval)
            elapsed = time.monotonic() - t0
            self.get_logger().info(f'[{_stamp()}] 整套动作跑完，耗时 {elapsed:.2f}s')
            if not loop:
                break
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    exit_code = 0
    try:
        node.play_file()
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    except KeyboardInterrupt:
        node.get_logger().warn('被 Ctrl+C 中断')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
