"""sequence_player.py —— 动作组播放器（任务 3）

读 yaml 动作文件，展开成 step 序列：
  · 一个 step 内可以有多个舵机 —— 同时下发（并行）
  · step 与 step 之间串行 —— 上一步所有 result 都回来了才进下一步
  · 同步信号是 result，不是 sleep

用法：
    ros2 run cquick_servo sequence_player --ros-args \
        -p action_file:=$(ros2 pkg prefix cquick_servo)/share/cquick_servo/config/actions/stand_up.yaml
"""

import os
from datetime import datetime

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):

    def __init__(self):
        super().__init__('sequence_player')

        self.declare_parameter('action_file', '')
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('step_gap_s', 0.2)
        self.declare_parameter('repeat', 1)

        self._gap = float(self.get_parameter('step_gap_s').value)
        self._repeat = int(self.get_parameter('repeat').value)
        self._default_duration = int(self.get_parameter('default_duration_ms').value)

        action_file = self.get_parameter('action_file').value
        if not action_file or not os.path.isfile(action_file):
            self.get_logger().error(f'动作文件不存在：{action_file!r}，请用 action_file:= 指定绝对路径')
            raise SystemExit(1)

        with open(action_file, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)

        self._name = data.get('name', os.path.basename(action_file))
        self._steps = data.get('steps', [])
        if not self._steps:
            self.get_logger().error('yaml 里没有任何 step')
            raise SystemExit(1)

        self.get_logger().info(
            f'动作组「{self._name}」：{len(self._steps)} 个 step，重复 {self._repeat} 遍')

        self._client = ActionClient(self, MoveServo, 'move_servo')
        self.get_logger().info('等待 action server /move_servo ...')
        self._client.wait_for_server()

        self._round = 0
        self._index = 0
        self._pending = 0
        self._play_next_step()

    # ---------------- 播放控制 ----------------

    def _play_next_step(self):
        if self._index >= len(self._steps):
            self._round += 1
            if self._round >= self._repeat:
                self.get_logger().info(
                    f'动作组「{self._name}」播放完毕（{self._repeat} 遍）')
                return
            self.get_logger().info(f'开始第 {self._round + 1} 遍')
            self._index = 0

        step = self._steps[self._index]
        self.get_logger().info(
            f'--- step {self._index + 1}/{len(self._steps)}：{len(step)} 个舵机同时下发')

        self._pending = len(step)
        for item in step:
            goal = MoveServo.Goal()
            goal.id = int(item['id'])
            goal.pwm = int(item['pwm'])
            goal.duration_ms = int(item.get('duration_ms', self._default_duration))
            future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
            future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            stamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self.get_logger().error(f'[{stamp}] goal 被拒绝')
            self._on_one_finished()
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        stamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        state = 'OK' if result.success else 'FAIL'
        self.get_logger().info(
            f'[{stamp}] result 收到：{state} final_pwm={result.final_pwm} '
            f'msg={result.message}')
        self._on_one_finished()

    def _on_one_finished(self):
        self._pending -= 1
        if self._pending > 0:
            return
        # 本 step 所有 result 都回来了，才允许进下一步
        self._index += 1
        self._gap_timer = self.create_timer(self._gap, self._gap_tick)

    def _gap_tick(self):
        self._gap_timer.cancel()
        self._play_next_step()

    def _on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'feedback pwm={feedback.current_pwm} progress={feedback.progress:.2f}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SequencePlayer()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
