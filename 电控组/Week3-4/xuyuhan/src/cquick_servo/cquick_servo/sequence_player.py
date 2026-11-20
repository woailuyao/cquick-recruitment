#!/usr/bin/env python3
"""动作组播放器：yaml → 并发 goal → result 同步，异常立即整机卸力。

安全规则：
  * 同一 step 只允许不同 ID，同时下发；
  * 任一拒绝、失败、响应/结果超时或 Ctrl+C，立即取消已知 goal，
    并设置驱动 torque_enable=false，防止迟到的 goal 响应重新加力；
  * 异常后必须人工检查，再显式恢复 torque_enable=true；
  * 成功时才按全部 result 回收推进下一 step，不用 sleep 猜时间。
"""
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from cquick_servo_interfaces.action import MoveServo

DEFAULT_ACTION = 'march_in_place_final.yaml'


def stamp():
    now = time.time()
    return time.strftime('%H:%M:%S', time.localtime(now)) + f'.{int(now * 1000) % 1000:03d}'


class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self.declare_parameter('action_file', '')
        self.declare_parameter('goal_response_timeout_s', 5.0)
        self.declare_parameter('result_timeout_margin_s', 5.0)
        self.declare_parameter('repeat', 1)
        self._client = ActionClient(self, MoveServo, 'move_servo')
        self._param_client = self.create_client(
            SetParameters, '/servo_driver/set_parameters')
        self._active_handles = {}
        self._pending_send_futures = []
        self._emergency_sent = False

    def resolve_file(self):
        path = self.get_parameter('action_file').value
        if path:
            return path
        share = get_package_share_directory('cquick_servo')
        return os.path.join(share, 'config', 'actions', DEFAULT_ACTION)

    def _remember_accepted(self):
        """把已完成响应的 handle 收回；响应阶段退出也能取消。"""
        for servo_id, future in self._pending_send_futures:
            if not future.done() or servo_id in self._active_handles:
                continue
            try:
                handle = future.result()
            except Exception:
                continue
            if handle is not None and handle.accepted:
                self._active_handles[servo_id] = handle

    def cancel_all_active(self):
        self._remember_accepted()
        for servo_id, handle in list(self._active_handles.items()):
            self.get_logger().warning(f'[{stamp()}] 取消活动 goal id={servo_id}')
            handle.cancel_goal_async()

    def emergency_stop(self):
        """取消已知 goal + 全局卸力；迟到响应也无法绕过驱动 inhibit。"""
        self.cancel_all_active()
        if self._emergency_sent:
            return
        self._emergency_sent = True
        if not self._param_client.service_is_ready():
            self.get_logger().error('卸力参数服务不可用：请直接断动力电')
            return
        request = SetParameters.Request()
        request.parameters = [Parameter('torque_enable', value=False).to_parameter_msg()]
        self._param_client.call_async(request)
        self.get_logger().error('已请求整机 torque_enable=false；检查后手动恢复')

    def drain_cancel_requests(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def play_step(self, index, step):
        servos = step.get('servos', []) or []
        if not servos:
            self.get_logger().error(f'step {index} 没有 servos，停止')
            return False
        try:
            servo_ids = [int(item['id']) for item in servos]
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'step {index} 格式错误: {exc}')
            return False
        if len(set(servo_ids)) != len(servo_ids):
            self.get_logger().error(f'step {index} 存在重复 ID {servo_ids}，禁止同 ID 并发')
            return False

        self.get_logger().info(f'--- step {index}: {step.get("name", "")} ---')
        send_futures = []
        durations = []
        for item in servos:
            goal = MoveServo.Goal()
            goal.id = int(item['id'])
            goal.pwm = int(item['pwm'])
            goal.duration_ms = int(item.get('duration_ms', 800))
            durations.append(goal.duration_ms)
            self.get_logger().info(
                f'[{stamp()}] send goal id={goal.id} pwm={goal.pwm} dur={goal.duration_ms}')
            send_futures.append((goal.id, self._client.send_goal_async(goal)))
        self._pending_send_futures = send_futures

        pending_response = dict(send_futures)
        pending_result = {}
        started = time.monotonic()
        response_deadline = started + float(
            self.get_parameter('goal_response_timeout_s').value)
        overall_deadline = started + max(durations) / 1000.0 + float(
            self.get_parameter('result_timeout_margin_s').value)

        # 响应与 result 在同一个循环处理：早到拒绝/失败不会等其他响应。
        while pending_response or pending_result:
            now = time.monotonic()
            if pending_response and now >= response_deadline:
                self.get_logger().error(f'[{stamp()}] goal 响应超时 ids={list(pending_response)}')
                self.emergency_stop()
                return False
            if now >= overall_deadline:
                self.get_logger().error(
                    f'[{stamp()}] goal/result 总超时 response={list(pending_response)} '
                    f'result={list(pending_result)}')
                self.emergency_stop()
                return False

            rclpy.spin_once(self, timeout_sec=0.02)

            for servo_id, future in list(pending_response.items()):
                if not future.done():
                    continue
                del pending_response[servo_id]
                handle = future.result()
                if handle is None or not handle.accepted:
                    self.get_logger().error(f'[{stamp()}] goal 被拒绝 id={servo_id}')
                    self.emergency_stop()       # 立即取消已接受者 + 全局卸力
                    return False
                self._active_handles[servo_id] = handle
                pending_result[servo_id] = handle.get_result_async()

            for servo_id, future in list(pending_result.items()):
                if not future.done():
                    continue
                del pending_result[servo_id]
                self._active_handles.pop(servo_id, None)
                result = future.result().result
                self.get_logger().info(
                    f'[{stamp()}] result id={servo_id} success={result.success} '
                    f'message={result.message} final_pwm={result.final_pwm}')
                if not result.success:
                    self.emergency_stop()
                    return False

        self._pending_send_futures = []
        self.get_logger().info(f'[{stamp()}] step {index} result 全部回收 ok=True')
        return True

    def play(self):
        path = self.resolve_file()
        self.get_logger().info(f'动作文件: {path}')
        if not os.path.exists(path):
            self.get_logger().error(f'文件不存在: {path}')
            return False
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        steps = data.get('steps', []) or []
        if not steps:
            self.get_logger().error('动作文件没有 steps')
            return False
        self.get_logger().info(f'动作组 "{data.get("name", "unnamed")}" 共 {len(steps)} 步')

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('等待 move_servo 超时')
            return False
        repeat = int(self.get_parameter('repeat').value)
        if not 1 <= repeat <= 10:
            self.get_logger().error('repeat 必须在 1~10')
            return False
        for run_index in range(repeat):
            self.get_logger().info(f'===== 第 {run_index + 1}/{repeat} 遍 =====')
            for index, step in enumerate(steps):
                if not self.play_step(index, step):
                    self.get_logger().error(f'[{stamp()}] step {index} 失败，动作组立即停止')
                    return False
        self.get_logger().info(f'[{stamp()}] 动作组播放结束 ok=True，共 {repeat} 遍')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    ok = False
    try:
        ok = node.play()
    except KeyboardInterrupt:
        node.get_logger().warning('Ctrl+C：取消 goal 并整机卸力')
    finally:
        if not ok:
            node.emergency_stop()
            node.drain_cancel_requests()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
