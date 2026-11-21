"""Play YAML servo steps through the MoveServo action."""

import argparse
from datetime import datetime
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import yaml

from cquick_servo_interfaces.action import MoveServo


def load_sequence(path):
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get("steps"), list):
        raise ValueError("YAML 顶层必须包含 steps 列表")
    if not data["steps"]:
        raise ValueError("steps 不能为空")
    steps = []
    for step_number, step in enumerate(data["steps"], 1):
        if not isinstance(step, dict) or not isinstance(step.get("goals"), list) or not step["goals"]:
            raise ValueError(f"第 {step_number} 步必须包含非空 goals 列表")
        seen = set()
        goals = []
        for goal in step["goals"]:
            if not isinstance(goal, dict) or set(goal) != {"id", "pwm", "duration_ms"}:
                raise ValueError(f"第 {step_number} 步的每个 goal 必须恰有 id、pwm、duration_ms")
            servo_id, pwm, duration = (goal[key] for key in ("id", "pwm", "duration_ms"))
            if any(type(value) is not int for value in (servo_id, pwm, duration)):
                raise ValueError(f"第 {step_number} 步的目标值必须是整数")
            if not (1 <= servo_id <= 254 and 0 <= pwm <= 65535 and 1 <= duration <= 9999):
                raise ValueError(f"第 {step_number} 步的目标值超出接口范围")
            if servo_id in seen:
                raise ValueError(f"第 {step_number} 步重复控制 ID {servo_id}")
            seen.add(servo_id)
            goals.append((servo_id, pwm, duration))
        steps.append(goals)
    return steps


class SequencePlayer(Node):
    def __init__(self):
        super().__init__("sequence_player")
        self.client = ActionClient(self, MoveServo, "/servo/move")

    def wait(self, futures, timeout):
        deadline = time.monotonic() + timeout
        while any(not future.done() for future in futures):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("等待 Action 应答超时")
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    def play(self, steps):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise TimeoutError("/servo/move Action 服务端未启动")
        for step_number, goals in enumerate(steps, 1):
            self.get_logger().info(f"step {step_number} 开始，目标数={len(goals)}")
            sent = []
            for servo_id, pwm, duration in goals:
                goal = MoveServo.Goal()
                goal.id, goal.pwm, goal.duration_ms = servo_id, pwm, duration
                sent.append((servo_id, self.client.send_goal_async(goal)))
                self.get_logger().info(f"{datetime.now().isoformat(timespec='milliseconds')} step {step_number} 发送 id={servo_id} pwm={pwm} duration_ms={duration}")
            self.wait([future for _, future in sent], 10.0)
            accepted = []
            rejected = False
            for servo_id, future in sent:
                handle = future.result()
                if handle is None or not handle.accepted:
                    self.get_logger().error(f"step {step_number} id={servo_id} 目标被拒绝")
                    rejected = True
                else:
                    accepted.append((servo_id, handle))
            if rejected:
                for _, handle in accepted:
                    handle.cancel_goal_async()
                raise RuntimeError(f"step {step_number} 有目标被拒绝，已请求取消同一步其余目标")
            results = [(servo_id, handle.get_result_async()) for servo_id, handle in accepted]
            try:
                self.wait([future for _, future in results], max(duration for _, _, duration in goals) / 1000.0 * 2 + 10.0)
            except TimeoutError:
                for _, handle in accepted:
                    handle.cancel_goal_async()
                raise
            failed = False
            for servo_id, future in results:
                response = future.result()
                result = response.result
                self.get_logger().info(f"{datetime.now().isoformat(timespec='milliseconds')} step {step_number} result id={servo_id} success={result.success} final_pwm={result.final_pwm} message={result.message}")
                failed |= not result.success
            if failed:
                raise RuntimeError(f"step {step_number} 有目标失败，动作组停止")
            self.get_logger().info(f"step {step_number} 全部 result 返回，进入下一步")


def main(args=None):
    parser = argparse.ArgumentParser(description="从 YAML 播放多舵机动作组")
    parser.add_argument("action", help="config/actions 下的文件名，或 YAML 绝对路径")
    parser.add_argument("--repeat", type=int, default=1)
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    if options.repeat < 1:
        parser.error("--repeat 必须大于 0")
    path = Path(options.action)
    if not path.is_absolute():
        path = Path(get_package_share_directory("cquick_servo")) / "config" / "actions" / path
    try:
        steps = load_sequence(path)
    except (OSError, yaml.YAMLError, ValueError) as exc:
        parser.error(str(exc))
    rclpy.init(args=args)
    node = SequencePlayer()
    try:
        for index in range(1, options.repeat + 1):
            node.get_logger().info(f"第 {index}/{options.repeat} 遍动作开始")
            node.play(steps)
        node.get_logger().info("动作组播放完成")
    except (RuntimeError, TimeoutError) as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
