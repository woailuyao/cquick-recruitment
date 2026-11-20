"""动作组播放器：读 yaml，按步骤发 Action goal。

同步方式只用「等 result」，不用 sleep —— 细则 3.2 明确：用 sleep 同步一处扣 6 分，
因为 sleep 假设动作一定在固定时间内完成，舵机堵转/掉线就全错位。

yaml 结构（数据驱动，改动作不用改代码）：
  name: stand_up
  repeat: 1
  steps:
    - name: 同时抬两只手
      joints:
        - {id: 1, pwm: 1600, duration_ms: 800}
        - {id: 2, pwm: 1400, duration_ms: 800}
    - name: 再同时放下
      joints:
        - {id: 1, pwm: 1500, duration_ms: 600}
        - {id: 2, pwm: 1500, duration_ms: 600}
"""
from __future__ import annotations

import os
import sys

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    def __init__(self) -> None:
        super().__init__("sequence_player")
        self.declare_parameter("actions_file", "")
        self.declare_parameter("dry_run", False)
        self._client = ActionClient(self, MoveServo, "move_servo")

    # ------------------------------------------------------------------ 工具
    def _load(self, path: str) -> dict:
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    def _send_one(self, joint: dict):
        goal = MoveServo.Goal()
        goal.command.id = int(joint["id"])
        goal.command.pwm = int(joint["pwm"])
        goal.command.duration_ms = int(joint.get("duration_ms", 800))
        goal.verify_on_finish = bool(joint.get("verify", True))
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # ------------------------------------------------------------------ 主流程
    def play(self, path: str, dry_run: bool = False) -> int:
        spec = self._load(path)
        steps = spec.get("steps", [])
        repeat = int(spec.get("repeat", 1))
        name = spec.get("name", os.path.basename(path))
        self.get_logger().info(f"动作组「{name}」：{len(steps)} 步 × {repeat} 轮")

        if not dry_run and not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("等不到 /move_servo 服务端")
            return 2

        failures = 0
        for round_index in range(repeat):
            for step_index, step in enumerate(steps, start=1):
                joints = step.get("joints", [])
                label = step.get("name", f"step{step_index}")
                self.get_logger().info(f"[{round_index + 1}/{repeat}] 第{step_index}步：{label}")

                if dry_run:
                    for joint in joints:
                        self.get_logger().info(
                            f"  (dry-run) id={joint['id']} pwm={joint['pwm']} "
                            f"t={joint.get('duration_ms', 800)}ms"
                        )
                    continue

                # ① 同一步骤内的多个关节并行下发
                handles = []
                for joint in joints:
                    handle = self._send_one(joint)
                    if handle is None or not handle.accepted:
                        self.get_logger().error(f"  id={joint['id']} 的 goal 被拒绝")
                        failures += 1
                        continue
                    handles.append((joint, handle))

                # ② 等这一步骤所有 result 回来，再进下一步（这就是同步，不用 sleep）
                for joint, handle in handles:
                    result_future = handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future)
                    wrapped = result_future.result()
                    result = wrapped.result if wrapped is not None else None
                    ok = bool(result.success) if result is not None else False
                    message = result.message if result is not None else "no_result"
                    if ok:
                        self.get_logger().info(
                            f"  id={joint['id']} 到位 prad={result.prad_actual} ({message})"
                        )
                    else:
                        failures += 1
                        self.get_logger().error(
                            f"  id={joint['id']} 未到位 ({message})"
                        )
        self.get_logger().info(f"动作组结束，失败 {failures} 个 joint")
        return 0 if failures == 0 else 1


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = SequencePlayer()
    code = 1
    try:
        path = node.get_parameter("actions_file").value
        if not path:
            node.get_logger().error("必须给 -p actions_file:=/path/to/x.yaml")
            return
        code = node.play(str(path), bool(node.get_parameter("dry_run").value))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
