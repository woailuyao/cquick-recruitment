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

抢占（加分项）：再订阅一个话题（默认 /sequence/play，std_msgs/String，内容是另一个
动作组 yaml 的绝对路径）。收到后先把在跑的 goal 取消，当前动作组在「步与步之间」中止，
然后立刻改跑新动作组 —— 舵机不会停在半路，也不会等旧动作组跑完。
"""
from __future__ import annotations

import os
import sys
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from cquick_servo_interfaces.action import MoveServo
from cquick_servo.sequence_logic import (
    STATUS_CANCELED,
    STATUS_OK,
    classify_canceled,
    pick_next_path,
    should_abort,
)

PREEMPTED = 3          # play() 的返回码：被抢占，main() 改跑新动作组
MAX_CHAIN = 3          # 连续抢占最多接几棒，防止抖动成死循环
CANCEL_WAIT_SEC = 2.0  # 等 cancel_goal 响应最多这么久，超时也不阻塞新动作组


class SequencePlayer(Node):
    def __init__(self) -> None:
        super().__init__("sequence_player")
        self.declare_parameter("actions_file", "")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("listen_topic", True)
        self.declare_parameter("play_topic", "/sequence/play")
        self._client = ActionClient(self, MoveServo, "move_servo")
        self._pending = ""
        self._preempt = False
        self._current = ""
        if bool(self.get_parameter("listen_topic").value):
            topic = str(self.get_parameter("play_topic").value)
            self.create_subscription(String, topic, self._on_play, 10)
            self.get_logger().info(f"抢占监听已开：{topic}（发 yaml 路径即可抢占）")

    # ------------------------------------------------------------------ 抢占
    def _on_play(self, msg: String) -> None:
        path = (msg.data or "").strip()
        if not path:
            self.get_logger().warn("收到空的抢占请求，忽略")
            return
        if not os.path.isfile(path):
            self.get_logger().error(f"抢占请求的 yaml 不存在：{path}")
            return
        if path == self._current and not self._preempt:
            # 同一条重复发（比如多播几次保证送达）不该把自己打断
            self.get_logger().warn(f"已经在跑「{os.path.basename(path)}」，忽略重复请求")
            return
        # 回调里只记状态、不做 I/O 阻塞，免得把 executor 占住
        self._pending = path
        self._preempt = True
        self.get_logger().warn(f"收到抢占请求：{os.path.basename(path)}")

    def take_pending(self) -> str:
        path = self._pending
        self._pending = ""
        self._preempt = False
        return path

    def _wait_future(self, future, watch_preempt: bool = True) -> bool:
        """轮询等 future：等到了返回 True，被抢占打断返回 False。"""
        while rclpy.ok():
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
            if future.done():
                return True
            if watch_preempt and self._preempt:
                return False
        return future.done()

    def _cancel_handles(self, handles) -> None:
        for joint, handle in handles:
            future = handle.cancel_goal_async()
            deadline = time.monotonic() + CANCEL_WAIT_SEC
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.05)
            if future.done():
                self.get_logger().warn(f"  id={joint['id']} 取消请求已受理")
            else:
                self.get_logger().warn(f"  id={joint['id']} 取消请求超时，不等它")

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
        self._current = path
        self.get_logger().info(f"动作组「{name}」：{len(steps)} 步 × {repeat} 轮")

        if not dry_run and not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("等不到 /move_servo 服务端")
            return 2

        failures = 0
        skipped = 0
        for round_index in range(repeat):
            for step_index, step in enumerate(steps, start=1):
                # 抢占只在步与步之间开刀：正在走的这一步不打断
                if should_abort(self._preempt, step_index, len(steps)):
                    self.get_logger().warn(
                        f"抢占：中断旧动作组「{name}」（第 {step_index} 步，没开始这一步）"
                    )
                    return PREEMPTED

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

                # ② 等这一步所有 result 回来再进下一步（这就是同步，不用 sleep）
                preempted = False
                for joint, handle in handles:
                    result_future = handle.get_result_async()
                    if not self._wait_future(result_future):
                        preempted = True
                        break
                    wrapped = result_future.result()
                    result = wrapped.result if wrapped is not None else None
                    ok = bool(result.success) if result is not None else False
                    message = result.message if result is not None else "no_result"
                    verdict = classify_canceled(ok, message)
                    if verdict == STATUS_OK:
                        self.get_logger().info(
                            f"  id={joint['id']} 到位 prad={result.prad_actual} ({message})"
                        )
                    elif verdict == STATUS_CANCELED:
                        skipped += 1
                        self.get_logger().warn(
                            f"  id={joint['id']} 被抢占取消，跳过（{message}）"
                        )
                    else:
                        failures += 1
                        self.get_logger().error(
                            f"  id={joint['id']} 未到位 ({message})"
                        )

                if preempted:
                    self.get_logger().warn(
                        f"抢占：中断动作组「{name}」（第 {step_index} 步，正在等 result）"
                    )
                    self._cancel_handles(handles)
                    return PREEMPTED

        tail = f"（被抢占取消跳过 {skipped} 个）" if skipped else ""
        self.get_logger().info(f"动作组结束，失败 {failures} 个 joint{tail}")
        return 0 if failures == 0 else 1


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = SequencePlayer()
    code = 1
    try:
        path = str(node.get_parameter("actions_file").value)
        dry_run = bool(node.get_parameter("dry_run").value)
        if not path:
            node.get_logger().error("必须给 -p actions_file:=/path/to/x.yaml")
            return
        for _ in range(MAX_CHAIN):
            code = node.play(path, dry_run)
            if code != PREEMPTED:
                break
            next_path, took = pick_next_path(node.take_pending(), path)
            if not took:
                node.get_logger().warn("抢占后没有新的动作组，结束")
                code = 0
                break
            node.get_logger().warn(f"-> 开始「{os.path.basename(next_path)}」")
            path = next_path
        else:
            node.get_logger().error(f"连续抢占超过 {MAX_CHAIN} 次，停止（防止抖动）")
            code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
