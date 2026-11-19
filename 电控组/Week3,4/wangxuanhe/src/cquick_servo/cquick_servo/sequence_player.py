#!/usr/bin/env python3
"""动作组播放器：读 config/actions/*.yaml，展开成一串 goal 顺序下发。

时序规则（任务 3 的硬要求）：
  - 同一 step 内的多个关节：同时下发（并行）
  - step 与 step 之间：等上一步所有 result 都回来，才发下一步
  - 同步信号只用 result，禁止用 sleep 猜舵机走完时间

换一套动作只改 yaml，不改本文件。
"""
import os
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    def __init__(self):
        super().__init__("sequence_player")

        # 只用参数控制行为：换动作 = 换 yaml
        self.declare_parameter("actions", "stand_up.yaml")
        self.declare_parameter("actions_dir", "")  # 空则用包内 share/config/actions
        self.declare_parameter("repeat", 1)        # 连跑几遍，验证可重复性
        self.declare_parameter("show_feedback", True)
        self.declare_parameter("stop_on_fail", False)

        self.cb_group = ReentrantCallbackGroup()
        self.client = ActionClient(
            self, MoveServo, "move_servo", callback_group=self.cb_group
        )

    # ---------- 基础工具 ----------
    def wait_all(self, futures):
        """自旋直到所有 future 完成，返回各自结果（超时/失败给 None）。"""
        while rclpy.ok() and not all(f.done() for f in futures):
            rclpy.spin_once(self, timeout_sec=0.02)
        return [f.result() if f.done() else None for f in futures]

    def resolve_actions_path(self):
        name = self.get_parameter("actions").value
        if os.path.isabs(name):
            return name
        custom_dir = self.get_parameter("actions_dir").value
        if custom_dir:
            return os.path.join(os.path.expanduser(custom_dir), name)
        share = get_package_share_directory("cquick_servo")
        return os.path.join(share, "config", "actions", name)

    def on_feedback(self, msg, sid):
        if not self.get_parameter("show_feedback").value:
            return
        fb = msg.feedback
        self.get_logger().info(
            f"  [feedback] id={sid} pwm={fb.current_pwm} progress={fb.progress:.2f}"
        )

    # ---------- 单个 step ----------
    def play_step(self, step_idx, step, default_dur):
        joints = step.get("joints") or []
        name = step.get("name", "")
        if not joints:
            self.get_logger().warn(f"step{step_idx} 没有 joints，跳过")
            return True

        self.get_logger().info(
            f"--- step{step_idx} [{name}] {len(joints)} 个关节同时下发 ---"
        )
        t_step = time.monotonic()

        pending = []
        for j in joints:
            sid = int(j["id"])
            goal = MoveServo.Goal()
            goal.id = sid
            goal.pwm = int(j["pwm"])
            goal.duration_ms = int(j.get("duration_ms", default_dur))
            fu = self.client.send_goal_async(
                goal, feedback_callback=lambda m, s=sid: self.on_feedback(m, s)
            )
            pending.append((sid, fu))

        handles = self.wait_all([f for _, f in pending])

        accepted = []
        for (sid, _), h in zip(pending, handles):
            if h is None:
                self.get_logger().error(f"id={sid} 没有收到 goal 响应")
            elif not h.accepted:
                self.get_logger().error(f"id={sid} goal 被拒绝（超限 / ID 不在 servo_ids）")
            else:
                accepted.append((sid, h))

        if not accepted:
            self.get_logger().error(f"step{step_idx} 全部被拒绝，动作组终止")
            return False

        result_futures = [(sid, h.get_result_async()) for sid, h in accepted]
        results = self.wait_all([f for _, f in result_futures])

        ok = True
        for (sid, _), r in zip(result_futures, results):
            if r is None:
                self.get_logger().error(f"id={sid} result 丢失")
                ok = False
                continue
            res = r.result
            elapsed = time.monotonic() - t_step
            if res.success:
                self.get_logger().info(
                    f"[result] step{step_idx} id={sid} 到位 final_pwm={res.final_pwm} "
                    f"t={elapsed:.2f}s msg={res.message}"
                )
            else:
                ok = False
                self.get_logger().error(
                    f"[result] step{step_idx} id={sid} 失败 final_pwm={res.final_pwm} "
                    f"t={elapsed:.2f}s msg={res.message}"
                )

        self.get_logger().info(
            f"--- step{step_idx} 结束，本步耗时 {time.monotonic() - t_step:.2f}s，"
            f"{'全部到位' if ok else '存在未到位'} ---"
        )
        return ok

    # ---------- 整套动作 ----------
    def play(self, seq):
        steps = seq.get("steps") or []
        default_dur = int(seq.get("default_duration_ms", 800))
        repeat = max(1, int(self.get_parameter("repeat").value))
        stop_on_fail = bool(self.get_parameter("stop_on_fail").value)

        self.get_logger().info(
            f"动作组 {seq.get('name', 'unnamed')}：{len(steps)} 个 step，"
            f"连跑 {repeat} 遍"
        )

        for r in range(1, repeat + 1):
            t_all = time.monotonic()
            self.get_logger().info(f"===== 第 {r}/{repeat} 遍开始 =====")
            for i, step in enumerate(steps, start=1):
                if not self.play_step(i, step, default_dur) and stop_on_fail:
                    self.get_logger().error("stop_on_fail=true，中断动作组")
                    return False
            self.get_logger().info(
                f"===== 第 {r}/{repeat} 遍完成，总耗时 {time.monotonic() - t_all:.2f}s ====="
            )
        return True


def main():
    rclpy.init()
    node = SequencePlayer()

    path = node.resolve_actions_path()
    node.get_logger().info(f"动作文件：{path}")
    if not os.path.isfile(path):
        node.get_logger().error(f"找不到动作文件 {path}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    with open(path, "r", encoding="utf-8") as f:
        seq = yaml.safe_load(f)

    if not node.client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("等不到 move_servo 服务端，先确认 servo_driver 起来了")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    ok = node.play(seq)
    node.get_logger().info("动作组播放完成" if ok else "动作组播放存在失败")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
