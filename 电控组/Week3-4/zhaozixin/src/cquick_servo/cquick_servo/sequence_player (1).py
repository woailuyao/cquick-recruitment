#!/usr/bin/env python3
"""动作组播放器：读 config/actions/*.yaml，展开成一串 goal 顺序下发。

时序规则（任务 3 的硬要求）：
  - 同一 step 内的多个关节：同时下发（并行）
  - step 与 step 之间：等上一步所有 result 都回来，才发下一步
  - 同步信号只用 result，禁止用 sleep 猜舵机走完时间

换一套动作只改 yaml，不改本文件：
  ros2 run cquick_servo sequence_player --ros-args -p actions:=wave.yaml
"""
from datetime import datetime
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo

MAX_ID = 254
MAX_PWM = 65535
MAX_DURATION_MS = 9999


def ts():
    """日志用挂钟时间戳，评审要看 result 时序时能对得上录屏。"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def validate_sequence(seq):
    """读进来先校验，别让字段写错的动作在舵机上静默跑歪。

    返回 (steps, default_duration_ms)。任何一处不合法都抛 ValueError，
    main 里打印一句人话原因后退出，而不是发一串 undefined 的 goal。
    """
    if not isinstance(seq, dict):
        raise ValueError("yaml 顶层必须是映射（name / steps / default_duration_ms）")
    steps = seq.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("yaml 里必须有不为空的 steps 列表")

    default_dur = int(seq.get("default_duration_ms", 800))
    if default_dur <= 0 or default_dur > MAX_DURATION_MS:
        raise ValueError(f"default_duration_ms={default_dur} 不在 1~{MAX_DURATION_MS}")

    checked = []
    for step_idx, step in enumerate(steps, start=1):
        if not isinstance(step, dict):
            raise ValueError(f"step{step_idx} 必须是映射")
        joints = step.get("joints")
        if not isinstance(joints, list) or not joints:
            raise ValueError(f"step{step_idx} 的 joints 必须是非空列表")

        seen = set()
        pairs = []
        for joint in joints:
            if not isinstance(joint, dict):
                raise ValueError(f"step{step_idx} 的 joint 必须是映射")
            for key in ("id", "pwm"):
                if key not in joint:
                    raise ValueError(f"step{step_idx} 的 joint 缺字段 {key}")
            sid = joint["id"]
            pwm = joint["pwm"]
            dur = joint.get("duration_ms", default_dur)
            if not isinstance(sid, int) or not (1 <= sid <= MAX_ID):
                raise ValueError(f"step{step_idx} 的 id={sid} 不在 1~{MAX_ID}")
            if not isinstance(pwm, int) or not (0 <= pwm <= MAX_PWM):
                raise ValueError(f"step{step_idx} 的 pwm={pwm} 不在 0~{MAX_PWM}")
            if not isinstance(dur, int) or not (1 <= dur <= MAX_DURATION_MS):
                raise ValueError(
                    f"step{step_idx} id={sid} 的 duration_ms={dur} "
                    f"不在 1~{MAX_DURATION_MS}")
            if sid in seen:
                raise ValueError(
                    f"step{step_idx} 里重复出现 id={sid}：同一舵机在一步内只能给一个目标")
            seen.add(sid)
            pairs.append((sid, pwm, dur))
        checked.append((step.get("name", ""), pairs))
    return checked, default_dur


class SequencePlayer(Node):
    def __init__(self):
        super().__init__("sequence_player")

        # 换动作只改这两个参数 / yaml，不动 python
        self.declare_parameter("actions", "stand_up.yaml")
        self.declare_parameter("actions_dir", "")  # 空则用包内 share/config/actions
        self.declare_parameter("repeat", 1)        # 连跑几遍，验证可重复性
        self.declare_parameter("show_feedback", True)
        self.declare_parameter("stop_on_fail", False)

        self.cb_group = ReentrantCallbackGroup()
        self.client = ActionClient(
            self, MoveServo, "move_servo", callback_group=self.cb_group
        )
        self.active_handles = []

    # ---------- 基础工具 ----------
    def wait_all(self, futures):
        """自旋直到所有 future 完成，返回各自结果。"""
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
            f"  [{ts()}] [feedback] id={sid} pwm={fb.current_pwm} "
            f"progress={fb.progress:.2f}"
        )

    def cancel_all(self):
        """Ctrl+C 时把还在跑的 goal 一起取消，别让舵机顶在那儿。"""
        for handle in self.active_handles:
            handle.cancel_goal_async()
        self.active_handles = []

    # ---------- 单个 step ----------
    def play_step(self, step_idx, name, joints):
        self.get_logger().info(
            f"[{ts()}] --- step{step_idx} [{name}] {len(joints)} 个关节同时下发 ---"
        )
        t_step = time.monotonic()

        pending = []
        for sid, pwm, dur in joints:
            goal = MoveServo.Goal()
            goal.id = sid
            goal.pwm = pwm
            goal.duration_ms = dur
            self.get_logger().info(
                f"[{ts()}] step{step_idx} 发送 id={sid} pwm={pwm} duration_ms={dur}")
            fu = self.client.send_goal_async(
                goal, feedback_callback=lambda m, s=sid: self.on_feedback(m, s)
            )
            pending.append((sid, fu))

        handles = self.wait_all([f for _, f in pending])

        accepted = []
        for (sid, _), h in zip(pending, handles):
            if h is None:
                self.get_logger().error(f"[{ts()}] id={sid} 没有收到 goal 响应")
            elif not h.accepted:
                self.get_logger().error(
                    f"[{ts()}] id={sid} goal 被拒绝（超限 / ID 不在 servo_ids）")
            else:
                accepted.append((sid, h))
                self.active_handles.append(h)

        if not accepted:
            self.get_logger().error(f"[{ts()}] step{step_idx} 全部被拒绝，动作组终止")
            return False

        result_futures = [(sid, h.get_result_async()) for sid, h in accepted]
        results = self.wait_all([f for _, f in result_futures])

        ok = True
        for (sid, _), r in zip(result_futures, results):
            if r is None:
                self.get_logger().error(f"[{ts()}] id={sid} result 丢失")
                ok = False
                continue
            res = r.result
            elapsed = time.monotonic() - t_step
            level = "result" if res.success else "result FAIL"
            line = (f"[{ts()}] step{step_idx} {level} id={sid} "
                    f"success={res.success} final_pwm={res.final_pwm} "
                    f"t={elapsed:.2f}s msg={res.message}")
            if res.success:
                self.get_logger().info(line)
            else:
                ok = False
                self.get_logger().error(line)

        self.active_handles = []
        self.get_logger().info(
            f"[{ts()}] --- step{step_idx} 全部 result 已回，本步耗时 "
            f"{time.monotonic() - t_step:.2f}s，{'全部到位' if ok else '存在未到位'} ---"
        )
        return ok

    # ---------- 整套动作 ----------
    def play(self, steps, default_dur, seq_name):
        repeat = max(1, int(self.get_parameter("repeat").value))
        stop_on_fail = bool(self.get_parameter("stop_on_fail").value)

        self.get_logger().info(
            f"[{ts()}] 动作组 {seq_name}：{len(steps)} 个 step，连跑 {repeat} 遍，"
            f"默认 duration_ms={default_dur}"
        )

        ok_all = True
        for r in range(1, repeat + 1):
            t_all = time.monotonic()
            self.get_logger().info(f"[{ts()}] ===== 第 {r}/{repeat} 遍开始 =====")
            for i, (name, joints) in enumerate(steps, start=1):
                if not self.play_step(i, name, joints) and stop_on_fail:
                    self.get_logger().error(f"[{ts()}] stop_on_fail=true，中断动作组")
                    return False
            self.get_logger().info(
                f"[{ts()}] ===== 第 {r}/{repeat} 遍完成，总耗时 "
                f"{time.monotonic() - t_all:.2f}s =====")
        return ok_all


def main():
    rclpy.init()
    node = SequencePlayer()

    path = node.resolve_actions_path()
    node.get_logger().info(f"[{ts()}] 动作文件：{path}")
    try:
        with open(path, "r", encoding="utf-8") as f:
            seq = yaml.safe_load(f)
        steps, default_dur = validate_sequence(seq)
    except (OSError, yaml.YAMLError, ValueError) as exc:
        node.get_logger().error(f"动作文件用不了：{exc}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    if not node.client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("等不到 move_servo 服务端，先确认 servo_driver 起来了")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    try:
        ok = node.play(steps, default_dur, seq.get("name", "unnamed"))
    except KeyboardInterrupt:
        node.get_logger().warn(f"[{ts()}] 收到 Ctrl+C，取消当前 goal")
        node.cancel_all()
        ok = False

    node.get_logger().info(
        f"[{ts()}] " + ("动作组播放完成" if ok else "动作组播放存在失败"))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
