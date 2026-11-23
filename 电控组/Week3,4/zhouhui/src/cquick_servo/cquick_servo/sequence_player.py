"""动作组播放器：读 config/actions/*.yaml，展开成一串 goal 顺序打出去。

三条时序规则（任务 3 的 8 分就在这）：
  1. 同一个 step 里的多个关节 → send_goal_async 一次全发，并行；
  2. step 之间 → 等上一步每个 goal 的 result 都回来，才发下一步；
  3. 同步只用 result，不用 time.sleep 猜「舵机应该走完了吧」。

换动作只改 yaml（或改 actions 参数指向另一个 yaml），这个文件一行都不用动：
  ros2 run cquick_servo sequence_player --ros-args -p actions:=wave.yaml -p repeat:=3
"""
import time
from datetime import datetime

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo

from .sequence_schema import SequenceError, load_sequence, total_duration_ms


def stamp():
    """带毫秒的挂钟时间 —— 日志里要能看出 result 的先后顺序。"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class SequencePlayer(Node):
    def __init__(self):
        super().__init__("sequence_player")
        self.declare_parameter("actions", "stand_up.yaml")
        self.declare_parameter("actions_dir", "")   # 空 = 用包内 share/config/actions
        self.declare_parameter("repeat", 1)
        self.declare_parameter("show_feedback", False)
        self.declare_parameter("stop_on_fail", True)

        self.client = ActionClient(
            self, MoveServo, "move_servo",
            callback_group=ReentrantCallbackGroup())
        self._handles = []

    # ------------------------------------------------------------ 基础工具
    def _spin_until(self, futures):
        while rclpy.ok() and not all(f.done() for f in futures):
            rclpy.spin_once(self, timeout_sec=0.02)
        return [f.result() if f.done() else None for f in futures]

    def _resolve(self):
        name = str(self.get_parameter("actions").value)
        if name.startswith("/"):
            return name
        custom = str(self.get_parameter("actions_dir").value)
        if custom:
            return f"{custom.rstrip('/')}/{name}"
        share = get_package_share_directory("cquick_servo")
        return f"{share}/config/actions/{name}"

    def _on_feedback(self, sid, msg):
        if not self.get_parameter("show_feedback").value:
            return
        fb = msg.feedback
        self.get_logger().info(
            f"  [{stamp()}] feedback id={sid} pwm={fb.current_pwm} "
            f"progress={fb.progress:.2f}")

    def cancel_all(self):
        for handle in self._handles:
            handle.cancel_goal_async()
        self._handles = []

    # ---------------------------------------------------------------- 单步
    def play_step(self, index, step):
        log = self.get_logger()
        log.info(f"[{stamp()}] === step{index} 「{step.label}」 "
                 f"{len(step.moves)} 个关节同时下发 ===")
        t0 = time.monotonic()

        pending = []
        for mv in step.moves:
            goal = MoveServo.Goal()
            goal.id = mv.sid
            goal.pwm = mv.pwm
            goal.duration_ms = mv.duration_ms
            log.info(f"[{stamp()}] step{index} TX goal id={mv.sid} "
                     f"pwm={mv.pwm} duration_ms={mv.duration_ms}")
            fut = self.client.send_goal_async(
                goal, feedback_callback=lambda m, s=mv.sid: self._on_feedback(s, m))
            pending.append((mv.sid, fut))

        accepted = []
        for sid, handle in zip([p[0] for p in pending], self._spin_until([p[1] for p in pending])):
            if handle is None:
                log.error(f"[{stamp()}] id={sid} 没有 goal 响应")
            elif not handle.accepted:
                log.error(f"[{stamp()}] id={sid} goal 被拒（超限 / id 不在 servo_ids / 安全锁）")
            else:
                accepted.append((sid, handle))
                self._handles.append(handle)

        if not accepted:
            log.error(f"[{stamp()}] step{index} 没有一条 goal 被接受")
            self._handles = []
            return False

        futures = [(sid, h.get_result_async()) for sid, h in accepted]
        results = self._spin_until([f for _, f in futures])

        ok = True
        for (sid, _), wrapped in zip(futures, results):
            if wrapped is None:
                log.error(f"[{stamp()}] step{index} id={sid} result 丢了")
                ok = False
                continue
            r = wrapped.result
            took = time.monotonic() - t0
            text = (f"[{stamp()}] step{index} RESULT id={sid} success={r.success} "
                    f"final_pwm={r.final_pwm} code={r.code} t={took:.2f}s msg={r.message}")
            if r.success:
                log.info(text)
            else:
                ok = False
                log.error(text)

        self._handles = []
        log.info(f"[{stamp()}] --- step{index} 全部 result 已回，耗时 "
                 f"{time.monotonic() - t0:.2f}s，{'全部到位' if ok else '有未到位'} ---")
        return ok

    # -------------------------------------------------------------- 整套
    def play(self, sequence):
        repeat = max(1, int(self.get_parameter("repeat").value))
        stop = bool(self.get_parameter("stop_on_fail").value)
        log = self.get_logger()

        log.info(f"[{stamp()}] 动作组 name={sequence.name}：{len(sequence.steps)} 步，"
                 f"重复 {repeat} 遍，理论时长 {total_duration_ms(sequence)} ms")
        if sequence.description:
            log.info(f"说明：{sequence.description}")

        all_ok = True
        for run in range(1, repeat + 1):
            log.info(f"[{stamp()}] ===== 第 {run}/{repeat} 遍 =====")
            run_t0 = time.monotonic()
            for index, step in enumerate(sequence.steps, start=1):
                step_ok = self.play_step(index, step)
                all_ok = all_ok and step_ok
                if not step_ok and stop:
                    log.error(f"[{stamp()}] stop_on_fail=true，动作组中止")
                    return False
            log.info(f"[{stamp()}] ===== 第 {run}/{repeat} 遍完成，"
                     f"总耗时 {time.monotonic() - run_t0:.2f}s =====")
        return all_ok


def main():
    rclpy.init()
    node = SequencePlayer()
    code = 0
    try:
        path = node._resolve()
        node.get_logger().info(f"[{stamp()}] 动作文件：{path}")
        # yaml 里没写 duration_ms 时用 800，写了就以 yaml 为准
        sequence = load_sequence(path, 800)
        if not node.client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error("等不到 move_servo 服务端，先起 servo_driver")
            code = 1
        else:
            ok = node.play(sequence)
            code = 0 if ok else 2
    except SequenceError as exc:
        node.get_logger().error(f"动作文件用不了：{exc}")
        code = 1
    except KeyboardInterrupt:
        node.get_logger().warn("收到 Ctrl+C，取消正在跑的 goal")
        node.cancel_all()
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()

