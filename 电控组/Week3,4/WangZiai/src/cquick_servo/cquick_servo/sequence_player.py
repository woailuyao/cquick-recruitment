#!/usr/bin/env python3
"""动作组播放器：读 yaml → 每个 step 并行发 Action → 等全部 result → 下一步。

设计要点（对应第 3 周细则 G 节）：
- 动作数据全在 yaml，改动作不改 Python；
- step 内多舵机并行（同时发 goal），step 之间严格等所有 result；
- 全程不用 time.sleep，等待靠 Future 完成事件，执行器不会被阻塞（主线程是被阻塞的）。
"""

from __future__ import annotations  # 让所有类型标注变成字符串，运行期不求值

import os
import time
import threading
from typing import Any

import rclpy
import yaml  # 读 yaml 动作指令
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
)  # 防止别人把 context 关了（比如 SIGINT handler、另一个节点调 rclpy.shutdown()）时
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    """按 yaml 顺序播放动作组：step 内并行、step 间等 result。"""

    def __init__(self) -> None:
        super().__init__("sequence_player")
        self.declare_parameter(
            "action", "stand_up"
        )  # yaml动作选择：yaml 名（不含 .yaml）或绝对路径
        self.declare_parameter("duration_scale", 1.0)  # 动作整体速度：0.5 = 全部动作快一倍
        self.declare_parameter("timeout", 15.0)  # 单次等待上限：单个 future 的最长等待
        self.declare_parameter(
            "dry_run", False
        )  # 只看不跑：加载 yaml 后逐条检查是否在允许范围内，不发任何指令
        self.declare_parameter("pwm_min", 900)  # 预检范围（必须与驱动参数一致）
        self.declare_parameter("pwm_max", 2100)
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.declare_parameter("goal_retry", 6)  # 舵机忙被拒时重试次数（0 = 不重试）
        self.declare_parameter("retry_interval", 0.3)  # 每次重试前等待秒数
        self._group = ReentrantCallbackGroup()  # 预防多 client 或加订阅情况
        self._client = ActionClient(
            self, MoveServo, "move_servo", callback_group=self._group
        )  # 设置 move_servo 服务

    # ---------------------- 加 载 yaml ------------------------
    def resolve(self, name: str) -> str:
        # 可接受的action参数类型： stand_up ； stand_up.yaml ； /abs/x.yaml ； ./x.yaml
        name = name.strip()
        if os.path.isabs(name) or os.sep in name:  # 绝对路径 / 带目录的相对路径 → 当真路径
            return os.path.abspath(name)
        if not name.endswith(".yaml"):  # 纯名字（可带可不带 .yaml）→ 去 share 目录找
            name += ".yaml"
        return os.path.join(get_package_share_directory("cquick_servo"), "config", "actions", name)

    def load(self) -> dict:
        path = self.resolve(str(self.get_parameter("action").value))
        if not os.path.isfile(path):  # 未找到action的反馈
            share = os.path.join(get_package_share_directory("cquick_servo"), "config", "actions")
            available = (
                sorted(f for f in os.listdir(share) if f.endswith(".yaml"))
                if os.path.isdir(share)
                else []
            )
            raise FileNotFoundError(f"找不到动作组：{path}\n  可用：{available}")

        try:
            with open(path, encoding="utf-8") as f:  # 用utf-8格式解析基本类型
                raw: Any = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise ValueError(f"{path}: YAML 语法错误 —— {exc}") from exc

        if not isinstance(
            raw, dict
        ):  # safe_load 可能返回 None / list / Any：顶层不是映射就直接报错，防止后面莫名失败
            raise ValueError(
                f"{path}: 顶层必须是键值对（name / steps），实际是 {type(raw).__name__}"
            )
        doc: dict = raw

        steps = doc.get("steps")  # 最小校验：判断结构对不对
        if not isinstance(steps, list) or not steps:
            raise ValueError(f"{path}: 缺少非空的 steps 列表，实际顶层键 {list(doc)}")
        self.get_logger().info(f"▸ 加载 {path} · {len(steps)} 步")
        return doc

    # ----------------------- 等 Future ------------------------
    @staticmethod
    def await_future(future: Any, timeout: float, what: str) -> Any:
        """等 future 完成；期间可被关闭信号打断。"""
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())  # 用future等待反馈
        deadline = time.monotonic() + timeout  # 防future卡死

        while True:
            remaining = deadline - time.monotonic()
            if event.wait(
                max(0.001, min(0.05, remaining))
            ):  # 每次最多睡 50 ms，醒来检查两的状态：future 完成 / 系统是否已关闭
                break
            if not rclpy.ok():
                raise ExternalShutdownException(f"等待{what}时系统已关闭")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"等待{what}超时（{timeout:.1f}s）")

        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"等待{what}失败：{exc!r}") from exc

    # ---------------------- 播 放 -------------------------
    def _accept_with_retry(self, goal, fut, step_idx: int, timeout: float):
        """等 accept；被拒（舵机忙）时按 goal_retry / retry_interval 重试。"""
        retries = max(0, int(self.get_parameter("goal_retry").value))
        interval = max(0.0, float(self.get_parameter("retry_interval").value))

        for attempt in range(retries + 1):
            try:
                goal_handle = self.await_future(
                    fut, timeout, f"舵机 {goal.id} 的 goal 响应"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"step {step_idx}：舵机 {goal.id} 未收到 accept —— {exc}")
                return None

            if goal_handle.accepted:
                if attempt:
                    self.get_logger().info(f"✓ 舵机 {goal.id} 第 {attempt + 1} 次尝试被接受")
                return goal_handle

            if attempt >= retries:
                break
            self.get_logger().warn(
                f"⚠ 舵机 {goal.id} 正忙，{interval:.1f}s 后重试（{attempt + 1}/{retries}）"
            )
            time.sleep(interval)
            fut = self._client.send_goal_async(goal)

        self.get_logger().error(
            f"step {step_idx}：舵机 {goal.id} 重试 {retries} 次仍被拒绝，取消本步"
        )
        return None

    def _cancel_all(self, handles) -> None:
        """出错时把本步还没走完的 goal 全部取消，防止机器人停在半路。"""
        for goal, h in handles:
            if h is None or not getattr(h, "accepted", False):
                continue
            try:
                h.cancel_goal_async()
                self.get_logger().warn(f"   已请求取消舵机 {goal.id}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"   取消舵机 {goal.id} 失败：{exc}")

    def _abort_step(self, pending, handles) -> None:
        """一步失败时：把还没拿到 handle 的 goal 也收回来取消，避免它们晚点自己动。"""
        settled = {id(g) for g, _ in handles}
        for goal, fut in pending:
            if id(goal) in settled:
                continue
            try:
                h = self.await_future(fut, 2.0, f"舵机 {goal.id} 的 goal 响应（收尾）")
            except Exception:  # noqa: BLE001
                continue
            if h is not None and getattr(h, "accepted", False):
                handles.append((goal, h))

        self._cancel_all(handles)

    def _precheck(self, doc: dict) -> bool:
        """离线预检：对照参数范围检查每一步，不发任何 goal，不需要驱动在线。"""
        steps = doc.get("steps") or []
        ids = [
            int(x)
            for x in self.get_parameter("servo_ids").get_parameter_value().integer_array_value
        ]
        lo = self.get_parameter("pwm_min").get_parameter_value().integer_value
        hi = self.get_parameter("pwm_max").get_parameter_value().integer_value
        scale = self.get_parameter("duration_scale").get_parameter_value().double_value
        self.get_logger().info(
            f"▸ 预检「{doc.get('name', '?')}」· {len(steps)} 步"
            f" · 允许 pwm {lo}~{hi} · 舵机 {ids}"
        )

        problems = 0
        for i, step in enumerate(steps, 1):
            servos = step.get("servos") or []
            name = step.get("name", "")
            if not servos:
                self.get_logger().warn(
                    f"  [{i}/{len(steps)}] {name} · 没有 servos（运行时会跳过）"
                )
                continue

            parts = []
            for servo in servos:
                sid = int(servo.get("id", -1))
                pwm = int(servo.get("pwm", 0))
                dur = int(servo.get("duration_ms", 800))
                if sid not in ids:
                    parts.append(f"舵机 {sid} ✗ id 不在 {ids}")
                    problems += 1
                elif not lo <= pwm <= hi:
                    parts.append(f"舵机 {sid} ✗ pwm {pwm} 超出 {lo}~{hi}")
                    problems += 1
                elif dur <= 0:
                    parts.append(f"舵机 {sid} ✗ duration {dur} ms 非法")
                    problems += 1
                else:
                    parts.append(f"舵机 {sid} → {pwm} · {int(dur * scale)} ms ✓")

            self.get_logger().info(f"  [{i}/{len(steps)}] {name} · " + " ｜ ".join(parts))

        if problems:
            self.get_logger().error(f"✗ 预检未通过：{problems} 处问题（未发送任何指令）")
            return False

        self.get_logger().info("✓ 预检通过：所有动作都在允许范围内（未发送任何指令）")
        return True

    def play(self, doc: dict) -> bool:  # noqa: C901  # 编排逻辑写在一起更清楚
        steps = doc.get("steps") or []
        if not steps:
            self.get_logger().warn("⚠ 动作组为空，没有可播放的步骤")
            return False

        scale = self.get_parameter("duration_scale").get_parameter_value().double_value
        if scale <= 0:
            self.get_logger().error(f"duration_scale 必须 > 0，当前 {scale}")
            return False

        timeout = (
            self.get_parameter("timeout").get_parameter_value().double_value
        )  # 自动缩放单次等待时间
        effective_timeout = timeout * max(1.0, scale)
        if (
            self.get_parameter("dry_run").get_parameter_value().bool_value
        ):  # 只预检不放：不需要驱动在线
            return self._precheck(doc)

        self.get_logger().info(
            f"▸ 动作组「{doc.get('name', '?')}」· {len(steps)} 步"
            f" · scale {scale:g} · 单步超时 {effective_timeout:.1f} s"
        )

        if not self._client.wait_for_server(
            timeout_sec=max(5.0, effective_timeout)
        ):  # 预检查服务端状态
            self.get_logger().error("✗ 找不到 /move_servo，请先启动 servo_driver")
            return False

        for i, step in enumerate(steps, 1):  # 跳过空servos
            servos = step.get("servos") or []
            if not servos:
                self.get_logger().warn(
                    f"⚠ [{i}/{len(steps)}] {step.get('name', '')} · 没有 servos，跳过"
                )
                continue

            step_name = step.get("name", "")
            self.get_logger().info(f"▸ [{i}/{len(steps)}] {step_name} · {len(servos)} 个舵机并行")

            pending = []  # 并行发出
            for servo in servos:
                goal = MoveServo.Goal()
                goal.id = int(servo["id"])
                goal.pwm = int(servo["pwm"])
                goal.duration_ms = max(1, int(int(servo.get("duration_ms", 800)) * scale))
                pending.append((goal, self._client.send_goal_async(goal)))

            handles = []  # 统一等 accept（舵机忙就等一会重试，不立刻判失败）
            for goal, fut in pending:
                goal_handle = self._accept_with_retry(goal, fut, i, effective_timeout)
                if goal_handle is None:
                    self._abort_step(pending, handles)
                    return False
                handles.append((goal, goal_handle))

            step_start = time.monotonic()  # 共享一个 deadline，等全部 result 齐了才进下一步
            step_deadline = step_start + effective_timeout

            for goal, goal_handle in handles:
                remaining = step_deadline - time.monotonic()
                if remaining <= 0:
                    self.get_logger().error(
                        f"step {i}：等舵机 {goal.id} 的 result 超时（{effective_timeout:.1f}s）"
                    )
                    self._abort_step(pending, handles)
                    return False

                try:
                    resp = self.await_future(
                        goal_handle.get_result_async(), remaining, f"舵机 {goal.id} 的 result"
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"step {i}：等舵机 {goal.id} 的 result 失败 —— {exc}")
                    self._abort_step(pending, handles)
                    return False
                result = resp.result  # MoveServo.Result
                if not result.success:
                    self.get_logger().error(f"step {i}：舵机 {goal.id} 失败 —— {result.message}")
                    self._abort_step(pending, handles)
                    return False

                self.get_logger().info(
                    f"      ✓ 舵机 {goal.id} 到位 {result.final_pwm}"
                    f" · {time.monotonic() - step_start:.2f} s"
                )

        return True


def main(args=None) -> None:
    """入口：加载动作 yaml 并播放。"""
    rclpy.init(args=args)
    node = SequencePlayer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def _spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f"执行器异常退出：{exc}")

    spin_thread = threading.Thread(target=_spin, daemon=True)  # 强杀后台线程
    spin_thread.start()

    code = 1  # 未走完路径默认失败退出
    try:
        doc = node.load()
        dry = node.get_parameter("dry_run").get_parameter_value().bool_value
        ok = node.play(doc)
        code = 0 if ok else 1
        if dry:
            node.get_logger().info("✓ 预检结束" if ok else "✗ 预检失败")
        else:
            node.get_logger().info("✓ 动作组结束" if ok else "✗ 动作组失败")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"异常：{exc}")
    finally:
        executor.shutdown()  # 让 spin() 返回
        spin_thread.join(timeout=3.0)  # 等后台线程真正退出，再销毁节点
        if spin_thread.is_alive():  # 超时兜底
            node.get_logger().warn("执行器线程未在 3 s 内退出，继续销毁节点")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
