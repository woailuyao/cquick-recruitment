"""演示客户端：依次发送若干目标，打印 feedback 和 result 后退出。

日志风格与 servo_driver / sequence_player 保持一致：
▸ 动作   ✓ 成功   ✗ 失败   ⚠ 警告   ·  分隔
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo

SERVER_TIMEOUT = 15.0  # 等驱动上线
SEND_TIMEOUT = 5.0  # 等 accept
RESULT_TIMEOUT = 60.0  # 等 result，必须大于最长的 duration_ms

# (pwm, duration_ms) —— 舵机 id 由参数 servo_id 决定（默认 1）
TARGETS = [
    (1800, 1000),
    (1200, 1000),
    (1500, 1000),
]


class MoveClient(Node):
    """依次演示「发 goal → 看 feedback → 收 result」。"""

    def __init__(self) -> None:
        super().__init__("move_client")
        self._client = ActionClient(self, MoveServo, "move_servo")  # 客户端
        self._last_progress = -1.0  # 用于节流
        self._current_id = -1  # 当前 goal 的舵机 id（feedback 里要打印）
        self.declare_parameter("servo_id", 1)
        self._servo_id = self.get_parameter("servo_id").get_parameter_value().integer_value

    def _on_feedback(self, msg) -> None:
        data = msg.feedback
        if (
            data.progress - self._last_progress < 0.2 and data.progress < 1.0
        ):  # 反馈状态，并避免刷屏
            return
        self._last_progress = data.progress
        self.get_logger().info(
            f"      · id={self._current_id} pwm={data.current_pwm:4d} "
            f"progress={data.progress:.2f}"
        )

    def _await(self, future, timeout: float, what: str):
        """自旋到 future 完成；超时或 context 关闭时抛异常。"""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.done():
            return future.result()
        if not rclpy.ok():
            raise KeyboardInterrupt(f"等待{what}时 context 已关闭")
        raise TimeoutError(f"等待{what}超时（{timeout:.1f}s）")

    def _move(self, idx: int, total: int, sid: int, pwm: int, duration_ms: int) -> bool:
        self._current_id = sid
        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = duration_ms
        self._last_progress = -1.0
        self.get_logger().info(f"  ▸ [{idx}/{total}] 舵机 {sid} → {pwm} · {duration_ms} ms")
        t0 = time.monotonic()

        handle = self._await(
            self._client.send_goal_async(goal, feedback_callback=self._on_feedback),
            SEND_TIMEOUT,
            " accept",
        )
        if handle is None or not handle.accepted:
            self.get_logger().error(
                f"  ✗ [{idx}/{total}] 舵机 {sid} 被拒绝（pwm {pwm} 超出限位？）"
            )
            return False

        resp = self._await(handle.get_result_async(), RESULT_TIMEOUT, " result")
        if resp is None or resp.result is None:
            self.get_logger().error(
                f"  ✗ [{idx}/{total}] 舵机 {sid} 没有拿到 result（被取消或超时）"
            )
            return False
        result = resp.result
        cost = time.monotonic() - t0
        if result.success:
            self.get_logger().info(f"  ✓ [{idx}/{total}] 到位 {result.final_pwm} · {cost:.2f} s")
        else:
            self.get_logger().error(f"  ✗ [{idx}/{total}] 失败 · {result.message}")
        return result.success

    def run(self) -> bool:
        if not self._client.wait_for_server(timeout_sec=SERVER_TIMEOUT):  # 超时报错
            self.get_logger().error("✗ 等不到 /move_servo，驱动没起来？")
            return False

        self.get_logger().info(
            f"▸ 演示开始 · 舵机 {self._servo_id} · {len(TARGETS)} 个目标 · 驱动已就绪"
        )
        t0 = time.monotonic()
        ok_count = 0

        for idx, (pwm, dur) in enumerate(TARGETS, 1):  # 读取目标
            try:
                if self._move(idx, len(TARGETS), self._servo_id, pwm, dur):
                    ok_count += 1
            except TimeoutError as exc:
                self.get_logger().error(f"  ✗ [{idx}/{len(TARGETS)}] {exc}")

        cost = time.monotonic() - t0
        if ok_count == len(TARGETS):
            self.get_logger().info(f"✓ 演示结束 · {ok_count}/{len(TARGETS)} 成功 · {cost:.2f} s")
        else:
            self.get_logger().error(f"✗ 演示结束 · {ok_count}/{len(TARGETS)} 成功 · {cost:.2f} s")
        return ok_count == len(TARGETS)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    code = 0

    try:
        node = MoveClient()
        if not node.run():
            code = 1
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        text = f"✗ 异常：{type(exc).__name__}: {exc}"
        if node is not None:
            node.get_logger().error(text)
        else:
            print(text)
        code = 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(code)


if __name__ == "__main__":
    main()
