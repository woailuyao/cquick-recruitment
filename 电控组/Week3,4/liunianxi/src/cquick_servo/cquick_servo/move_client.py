"""Action 客户端：从命令行给一次 goal，打印 feedback 与 result。

用法：
  ros2 run cquick_servo move_client --id 1 --pwm 1700 --duration 800
  ros2 run cquick_servo move_client --id 1 --pwm 1700 --no-verify
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


# 最小的演示客户端：发一条 goal、打 feedback、打 result；退出码给 shell 脚本判断用。
class MoveClient(Node):
    def __init__(self) -> None:
        super().__init__("move_client")
        self._client = ActionClient(self, MoveServo, "move_servo")

    # feedback 回调：屏幕上“pwm_current 连续变化”就是这条消息在逐帧刷新。
    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"id={self._req_id} pwm={fb.pwm_current} progress={fb.progress:.2f}"
        )

    def send(self, frame_id: int, pwm: int, duration_ms: int, verify: bool) -> int:
        self._req_id = frame_id
        # 5 s 等不到服务端就返回 2：现场忘了起 launch 时脚本立刻报错，不会无限挂住。
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("等不到 /move_servo 服务端（驱动节点没起来？）")
            return 2

        goal = MoveServo.Goal()
        goal.command.id = int(frame_id)
        goal.command.pwm = int(pwm)
        goal.command.duration_ms = int(duration_ms)
        # verify 决定“结束后要不要读 PRAD 校验”；--no-verify 可关掉（板子不回包时用）。
        goal.verify_on_finish = bool(verify)

        # 异步发 goal 并挂 feedback 回调：同步 API 拿不到中间反馈。
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        # 被拒（超限位 / ID 不在名单 / 时长非法）返回 1，脚本能感知失败。
        if goal_handle is None or not goal_handle.accepted:
            print("goal rejected（超限 / ID 不在 servo_ids / duration 非法）")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        # 两层 result：外层是 rclpy 的包装对象，内层才是我们定义的 MoveServo.Result。
        result = result_future.result().result
        print(
            "result: success=%s prad_actual=%d message=%s"
            % (result.success, result.prad_actual, result.message)
        )
        return 0 if getattr(result, "success", False) else 1


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="给舵机发一条 MoveServo goal")
    parser.add_argument("--id", type=int, required=True, help="舵机 ID")
    parser.add_argument("--pwm", type=int, required=True, help="目标 PWM")
    # 默认 800 ms，但驱动会把 <1000 抬到 1000 并打 WARN（现场可演示这条日志）。
    parser.add_argument("--duration", type=int, default=800, help="用时 ms")
    parser.add_argument("--no-verify", action="store_true", help="不读 PRAD 校验")
    args = parser.parse_args(argv)

    rclpy.init()
    node = MoveClient()
    code = 1
    try:
        code = node.send(args.id, args.pwm, args.duration, not args.no_verify)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
