#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import yaml

from cquick_servo_interfaces.action import MoveServo


class SequencePlayer(Node):
    """Play YAML servo sequences through the MoveServo Action."""

    def __init__(self):
        super().__init__("sequence_player")

        default_file = (
            Path(get_package_share_directory("cquick_servo"))
            / "config"
            / "actions"
            / "validate_all.yaml"
        )

        self.declare_parameter(
            "action_file",
            str(default_file),
        )
        self.declare_parameter(
            "action_name",
            "/move_servo",
        )
        self.declare_parameter(
            "release_after_step",
            False,
        )

        self.client = ActionClient(
            self,
            MoveServo,
            str(
                self.get_parameter(
                    "action_name"
                ).value
            ),
        )

        self.recover_client = self.create_client(
            Trigger,
            "/servo/recover",
        )
        self.release_client = self.create_client(
            Trigger,
            "/servo/release_all",
        )

        self.heartbeat_publisher = self.create_publisher(
            Bool,
            "/servo/heartbeat",
            10,
        )
        self.heartbeat_timer = self.create_timer(
            0.2,
            self._publish_heartbeat,
        )

    def _publish_heartbeat(self):
        message = Bool()
        message.data = True
        self.heartbeat_publisher.publish(message)

    def _spin_future(
        self,
        future,
        timeout_sec=10.0,
    ):
        deadline = (
            datetime.now().timestamp()
            + timeout_sec
        )

        while (
            not future.done()
            and datetime.now().timestamp() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        return future.done()

    def _wait_for_service(
        self,
        client,
        timeout_sec=5.0,
    ):
        deadline = (
            datetime.now().timestamp()
            + timeout_sec
        )

        while (
            not client.service_is_ready()
            and datetime.now().timestamp() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        return client.service_is_ready()

    def _wait_for_action_server(
        self,
        timeout_sec=10.0,
    ):
        deadline = (
            datetime.now().timestamp()
            + timeout_sec
        )

        while (
            not self.client.server_is_ready()
            and datetime.now().timestamp() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        return self.client.server_is_ready()

    def _recover_watchdog(self):
        if not self._wait_for_service(
            self.recover_client,
            timeout_sec=5.0,
        ):
            self.get_logger().warning(
                "/servo/recover 服务不存在"
            )
            return False

        future = self.recover_client.call_async(
            Trigger.Request()
        )

        if not self._spin_future(future):
            self.get_logger().error(
                "看门狗恢复请求超时"
            )
            return False

        response = future.result()

        if response is None or not response.success:
            self.get_logger().error(
                "看门狗恢复失败: "
                f"{getattr(response, 'message', 'unknown')}"
            )
            return False

        self.get_logger().info(
            f"看门狗恢复: {response.message}"
        )
        return True

    def _release_all_torque(self):
        if not self._wait_for_service(
            self.release_client,
            timeout_sec=5.0,
        ):
            self.get_logger().error(
                "/servo/release_all 服务不存在"
            )
            return False

        future = self.release_client.call_async(
            Trigger.Request()
        )

        if not self._spin_future(future):
            self.get_logger().error(
                "释放舵机请求超时"
            )
            return False

        response = future.result()
        if response is None or not response.success:
            self.get_logger().error(
                "释放舵机失败: "
                f"{getattr(response, 'message', 'unknown')}"
            )
            return False

        self.get_logger().info(
            f"释放舵机: {response.message}"
        )
        return True

    def _load_sequence(self) -> dict[str, Any]:
        file_value = str(
            self.get_parameter(
                "action_file"
            ).value
        )
        action_file = Path(file_value)

        if not action_file.is_absolute():
            raise ValueError(
                "action_file 必须是绝对路径"
            )

        if not action_file.is_file():
            raise FileNotFoundError(
                str(action_file)
            )

        with action_file.open(
            "r",
            encoding="utf-8",
        ) as stream:
            document = yaml.safe_load(stream)

        if not isinstance(document, dict):
            raise ValueError(
                "YAML 顶层必须是字典"
            )

        steps = document.get("steps")

        if not isinstance(steps, list) or not steps:
            raise ValueError(
                "YAML 必须包含非空 steps 列表"
            )

        return document

    def _build_goal(
        self,
        raw_goal: dict[str, Any],
    ) -> MoveServo.Goal:
        for field in (
            "id",
            "pwm",
            "duration_ms",
        ):
            if field not in raw_goal:
                raise ValueError(
                    f"Goal 缺少字段: {field}"
                )

        goal = MoveServo.Goal()
        goal.id = int(raw_goal["id"])
        goal.pwm = int(raw_goal["pwm"])
        goal.duration_ms = int(
            raw_goal["duration_ms"]
        )

        if not 1 <= goal.id <= 254:
            raise ValueError(
                f"非法舵机 ID: {goal.id}"
            )

        if not 0 <= goal.pwm <= 65535:
            raise ValueError(
                f"非法 PWM: {goal.pwm}"
            )

        if not 0 <= goal.duration_ms <= 60000:
            raise ValueError(
                f"非法 duration_ms: {goal.duration_ms}"
            )

        return goal

    def _feedback_callback(
        self,
        servo_id: int,
        feedback_message,
    ):
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"FEEDBACK id={servo_id}, "
            f"pwm={feedback.current_pwm}, "
            f"progress={feedback.progress:.2f}"
        )

    def _send_step(
        self,
        step_index: int,
        step_name: str,
        raw_goals: list[dict[str, Any]],
    ) -> bool:
        goals = [
            self._build_goal(raw_goal)
            for raw_goal in raw_goals
        ]

        ids = [int(goal.id) for goal in goals]

        if len(ids) != len(set(ids)):
            raise ValueError(
                f"Step {step_index} 中同一舵机出现多次"
            )

        self.get_logger().info(
            f"========== STEP {step_index} "
            f"{step_name}: START =========="
        )

        send_futures = [
            (
                goal,
                self.client.send_goal_async(
                    goal,
                    feedback_callback=(
                        lambda feedback,
                        servo_id=int(goal.id): (
                            self._feedback_callback(
                                servo_id,
                                feedback,
                            )
                        )
                    ),
                ),
            )
            for goal in goals
        ]

        handles = []

        for goal, future in send_futures:
            if not self._spin_future(future):
                self.get_logger().error(
                    f"id={goal.id} 等待 Goal 接受超时"
                )
                return False

            handle = future.result()

            if handle is None or not handle.accepted:
                self.get_logger().error(
                    f"Step {step_index}: "
                    f"id={goal.id} Goal 被拒绝"
                )

                for _, accepted_handle in handles:
                    accepted_handle.cancel_goal_async()

                return False

            handles.append((int(goal.id), handle))

        result_futures = [
            (
                servo_id,
                handle,
                handle.get_result_async(),
            )
            for servo_id, handle in handles
        ]

        success = True

        for servo_id, handle, future in result_futures:
            if not self._spin_future(
                future,
                timeout_sec=30.0,
            ):
                self.get_logger().error(
                    f"id={servo_id} 等待 Result 超时"
                )
                success = False
                continue

            wrapped = future.result()

            if wrapped is None:
                self.get_logger().error(
                    f"id={servo_id} 没有收到 Result，"
                    f"goal_id={handle.goal_id.uuid}"
                )
                success = False
                continue

            result = wrapped.result
            timestamp = (
                self.get_clock()
                .now()
                .nanoseconds
            )

            self.get_logger().info(
                f"RESULT time_ns={timestamp}, "
                f"step={step_index}, "
                f"id={servo_id}, "
                f"status={wrapped.status}, "
                f"success={result.success}, "
                f"final_pwm={result.final_pwm}, "
                f"message={result.message}"
            )

            if (
                wrapped.status
                != GoalStatus.STATUS_SUCCEEDED
                or not result.success
            ):
                success = False

        self.get_logger().info(
            f"========== STEP {step_index} "
            f"{step_name}: "
            f"{'DONE' if success else 'FAILED'} =========="
        )
        return success

    def run(self) -> bool:
        try:
            document = self._load_sequence()
        except (
            OSError,
            ValueError,
            yaml.YAMLError,
        ) as exc:
            self.get_logger().error(
                f"动作 YAML 加载失败: {exc}"
            )
            return False

        if not self._wait_for_action_server():
            self.get_logger().error(
                "/move_servo Action 不存在"
            )
            return False

        if not self._recover_watchdog():
            return False

        release_after_step = bool(
            self.get_parameter(
                "release_after_step"
            ).value
        )
        if release_after_step:
            if not self._release_all_torque():
                return False

        steps = document["steps"]

        self.get_logger().info(
            f"开始播放: "
            f"{document.get('name', 'unnamed')}"
        )

        all_success = True
        failed_steps = []

        for step_index, step in enumerate(
            steps,
            start=1,
        ):

            if not isinstance(step, dict):
                message = f"Step {step_index} 不是字典"
                self.get_logger().error(message)
                all_success = False
                failed_steps.append(message)
                continue

            raw_goals = step.get("goals")

            if (
                not isinstance(raw_goals, list)
                or not raw_goals
            ):
                message = f"Step {step_index} 的 goals 为空"
                self.get_logger().error(message)
                all_success = False
                failed_steps.append(message)
                continue

            step_name = str(
                step.get(
                    "name",
                    f"step_{step_index}",
                )
            )

            try:
                step_ok = self._send_step(
                    step_index,
                    step_name,
                    raw_goals,
                )
            except Exception as exc:
                message = (
                    f"Step {step_index} {step_name} "
                    f"异常: {exc}"
                )
                self.get_logger().error(message)
                failed_steps.append(message)
                all_success = False
                continue

            if not step_ok:
                message = (
                    f"Step {step_index} {step_name} 失败"
                )
                self.get_logger().error(message)
                failed_steps.append(message)
                all_success = False

            if release_after_step:
                if not self._release_all_torque():
                    message = (
                        f"Step {step_index} 释放舵机失败"
                    )
                    self.get_logger().error(message)
                    failed_steps.append(message)
                    all_success = False

        if failed_steps:
            self.get_logger().error(
                f"动作组执行完成，但有 "
                f"{len(failed_steps)} 个 Step 失败"
            )
            for failed_step in failed_steps:
                self.get_logger().error(failed_step)
            return False

        self.get_logger().info(
            "全部动作组执行成功"
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()

    try:
        ok = node.run()
        raise SystemExit(0 if ok else 1)
    except KeyboardInterrupt:
        raise SystemExit(130)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
