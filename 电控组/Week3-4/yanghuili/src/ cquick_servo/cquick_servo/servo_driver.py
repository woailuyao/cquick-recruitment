
#!/usr/bin/env python3

import json
import re
import threading
import time
from collections import deque
from typing import Optional

import rclpy
from cquick_servo.command_builder import build_move_command
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import serial
from serial.serialutil import SerialException
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class ServoDriver(Node):
    """ROS 2 Action server for the CQuick bus-servo controller."""

    CENTER_PWM = 1500
    POSITION_TOLERANCE = 30
    FEEDBACK_PERIOD_SEC = 0.2
    HEARTBEAT_TIMEOUT_SEC = 1.0

    def __init__(self):
        super().__init__("servo_driver")

        self.callback_group = ReentrantCallbackGroup()
        self.serial_lock = threading.RLock()
        self.watchdog_lock = threading.Lock()
        self.command_echo_lock = threading.Lock()
        self.pending_self_echoes = deque()

        self.ser: Optional[serial.Serial] = None
        self.serial_connected = False
        self.last_heartbeat = time.monotonic()
        self.released = False

        self._declare_parameters()
        self.add_on_set_parameters_callback(
            self._on_set_parameters
        )

        self.state_publisher = self.create_publisher(
            String,
            "/servo/state",
            10,
        )

        self.command_publisher = self.create_publisher(
            ServoCommand,
            "/servo/command",
            10,
        )
        self.command_subscription = self.create_subscription(
            ServoCommand,
            "/servo/command",
            self._command_callback,
            10,
            callback_group=self.callback_group,
        )

        self.heartbeat_subscription = self.create_subscription(
            Bool,
            "/servo/heartbeat",
            self._heartbeat_callback,
            10,
            callback_group=self.callback_group,
        )

        self.recover_service = self.create_service(
            Trigger,
            "/servo/recover",
            self._recover_callback,
            callback_group=self.callback_group,
        )
        self.release_all_service = self.create_service(
            Trigger,
            "/servo/release_all",
            self._release_all_callback,
            callback_group=self.callback_group,
        )

        self._open_serial()

        self.watchdog_timer = self.create_timer(
            1.0,
            self._watchdog_check,
            callback_group=self.callback_group,
        )
        self.reconnect_timer = self.create_timer(
            1.0,
            self._reconnect_check,
            callback_group=self.callback_group,
        )
        self.state_timer = self.create_timer(
            1.0,
            self._publish_heartbeat,
            callback_group=self.callback_group,
        )

        if self.serial_connected:
            if bool(self.get_parameter("startup_reset").value):
                self.reset_all()
            else:
                # Calibration mode: keep all servos released until a goal
                # explicitly asks one servo to move.
                for servo_id in self.get_parameter(
                    "servo_ids"
                ).value:
                    self._disable_torque(int(servo_id))

        self.action_server = ActionServer(
            self,
            MoveServo,
            "/move_servo",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "servo_driver ready: "
            f"port={self.get_parameter('port').value}, "
            f"ids={list(self.get_parameter('servo_ids').value)}"
        )

    def _declare_parameters(self):
        self.declare_parameter("port", "/dev/cq_servo")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("pwm_min", 900)
        self.declare_parameter("pwm_max", 2100)
        self.declare_parameter("default_duration_ms", 800)
        self.declare_parameter(
            "servo_ids",
            [
                101,
                102,
                103,
                104,
                105,
                106,
                107,
                108,
                109,
                110,
            ],
        )
        self.declare_parameter("startup_reset", True)

    def _on_set_parameters(self, parameters):
        current = {
            name: self.get_parameter(name).value
            for name in (
                "port",
                "baudrate",
                "pwm_min",
                "pwm_max",
                "default_duration_ms",
                "servo_ids",
                "startup_reset",
            )
        }
        proposed = {
            parameter.name: parameter.value
            for parameter in parameters
        }
        effective = {**current, **proposed}

        if (
            "port" in proposed
            and proposed["port"] != current["port"]
        ):
            return SetParametersResult(
                successful=False,
                reason="port changes require restarting the driver",
            )
        if (
            "baudrate" in proposed
            and proposed["baudrate"] != current["baudrate"]
        ):
            return SetParametersResult(
                successful=False,
                reason="baudrate changes require restarting the driver",
            )

        pwm_min = int(effective["pwm_min"])
        pwm_max = int(effective["pwm_max"])

        if not 500 <= pwm_min <= 2500:
            return SetParametersResult(
                successful=False,
                reason="pwm_min must be in 500..2500",
            )
        if not 500 <= pwm_max <= 2500:
            return SetParametersResult(
                successful=False,
                reason="pwm_max must be in 500..2500",
            )
        if pwm_min >= pwm_max:
            return SetParametersResult(
                successful=False,
                reason="pwm_min must be less than pwm_max",
            )

        duration_ms = int(effective["default_duration_ms"])
        if not 1 <= duration_ms <= 60000:
            return SetParametersResult(
                successful=False,
                reason="default_duration_ms must be in 1..60000",
            )

        servo_ids = [
            int(value)
            for value in effective["servo_ids"]
        ]
        if not servo_ids:
            return SetParametersResult(
                successful=False,
                reason="servo_ids must not be empty",
            )
        if len(servo_ids) != len(set(servo_ids)):
            return SetParametersResult(
                successful=False,
                reason="servo_ids must not contain duplicates",
            )
        if any(
            not 1 <= servo_id <= 254
            for servo_id in servo_ids
        ):
            return SetParametersResult(
                successful=False,
                reason="every servo id must be in 1..254",
            )

        supported_baudrates = {
            9600,
            19200,
            38400,
            57600,
            115200,
            230400,
        }
        if int(effective["baudrate"]) not in supported_baudrates:
            return SetParametersResult(
                successful=False,
                reason=(
                    "unsupported baudrate: "
                    f"{effective['baudrate']}"
                ),
            )

        return SetParametersResult(successful=True)

    def _open_serial(self) -> bool:
        port = str(self.get_parameter("port").value)
        baudrate = int(
            self.get_parameter("baudrate").value
        )

        with self.serial_lock:
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05,
                    write_timeout=0.2,
                )
                self.serial_connected = True
                self.get_logger().info(
                    f"串口已打开: {port} @ {baudrate}"
                )
                return True
            except SerialException as exc:
                self.ser = None
                self.serial_connected = False
                self.get_logger().warning(
                    f"串口打开失败: {exc}"
                )
                return False

    def _handle_serial_lost(self, exc):
        with self.serial_lock:
            self.serial_connected = False
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.get_logger().warning(
            f"串口断开: {exc}，等待重新连接"
        )

    def _reconnect_check(self):
        if self.serial_connected:
            return

        self.get_logger().warning("尝试重新连接串口")
        if not self._open_serial():
            return

        if not self.released:
            if bool(self.get_parameter("startup_reset").value):
                for servo_id in self.get_parameter(
                    "servo_ids"
                ).value:
                    self._enable_torque(int(servo_id))
            else:
                for servo_id in self.get_parameter(
                    "servo_ids"
                ).value:
                    self._disable_torque(int(servo_id))

        self.get_logger().info("串口重连成功")

    def _safe_write(self, command: str):
        with self.serial_lock:
            if not self.serial_connected or self.ser is None:
                raise SerialException("串口未连接")

            try:
                self.ser.write(
                    (
                        command.rstrip("\r\n") + "\r\n"
                    ).encode("ascii")
                )
                self.ser.flush()
            except SerialException as exc:
                self._handle_serial_lost(exc)
                raise

    def _safe_read(self) -> str:
        with self.serial_lock:
            if not self.serial_connected or self.ser is None:
                raise SerialException("串口未连接")

            deadline = time.monotonic() + 0.15
            data = bytearray()

            try:
                while time.monotonic() < deadline:
                    waiting = self.ser.in_waiting
                    chunk = self.ser.read(
                        waiting if waiting > 0 else 1
                    )
                    if chunk:
                        data.extend(chunk)
                        if b"!" in data:
                            break
                    else:
                        time.sleep(0.005)
            except SerialException as exc:
                self._handle_serial_lost(exc)
                raise

            return data.decode(
                "ascii",
                errors="replace",
            ).strip()

    def _enable_torque(self, servo_id: int):
        try:
            self._safe_write(
                f"#{int(servo_id):03d}PULR!"
            )
            # Give the servo controller a short time to apply torque before
            # sending a position command, especially after a preceding PULK.
            time.sleep(0.5)
        except SerialException:
            pass

    def _disable_torque(self, servo_id: int):
        try:
            self._safe_write(
                f"#{int(servo_id):03d}PULK!"
            )
        except SerialException:
            pass

    def _read_position_once(
        self,
        servo_id: int,
    ) -> Optional[int]:
        with self.serial_lock:
            try:
                self.ser.reset_input_buffer()
                self._safe_write(
                    f"#{int(servo_id):03d}PRAD!"
                )
                response = self._safe_read()
            except SerialException as exc:
                self.get_logger().warning(
                    f"PRAD 读取失败: {exc}"
                )
                return None

        match = re.fullmatch(
            r"#\d{3}P(\d+)!",
            response,
        )
        if match is None:
            self.get_logger().warning(
                f"PRAD 回复异常: {response!r}"
            )
            return None

        value = int(match.group(1))
        return value if 0 <= value <= 65535 else None

    def read_position_retry(
        self,
        servo_id: int,
    ) -> Optional[int]:
        """读位置，失败等 20ms 重读，最多 3 次"""
        for attempt in range(3):
            value = self._read_position_once(servo_id)
            if value is not None:
                return value
            if attempt < 2:
                time.sleep(0.02)
        return None


    def _publish_state(self, event: str, **fields):
        message = String()
        message.data = json.dumps(
            {
                "event": event,
                "stamp": time.time(),
                **fields,
            },
            ensure_ascii=False,
        )
        self.state_publisher.publish(message)

    def _publish_heartbeat(self):
        self._publish_state(
            "heartbeat",
            serial_connected=self.serial_connected,
            released=self.released,
        )

    def _heartbeat_callback(self, message: Bool):
        if bool(message.data):
            self.last_heartbeat = time.monotonic()

    def _watchdog_check(self):
        if self.released:
            return

        if (
            time.monotonic() - self.last_heartbeat
            > self.HEARTBEAT_TIMEOUT_SEC
        ):
            self._release_all(
                "more than 1 second without heartbeat"
            )

    def _release_all(self, reason: str):
        with self.watchdog_lock:
            if self.released:
                return
            self.released = True

        for servo_id in self.get_parameter(
            "servo_ids"
        ).value:
            self._disable_torque(int(servo_id))

        self.get_logger().error(
            f"安全看门狗已卸力: {reason}"
        )
        self._publish_state(
            "watchdog_released",
            reason=reason,
        )

    def _recover_callback(self, request, response):
        del request

        if not self.serial_connected:
            response.success = False
            response.message = "串口未连接，不能恢复"
            return response

        with self.watchdog_lock:
            self.released = False
            self.last_heartbeat = time.monotonic()

        for servo_id in self.get_parameter(
            "servo_ids"
        ).value:
            try:
                self._safe_write(
                    f"#{int(servo_id):03d}PULR!"
                )
            except SerialException:
                response.success = False
                response.message = "恢复扭力失败"
                return response

        response.success = True
        response.message = "看门狗已恢复"
        self.get_logger().info(response.message)
        return response

    def _release_all_callback(self, request, response):
        del request

        if not self.serial_connected:
            response.success = False
            response.message = "串口未连接，不能释放舵机"
            return response

        for servo_id in self.get_parameter(
            "servo_ids"
        ).value:
            self._disable_torque(int(servo_id))

        response.success = True
        response.message = "全部舵机已发送 PULK，但未进入看门狗锁定"
        self.get_logger().info(response.message)
        return response

    def _effective_duration(self, duration_ms: int) -> int:
        if int(duration_ms) == 0:
            return int(
                self.get_parameter(
                    "default_duration_ms"
                ).value
            )
        return int(duration_ms)

    def _validate_target(
        self,
        servo_id: int,
        pwm: int,
        duration_ms: int,
    ) -> Optional[str]:
        ids = {
            int(value)
            for value in self.get_parameter(
                "servo_ids"
            ).value
        }
        pwm_min = int(
            self.get_parameter("pwm_min").value
        )
        pwm_max = int(
            self.get_parameter("pwm_max").value
        )

        if int(servo_id) not in ids:
            return (
                f"id {servo_id} is not in servo_ids "
                f"{sorted(ids)}"
            )
        if not pwm_min <= int(pwm) <= pwm_max:
            return (
                f"pwm {pwm} is outside safe range "
                f"{pwm_min}..{pwm_max}"
            )
        if (
            int(duration_ms) != 0
            and not 1 <= int(duration_ms) <= 60000
        ):
            return (
                f"duration_ms {duration_ms} "
                "is outside 1..60000"
            )
        return None

    def _move(
        self,
        servo_id: int,
        pwm: int,
        duration_ms: int,
        publish_command: bool = True,
    ):
        if self.released:
            raise ValueError(
                "看门狗已卸力，必须调用 /servo/recover"
            )

        reason = self._validate_target(
            servo_id,
            pwm,
            duration_ms,
        )
        if reason is not None:
            raise ValueError(reason)

        command = build_move_command(
            servo_id,
            pwm,
            duration_ms,
        )
        self._enable_torque(servo_id)
        self._safe_write(command)

        if publish_command:
            message = ServoCommand()
            message.id = int(servo_id)
            message.pwm = int(pwm)
            message.duration_ms = int(duration_ms)

            identity = (
                int(servo_id),
                int(pwm),
                int(duration_ms),
            )
            with self.command_echo_lock:
                self.pending_self_echoes.append(identity)

            self.command_publisher.publish(message)

        self._publish_state(
            "move",
            id=int(servo_id),
            pwm=int(pwm),
            duration_ms=int(duration_ms),
        )
        self.get_logger().info(f"发送: {command}")

    def _command_callback(self, message: ServoCommand):
        identity = (
            int(message.id),
            int(message.pwm),
            int(message.duration_ms),
        )
        with self.command_echo_lock:
            if identity in self.pending_self_echoes:
                self.pending_self_echoes.remove(identity)
                return

        if self.released:
            self.get_logger().warning(
                "拒绝 /servo/command: 看门狗已卸力"
            )
            return

        duration_ms = self._effective_duration(
            message.duration_ms
        )
        try:
            self._move(
                int(message.id),
                int(message.pwm),
                duration_ms,
                publish_command=False,
            )
        except (SerialException, ValueError) as exc:
            self.get_logger().error(
                f"bag 回放命令失败: {exc}"
            )

    def reset_all(self):
        for servo_id in self.get_parameter(
            "servo_ids"
        ).value:
            try:
                self._move(
                    int(servo_id),
                    self.CENTER_PWM,
                    int(
                        self.get_parameter(
                            "default_duration_ms"
                        ).value
                    ),
                )
            except (SerialException, ValueError):
                return

    def goal_callback(self, goal_request):
        if self.released:
            self.get_logger().warning(
                "Goal rejected: 看门狗已卸力"
            )
            return GoalResponse.REJECT

        reason = self._validate_target(
            goal_request.id,
            goal_request.pwm,
            goal_request.duration_ms,
        )
        if reason is not None:
            self.get_logger().warning(
                f"Goal rejected: {reason}"
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            "Goal accepted: "
            f"id={goal_request.id}, "
            f"pwm={goal_request.pwm}, "
            f"duration_ms="
            f"{self._effective_duration(goal_request.duration_ms)}"
        )
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().warning(
            f"取消请求: id={goal_handle.request.id}"
        )
        return CancelResponse.ACCEPT

    def _cancel_and_neutralize(
        self,
        goal_handle,
        servo_id: int,
    ):
        self._disable_torque(servo_id)

        if not self.released:
            try:
                self._move(
                    servo_id,
                    self.CENTER_PWM,
                    300,
                )
            except (SerialException, ValueError):
                pass

        self._publish_state(
            "canceled",
            id=int(servo_id),
            message="canceled",
        )
        goal_handle.canceled()

        result = MoveServo.Result()
        result.success = False
        result.final_pwm = 0
        result.message = "canceled"
        return result

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        servo_id = int(request.id)
        target_pwm = int(request.pwm)
        duration_ms = self._effective_duration(
            request.duration_ms
        )
        result = MoveServo.Result()

        if self.released:
            goal_handle.abort()
            result.success = False
            result.final_pwm = 0
            result.message = (
                "看门狗已卸力，必须调用 /servo/recover"
            )
            return result

        if not self.serial_connected:
            goal_handle.abort()
            result.success = False
            result.final_pwm = 0
            result.message = "serial disconnected"
            return result

        try:
            start_pwm = self.read_position_retry(servo_id)
            if start_pwm is None:
                start_pwm = target_pwm

            self._move(
                servo_id,
                target_pwm,
                duration_ms,
            )

            start_time = time.monotonic()
            duration_sec = duration_ms / 1000.0
            deadline = start_time + duration_sec
            next_feedback = start_time

            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    return self._cancel_and_neutralize(
                        goal_handle,
                        servo_id,
                    )

                if self.released:
                    goal_handle.abort()
                    result.success = False
                    result.final_pwm = 0
                    result.message = "watchdog released"
                    return result

                now = time.monotonic()
                progress = min(
                    1.0,
                    (now - start_time) / duration_sec,
                )
                estimated_pwm = round(
                    start_pwm
                    + (
                        target_pwm - start_pwm
                    ) * progress
                )
                measured_pwm = self.read_position_retry(
                    servo_id
                )

                feedback = MoveServo.Feedback()
                feedback.id = servo_id
                feedback.current_pwm = (
                    measured_pwm
                    if measured_pwm is not None
                    else estimated_pwm
                )
                feedback.progress = float(progress)
                goal_handle.publish_feedback(feedback)
                self._publish_state(
                    "feedback",
                    id=servo_id,
                    pwm=int(feedback.current_pwm),
                    progress=float(progress),
                )

                next_feedback += self.FEEDBACK_PERIOD_SEC
                time.sleep(
                    max(
                        0.0,
                        next_feedback - time.monotonic(),
                    )
                )

            if goal_handle.is_cancel_requested:
                return self._cancel_and_neutralize(
                    goal_handle,
                    servo_id,
                )

            final_pwm = self.read_position_retry(servo_id)
            if final_pwm is None:
                goal_handle.abort()
                result.success = False
                result.final_pwm = 0
                result.message = (
                    "PRAD 读取失败，无法确认最终位置"
                )
                self._publish_state(
                    "result",
                    id=servo_id,
                    success=False,
                    message=result.message,
                )
                return result

            difference = abs(
                target_pwm - final_pwm
            )
            result.final_pwm = final_pwm

            if difference > self.POSITION_TOLERANCE:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"未到位: 目标{target_pwm}, "
                    f"读回{final_pwm}, 差{difference}"
                )
            else:
                goal_handle.succeed()
                result.success = True
                result.message = f"ok, 误差{difference}"

            self._publish_state(
                "result",
                id=servo_id,
                success=bool(result.success),
                final_pwm=int(result.final_pwm),
                message=result.message,
            )
            return result

        except (SerialException, ValueError) as exc:
            self.get_logger().error(
                f"Action 执行失败: {exc}"
            )
            self._disable_torque(servo_id)
            goal_handle.abort()

            result.success = False
            result.final_pwm = 0
            result.message = f"error: {exc}"
            self._publish_state(
                "result",
                id=servo_id,
                success=False,
                message=result.message,
            )
            return result


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node.ser is not None:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
