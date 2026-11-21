import json
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cquick_servo.real_servo import RealServoBus
from cquick_servo.simulated_servo import SimulatedServoBus
from cquick_servo.watchdog import SafetyWatchdog
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand


class ServoDriver(Node):
    def __init__(self):
        super().__init__("servo_driver")

        declared_parameters = self.declare_parameters(
            namespace="",
            parameters=[
                ("port", Parameter.Type.STRING),
                ("baudrate", Parameter.Type.INTEGER),
                ("pwm_min", Parameter.Type.INTEGER),
                ("pwm_max", Parameter.Type.INTEGER),
                ("default_duration_ms", Parameter.Type.INTEGER),
                ("servo_ids", Parameter.Type.INTEGER_ARRAY),
                ("startup_reset", Parameter.Type.BOOL),
                ("simulate", Parameter.Type.BOOL),
                ("feedback_hz", Parameter.Type.DOUBLE),
                ("watchdog_pause_heartbeat", False),
            ],
        )

        for parameter in declared_parameters:
            self.get_logger().info(
                f"{parameter.name} = {parameter.value}"
            )

        self.add_on_set_parameters_callback(self.validate_parameters)

        servo_ids = list(self.get_parameter("servo_ids").value)
        self.servo_ids = set(servo_ids)
        self.simulate = self.get_parameter("simulate").value

        if self.simulate:
            self.bus = SimulatedServoBus(servo_ids, 1500)
            self.get_logger().info(
                f"模拟总线初始化完成：{self.bus.positions}"
            )
        else:
            port = self.get_parameter("port").value
            baudrate = self.get_parameter("baudrate").value
            self.bus = RealServoBus(port, baudrate)
            self.serial_link_was_down = not self.bus.reconnect()
            if self.serial_link_was_down:
                self.get_logger().warn(
                    f"串口 {port} 暂不可用：{self.bus.last_error}；将自动重连"
                )
            else:
                self.get_logger().info(
                    f"真实串口已打开：{port}，波特率 {baudrate}"
                )

        # Action 和 bag 回放都通过 send_command() 调用同一总线函数。
        self.bus_lock = threading.RLock()
        self.active_goals_lock = threading.Lock()
        self.active_goals = {}
        self.command_lock = threading.Lock()
        self.local_commands = []
        self.watchdog = SafetyWatchdog(self.watchdog_timeout)

        self.callback_group = ReentrantCallbackGroup()

        if not self.simulate:
            self.reconnect_timer = self.create_timer(
                1.0,
                self.reconnect_callback,
                callback_group=self.callback_group,
            )

        self.heartbeat_timer = self.create_timer(
            0.2,
            self.heartbeat_callback,
            callback_group=self.callback_group,
        )
        self.watchdog_reset_service = self.create_service(
            Trigger,
            "/servo/watchdog/reset",
            self.reset_watchdog,
            callback_group=self.callback_group,
        )

        self.command_pub = self.create_publisher(
            ServoCommand, "/servo/command", 10
        )
        self.state_pub = self.create_publisher(
            String, "/servo/state", 10
        )
        self.command_sub = self.create_subscription(
            ServoCommand,
            "/servo/command",
            self.command_callback,
            10,
            callback_group=self.callback_group,
        )

        self.action_server = ActionServer(
            self,
            MoveServo,
            "/servo/move",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group,
        )

        if self.get_parameter("startup_reset").value:
            neutral_pwm = 1500
            duration_ms = self.get_parameter("default_duration_ms").value
            for servo_id in servo_ids:
                try:
                    self.send_command(
                        servo_id, neutral_pwm, duration_ms, record=False
                    )
                except ConnectionError as exc:
                    self.get_logger().warn(
                        f"启动回中位未执行，串口断开：{exc}"
                    )
                    break

        self.watchdog.feed()
        self.watchdog.start()

    def reconnect_callback(self):
        with self.bus_lock:
            connected = self.bus.refresh_connection()
            if not connected and not self.serial_link_was_down:
                self.serial_link_was_down = True
                self.get_logger().warn(
                    f"串口 {self.bus.device} 已掉线：{self.bus.last_error}；"
                    "节点保持运行，每秒尝试重连"
                )
            if connected:
                return
            if self.bus.reconnect():
                self.serial_link_was_down = False
                self.get_logger().info(
                    f"串口 {self.bus.device} 已重新打开；"
                    "下一条动作会重新发送 PULR，不自动恢复旧动作"
                )

    def heartbeat_callback(self):
        if self.get_parameter("watchdog_pause_heartbeat").value:
            return
        self.publish_state(0, 0, "heartbeat")
        self.watchdog.feed()

    def watchdog_timeout(self):
        self.get_logger().error(
            "超过 1 秒没有 /servo/state 心跳；看门狗锁定，"
            "正在给所有舵机发送 PULK 卸力"
        )
        with self.bus_lock:
            for servo_id in sorted(self.servo_ids):
                try:
                    self.bus.disable_torque(servo_id)
                    self.get_logger().error(
                        f"舵机 {servo_id} 已发送 PULK 卸力"
                    )
                except Exception as exc:
                    self.get_logger().error(
                        f"舵机 {servo_id} PULK 发送失败：{exc}"
                    )
        self.publish_state(0, 0, "watchdog_tripped")

    def reset_watchdog(self, request, response):
        response.success = self.watchdog.reset()
        if response.success:
            response.message = "外部确认已收到；看门狗解锁"
            self.get_logger().warn(response.message)
        else:
            response.message = "心跳仍超时，不能解锁"
        return response

    def watchdog_abort_result(self, goal_handle, last_pwm):
        result = MoveServo.Result()
        result.success = False
        result.final_pwm = int(last_pwm)
        result.message = "看门狗已锁定，舵机已卸力"
        goal_handle.abort()
        return result

    def publish_state(self, servo_id, pwm, status):
        message = String()
        message.data = json.dumps(
            {
                "id": int(servo_id),
                "pwm": int(pwm),
                "status": status,
            },
            ensure_ascii=False,
        )
        self.state_pub.publish(message)

    def send_command(
        self, servo_id, target_pwm, duration_ms, record=True,
        goal_handle=None,
    ):
        """Send a command from an Action or bag playback to the bus."""
        with self.bus_lock:
            if goal_handle is not None and not self.is_current_goal(goal_handle):
                raise RuntimeError("目标已被新目标抢占")
            if self.watchdog.is_tripped():
                raise RuntimeError("看门狗已锁定，禁止发送动作")
            if self.simulate:
                self.bus.move_to(servo_id, target_pwm)
            else:
                self.bus.move_to(
                    servo_id, target_pwm, duration_ms
                )

            if record:
                command = ServoCommand()
                command.id = int(servo_id)
                command.pwm = int(target_pwm)
                command.duration_ms = int(duration_ms)

                # 本节点也订阅 /servo/command。记下刚发布的消息，
                # 避免收到自己的消息后重复发送串口指令。
                with self.command_lock:
                    now = time.monotonic()
                    self.local_commands = [
                        item for item in self.local_commands
                        if now - item[0] < 2.0
                    ]
                    self.local_commands.append(
                        (
                            now,
                            command.id,
                            command.pwm,
                            command.duration_ms,
                        )
                    )

                self.command_pub.publish(command)

            self.publish_state(
                servo_id, target_pwm, "command_sent"
            )

    def command_callback(self, command):
        if self.watchdog.is_tripped():
            self.get_logger().error(
                "看门狗已锁定，拒绝 /servo/command 动作"
            )
            return
        signature = (
            command.id,
            command.pwm,
            command.duration_ms,
        )

        with self.command_lock:
            now = time.monotonic()
            self.local_commands = [
                item for item in self.local_commands
                if now - item[0] < 2.0
            ]

            for index, item in enumerate(self.local_commands):
                if item[1:] == signature:
                    self.local_commands.pop(index)
                    return

        # 走到这里的是外部发布的命令，包括 ros2 bag play。
        if command.id not in self.servo_ids:
            self.get_logger().warn(
                f"拒绝回放命令：ID {command.id} 不存在"
            )
            return

        pwm_min = self.get_parameter("pwm_min").value
        pwm_max = self.get_parameter("pwm_max").value

        if not pwm_min <= command.pwm <= pwm_max:
            self.get_logger().warn(
                f"拒绝回放命令：PWM {command.pwm} "
                f"超出 {pwm_min}～{pwm_max}"
            )
            return

        if not 1 <= command.duration_ms <= 9999:
            self.get_logger().warn(
                "拒绝回放命令：duration_ms 必须在 1～9999 之间"
            )
            return

        try:
            self.send_command(
                command.id,
                command.pwm,
                command.duration_ms,
                record=False,
            )
            self.get_logger().info(
                f"回放命令已发送：ID={command.id}, "
                f"PWM={command.pwm}, "
                f"时长={command.duration_ms}ms"
            )
        except Exception as exc:
            self.get_logger().error(
                f"回放命令发送失败：{exc}"
            )

    def cancel_callback(self, goal_handle):
        self.get_logger().info("收到取消请求")
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        servo_id = goal_handle.request.id
        with self.active_goals_lock:
            previous = self.active_goals.get(servo_id)
            self.active_goals[servo_id] = goal_handle
        if previous is not None and previous.is_active:
            self.get_logger().warn(
                f"舵机 {servo_id} 新目标已接管，旧目标将返回 preempted"
            )
        goal_handle.execute()

    def is_current_goal(self, goal_handle):
        with self.active_goals_lock:
            return self.active_goals.get(goal_handle.request.id) is goal_handle

    def preempted_result(self, goal_handle, last_pwm):
        result = MoveServo.Result()
        result.success = False
        result.final_pwm = int(last_pwm)
        result.message = "preempted：新目标已接管，未回中位"
        goal_handle.abort()
        return result

    def canceled_result(self, goal_handle, last_pwm):
        if not self.is_current_goal(goal_handle):
            return self.preempted_result(goal_handle, last_pwm)
        result = MoveServo.Result()
        result.success = False
        result.final_pwm = int(last_pwm)

        if self.watchdog.is_tripped():
            result.message = "canceled：看门狗已卸力，未发送回中位动作"
        else:
            try:
                self.send_command(
                    goal_handle.request.id,
                    1500,
                    self.get_parameter("default_duration_ms").value,
                    goal_handle=goal_handle,
                )
                result.message = "canceled：已发送回中位指令"
            except Exception as exc:
                if not self.is_current_goal(goal_handle):
                    return self.preempted_result(goal_handle, last_pwm)
                result.message = (
                    f"canceled：回中位指令发送失败：{exc}"
                )

        if not self.is_current_goal(goal_handle):
            return self.preempted_result(goal_handle, last_pwm)
        goal_handle.canceled()
        return result

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        if not self.is_current_goal(goal_handle):
            return self.preempted_result(goal_handle, 0)
        if self.watchdog.is_tripped():
            return self.watchdog_abort_result(goal_handle, 0)
        duration_ms = request.duration_ms

        if duration_ms == 0:
            duration_ms = self.get_parameter(
                "default_duration_ms"
            ).value

        feedback_hz = self.get_parameter("feedback_hz").value

        if not 1 <= duration_ms <= 9999 or feedback_hz <= 0:
            goal_handle.abort()
            result = MoveServo.Result()
            result.success = False
            result.message = "动作时间或反馈频率非法"
            return result

        if not self.simulate:
            return self.execute_real(
                goal_handle, duration_ms, feedback_hz
            )

        with self.bus_lock:
            start_pwm = self.bus.read_position(request.id)
        start_time = time.monotonic()
        duration_sec = duration_ms / 1000.0

        while True:
            if not self.is_current_goal(goal_handle):
                return self.preempted_result(
                    goal_handle, self.bus.read_position(request.id)
                )
            if self.watchdog.is_tripped():
                return self.watchdog_abort_result(
                    goal_handle, self.bus.read_position(request.id)
                )
            if goal_handle.is_cancel_requested:
                return self.canceled_result(
                    goal_handle,
                    self.bus.read_position(request.id),
                )

            elapsed = time.monotonic() - start_time
            progress = min(
                elapsed / duration_sec, 1.0
            )
            current_pwm = round(
                start_pwm
                + (request.pwm - start_pwm) * progress
            )

            with self.bus_lock:
                if not self.is_current_goal(goal_handle):
                    return self.preempted_result(
                        goal_handle, self.bus.read_position(request.id)
                    )
                self.bus.move_to(request.id, current_pwm)

            self.publish_state(
                request.id, current_pwm, "moving"
            )

            feedback = MoveServo.Feedback()
            feedback.id = request.id
            feedback.current_pwm = int(current_pwm)
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)

            if progress >= 1.0:
                break

            time.sleep(
                min(
                    1.0 / feedback_hz,
                    max(0.0, duration_sec - elapsed),
                )
            )

        if goal_handle.is_cancel_requested:
            return self.canceled_result(
                goal_handle,
                self.bus.read_position(request.id),
            )

        # 模拟模式在运动结束时记录一条目标命令。
        if not self.is_current_goal(goal_handle):
            return self.preempted_result(
                goal_handle, self.bus.read_position(request.id)
            )
        try:
            self.send_command(
                request.id, request.pwm, duration_ms,
                goal_handle=goal_handle,
            )
        except RuntimeError:
            return self.preempted_result(
                goal_handle, self.bus.read_position(request.id)
            )

        result = MoveServo.Result()
        result.final_pwm = self.bus.read_position(
            request.id
        )
        result.success = (
            abs(result.final_pwm - request.pwm) <= 30
        )
        result.message = (
            "模拟运动完成"
            if result.success
            else "模拟位置未到位"
        )

        if not self.is_current_goal(goal_handle):
            return self.preempted_result(goal_handle, result.final_pwm)
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def execute_real(
        self, goal_handle, duration_ms, feedback_hz
    ):
        request = goal_handle.request
        result = MoveServo.Result()
        last_pwm = 0

        try:
            if not self.is_current_goal(goal_handle):
                return self.preempted_result(goal_handle, last_pwm)
            with self.bus_lock:
                last_pwm = self.bus.read_position(
                    request.id
                )

            if goal_handle.is_cancel_requested:
                return self.canceled_result(
                    goal_handle, last_pwm
                )

            self.send_command(
                request.id,
                request.pwm,
                duration_ms,
                goal_handle=goal_handle,
            )

            duration_sec = duration_ms / 1000.0
            start_time = time.monotonic()

            while True:
                if not self.is_current_goal(goal_handle):
                    return self.preempted_result(goal_handle, last_pwm)
                if self.watchdog.is_tripped():
                    return self.watchdog_abort_result(
                        goal_handle, last_pwm
                    )
                if goal_handle.is_cancel_requested:
                    return self.canceled_result(
                        goal_handle, last_pwm
                    )

                elapsed = time.monotonic() - start_time
                progress = min(
                    elapsed / duration_sec, 1.0
                )

                with self.bus_lock:
                    last_pwm = self.bus.read_position(request.id)

                feedback = MoveServo.Feedback()
                feedback.id = request.id
                feedback.current_pwm = int(last_pwm)
                feedback.progress = float(progress)
                goal_handle.publish_feedback(feedback)

                if progress >= 1.0:
                    break

                time.sleep(
                    min(
                        1.0 / feedback_hz,
                        max(0.0, duration_sec - elapsed),
                    )
                )

            if goal_handle.is_cancel_requested:
                return self.canceled_result(
                    goal_handle, last_pwm
                )

            if self.watchdog.is_tripped():
                return self.watchdog_abort_result(
                    goal_handle, last_pwm
                )

            if not self.is_current_goal(goal_handle):
                return self.preempted_result(goal_handle, last_pwm)
            time.sleep(0.2)

            if self.watchdog.is_tripped():
                return self.watchdog_abort_result(
                    goal_handle, last_pwm
                )

            if goal_handle.is_cancel_requested:
                return self.canceled_result(
                    goal_handle, last_pwm
                )

            with self.bus_lock:
                last_pwm = self.bus.read_position(
                    request.id
                )

            self.publish_state(
                request.id, last_pwm, "read_back"
            )

            if self.watchdog.is_tripped():
                return self.watchdog_abort_result(
                    goal_handle, last_pwm
                )

            if abs(last_pwm - request.pwm) > 30:
                if not self.is_current_goal(goal_handle):
                    return self.preempted_result(goal_handle, last_pwm)
                self.get_logger().warn(
                    f"舵机 {request.id} 未到位：目标 {request.pwm}，"
                    f"PRAD 读回 {last_pwm}；重发一次"
                )
                if goal_handle.is_cancel_requested:
                    return self.canceled_result(
                        goal_handle, last_pwm
                    )

                self.send_command(
                    request.id, request.pwm, duration_ms,
                    goal_handle=goal_handle,
                )
                retry_deadline = (
                    time.monotonic() + duration_sec + 0.2
                )
                while time.monotonic() < retry_deadline:
                    if not self.is_current_goal(goal_handle):
                        return self.preempted_result(goal_handle, last_pwm)
                    if self.watchdog.is_tripped():
                        return self.watchdog_abort_result(
                            goal_handle, last_pwm
                        )
                    if goal_handle.is_cancel_requested:
                        return self.canceled_result(
                            goal_handle, last_pwm
                        )
                    time.sleep(
                        min(
                            0.05,
                            max(0.0, retry_deadline - time.monotonic()),
                        )
                    )

                with self.bus_lock:
                    last_pwm = self.bus.read_position(request.id)
                if self.watchdog.is_tripped():
                    return self.watchdog_abort_result(
                        goal_handle, last_pwm
                    )
                self.publish_state(
                    request.id, last_pwm, "retry_read_back"
                )
                if abs(last_pwm - request.pwm) <= 30:
                    self.get_logger().info(
                        f"舵机 {request.id} 重发后到位："
                        f"PRAD 读回 {last_pwm}"
                    )
                else:
                    self.get_logger().error(
                        f"舵机 {request.id} 重发后仍未到位："
                        f"目标 {request.pwm}，PRAD 读回 {last_pwm}"
                    )

            if self.watchdog.is_tripped():
                return self.watchdog_abort_result(
                    goal_handle, last_pwm
                )

            if not self.is_current_goal(goal_handle):
                return self.preempted_result(goal_handle, last_pwm)

            result.final_pwm = int(last_pwm)
            result.success = (
                abs(last_pwm - request.pwm) <= 30
            )
            result.message = (
                "真实舵机已到位"
                if result.success
                else f"真实舵机未到位，读回 {last_pwm}"
            )

        except Exception as exc:
            if not self.is_current_goal(goal_handle):
                return self.preempted_result(goal_handle, last_pwm)
            result.success = False
            result.final_pwm = int(last_pwm)
            result.message = (
                f"串口或舵机通信失败：{exc}"
            )

        if not self.is_current_goal(goal_handle):
            return self.preempted_result(goal_handle, last_pwm)
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def goal_callback(self, request):
        if self.watchdog.is_tripped():
            self.get_logger().error(
                "看门狗已锁定，拒绝新 Action 目标"
            )
            return GoalResponse.REJECT
        if request.id not in self.servo_ids:
            self.get_logger().warn(
                f"拒绝目标：ID {request.id} 不存在"
            )
            return GoalResponse.REJECT

        pwm_min = self.get_parameter(
            "pwm_min"
        ).value
        pwm_max = self.get_parameter(
            "pwm_max"
        ).value

        if not pwm_min <= request.pwm <= pwm_max:
            self.get_logger().warn(
                f"拒绝目标：PWM {request.pwm} "
                f"超出 {pwm_min}～{pwm_max}"
            )
            return GoalResponse.REJECT

        duration_ms = request.duration_ms or self.get_parameter(
            "default_duration_ms"
        ).value
        if not 1 <= duration_ms <= 9999:
            self.get_logger().warn(
                f"拒绝目标：duration_ms {duration_ms} 必须在 1～9999 之间"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def validate_parameters(self, parameters):
        limits = {
            "pwm_min": self.get_parameter(
                "pwm_min"
            ).value,
            "pwm_max": self.get_parameter(
                "pwm_max"
            ).value,
        }

        for parameter in parameters:
            if parameter.name in limits:
                limits[parameter.name] = parameter.value
            if parameter.name == "baudrate":
                if parameter.value not in (9600, 19200, 38400, 57600, 115200):
                    return SetParametersResult(
                        successful=False,
                        reason="baudrate 不是支持的波特率",
                    )
                if parameter.value != self.get_parameter("baudrate").value:
                    return SetParametersResult(
                        successful=False,
                        reason="串口已打开；更换 baudrate 请重启节点",
                    )
            if parameter.name in (
                "port", "servo_ids", "simulate", "startup_reset"
            ):
                if parameter.value != self.get_parameter(parameter.name).value:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} 更改后需要重启节点",
                    )
            if parameter.name == "default_duration_ms":
                if not 1 <= parameter.value <= 9999:
                    return SetParametersResult(
                        successful=False,
                        reason="default_duration_ms 必须在 1～9999 之间",
                    )
            if parameter.name == "feedback_hz":
                if parameter.value < 5:
                    return SetParametersResult(
                        successful=False,
                        reason="feedback_hz 不能低于 5 Hz",
                    )

        if not 500 <= limits["pwm_min"] <= 2500:
            return SetParametersResult(
                successful=False,
                reason=(
                    "pwm_min 必须在 500～2500 之间"
                ),
            )

        if not 500 <= limits["pwm_max"] <= 2500:
            return SetParametersResult(
                successful=False,
                reason=(
                    "pwm_max 必须在 500～2500 之间"
                ),
            )

        if limits["pwm_min"] >= limits["pwm_max"]:
            return SetParametersResult(
                successful=False,
                reason=(
                    "pwm_min 必须小于 pwm_max"
                ),
            )

        return SetParametersResult(
            successful=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.watchdog.stop()
        executor.shutdown()
        if not node.simulate:
            with node.bus_lock:
                node.bus.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
