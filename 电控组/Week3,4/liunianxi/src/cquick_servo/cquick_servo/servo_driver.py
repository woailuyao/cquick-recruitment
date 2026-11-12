"""舵机驱动节点：Action Server + 串口。

对外只暴露 Action `/move_servo`（不用话题、不用服务当控制口）；
另外订阅 `/servo/command`（内部调试总线）让 bag 回放能复现动作，
发布 `/servo/state`（bag 对账 + 看门狗心跳）。

参数一个都不许写死：设备名、波特率、限位、ID 列表全部来自参数（见 config/servo_params.yaml）。
"""
from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from builtin_interfaces.msg import Time as TimeMsg
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand, ServoState

from .servo_link import PWM_ELECTRIC_MAX, PWM_ELECTRIC_MIN, SerialLink

FEEDBACK_PERIOD = 0.05      # 20 Hz，细则要求 >= 5 Hz
PRAD_TOLERANCE = 30         # 回读与目标差超过 30 算没到位
ALLOWED_BAUD = (9600, 19200, 38400, 57600, 115200, 230400)
CENTER_PWM = 1500           # 中立位（协议/物理约定，不是配置）
PRAD_EVERY = 4              # 每 4 个反馈周期读一次 PRAD（20Hz -> 5Hz），别把半双工总线读爆


class ServoDriver(Node):
    def __init__(self) -> None:
        super().__init__("servo_driver")

        # ---------- ① 声明参数（只声明，不在 __init__ 存副本！） ----------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("pwm_min", 900)
        self.declare_parameter("pwm_max", 2100)
        self.declare_parameter("default_duration_ms", 800)
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("startup_reset", True)

        # ---------- ② 参数校验：不注册这个回调，100000 也能进得去 ----------
        self.add_on_set_parameters_callback(self._on_param_set)

        self._cb_group = ReentrantCallbackGroup()
        self._link: Optional[SerialLink] = None
        self._torque_on_done: set[int] = set()
        self._tick = 0
        self._open_link()

        self._state_pub = self.create_publisher(ServoState, "/servo/state", 10)
        # 内部调试总线：Action 和它走同一个底层发送函数，bag 回放才能对得上
        self.create_subscription(
            ServoCommand, "/servo/command", self._on_command_msg, 10,
            callback_group=self._cb_group,
        )

        self._action_server = ActionServer(
            self,
            MoveServo,
            "move_servo",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            callback_group=self._cb_group,
        )

        if self.get_parameter("startup_reset").value:
            self._startup_reset()

        self.get_logger().info(
            "servo_driver 就绪：port=%s baudrate=%d 安全区间=[%d,%d] ids=%s"
            % (
                self.get_parameter("port").value,
                self.get_parameter("baudrate").value,
                self.get_parameter("pwm_min").value,
                self.get_parameter("pwm_max").value,
                list(self.get_parameter("servo_ids").value),
            )
        )

    # ------------------------------------------------------------------ 串口
    def _open_link(self) -> None:
        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        try:
            self._link = SerialLink(port, int(baud))
            self.get_logger().info(f"串口已打开：{port} @ {baud} 8-N-1")
        except Exception as exc:  # 板卡上没插线也要能起来，别直接崩
            self._link = None
            self.get_logger().error(f"串口打不开：{port} -> {exc}")

    def _ensure_link(self) -> bool:
        if self._link is not None and self._link.is_open:
            return True
        self.get_logger().warn("串口不可用，尝试重连…")
        self._open_link()
        return self._link is not None

    # -------------------------------------------------------------- 参数校验
    def _on_param_set(self, params: list[Parameter]):
        from rcl_interfaces.msg import SetParametersResult

        for p in params:
            if p.name in ("pwm_min", "pwm_max"):
                value = int(p.value)
                if not (PWM_ELECTRIC_MIN <= value <= PWM_ELECTRIC_MAX):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{p.name}={value} 超出电气范围 {PWM_ELECTRIC_MIN}~{PWM_ELECTRIC_MAX}",
                    )
            elif p.name == "baudrate":
                value = int(p.value)
                if value not in ALLOWED_BAUD:
                    return SetParametersResult(
                        successful=False,
                        reason=f"baudrate={value} 不是支持的值：{ALLOWED_BAUD}",
                    )
            elif p.name == "port":
                if not str(p.value).strip():
                    return SetParametersResult(successful=False, reason="port 不能为空")
            elif p.name == "default_duration_ms":
                if int(p.value) <= 0:
                    return SetParametersResult(successful=False, reason="duration 必须为正")
            elif p.name == "servo_ids":
                if len(list(p.value)) == 0:
                    return SetParametersResult(successful=False, reason="servo_ids 不能为空")

        # 组合校验：改完这一批之后 pwm_min 必须仍小于 pwm_max
        incoming = {p.name: p.value for p in params}
        pwm_min = int(incoming.get("pwm_min", self.get_parameter("pwm_min").value))
        pwm_max = int(incoming.get("pwm_max", self.get_parameter("pwm_max").value))
        if pwm_min >= pwm_max:
            return SetParametersResult(
                successful=False, reason=f"pwm_min({pwm_min}) 必须小于 pwm_max({pwm_max})"
            )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ 实时读参数
    # 注意：这里每次都 get_parameter(...).value，绝不读 __init__ 里的副本，
    # 否则 `ros2 param set` 改了不生效（这是 2.6 第 ④ 步失败的最常见原因）。
    def _pwm_limits(self) -> tuple[int, int]:
        return (
            int(self.get_parameter("pwm_min").value),
            int(self.get_parameter("pwm_max").value),
        )

    def _check_command(self, cmd: ServoCommand) -> Optional[str]:
        lo, hi = self._pwm_limits()
        ids = list(self.get_parameter("servo_ids").value)
        if int(cmd.id) not in ids:
            return f"id={cmd.id} 不在 servo_ids={ids} 里"
        if not (lo <= int(cmd.pwm) <= hi):
            return f"pwm={cmd.pwm} 超出安全区间 [{lo},{hi}]"
        if int(cmd.duration_ms) <= 0:
            return f"duration_ms={cmd.duration_ms} 必须为正"
        return None

    # ------------------------------------------------------------- 底层发送
    def _send_to_hardware(self, cmd: ServoCommand) -> bool:
        """Action 和 /servo/command 都走这一个函数——bag 回放才对得上。"""
        if not self._ensure_link():
            self._publish_state(cmd, 0, False, "serial_error")
            return False
        try:
            if int(cmd.id) not in self._torque_on_done:
                self._link.torque_on(int(cmd.id))   # 第一次动作之前必须 PULR
                self._torque_on_done.add(int(cmd.id))
            self._link.move(int(cmd.id), int(cmd.pwm), int(cmd.duration_ms))
            self.get_logger().info(
                f"发送 #{int(cmd.id):03d}P{int(cmd.pwm)}T{int(cmd.duration_ms)}!"
            )
            # 发出去就记一条 state：bag 里 /servo/state 才有东西可对账
            self._publish_state(cmd, 0, True, "sent")
            return True
        except Exception as exc:
            self.get_logger().error(f"串口写失败：{exc}")
            self._publish_state(cmd, 0, False, "serial_error")
            return False

    def _publish_state(self, cmd: ServoCommand, actual: int, ok: bool, note: str) -> None:
        msg = ServoState()
        now = self.get_clock().now().to_msg()
        msg.stamp = TimeMsg(sec=now.sec, nanosec=now.nanosec)
        msg.id = int(cmd.id)
        msg.pwm_target = int(cmd.pwm)
        msg.pwm_actual = int(actual)
        msg.ok = bool(ok)
        msg.note = note
        self._state_pub.publish(msg)

    def _read_pwm(self, frame_id: int, retry: int = 2) -> int:
        if self._link is None or not self._link.is_open:
            return 0
        for _ in range(max(1, retry)):
            value = self._link.read_pwm(frame_id)
            if value is not None and PWM_ELECTRIC_MIN <= int(value) <= PWM_ELECTRIC_MAX:
                return int(value)
            time.sleep(0.03)   # 半双工总线偶尔丢一帧，重问一次再判死
        return 0

    # ------------------------------------------------------------ 启动回中位
    def _startup_reset(self) -> None:
        if not self._ensure_link():
            return
        ids = list(self.get_parameter("servo_ids").value)
        duration = int(self.get_parameter("default_duration_ms").value)
        for frame_id in ids:
            try:
                self._link.torque_on(int(frame_id))
                self._link.move(int(frame_id), CENTER_PWM, duration)
            except Exception as exc:
                self.get_logger().warn(f"startup_reset id={frame_id} 失败：{exc}")
        self.get_logger().info(f"startup_reset：{len(ids)} 个舵机回中位 {CENTER_PWM}")

    # ------------------------------------------------------------- Action 回调
    def _goal_callback(self, goal_request: MoveServo.Goal):
        reason = self._check_command(goal_request.command)
        if reason is not None:
            self.get_logger().warn(f"goal rejected：{reason}")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"goal accepted：id={goal_request.command.id} pwm={goal_request.command.pwm}"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().warn("收到取消请求 -> ACCEPT")
        return CancelResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        # 标准做法：接受之后显式交给 executor 去跑 execute_callback
        goal_handle.execute()

    def _execute_callback(self, goal_handle):
        cmd = goal_handle.request.command
        result = MoveServo.Result()

        # 执行前再校验一次：参数可能在 goal 接受之后被改掉了
        reason = self._check_command(cmd)
        if reason is not None:
            goal_handle.abort()
            result.success = False
            result.message = f"out_of_range: {reason}"
            self._publish_state(cmd, 0, False, "out_of_range")
            return result

        if not self._send_to_hardware(cmd):
            goal_handle.abort()
            result.success = False
            result.message = "serial_error"
            return result

        # ---------- 循环发 feedback，同时每轮检查取消 ----------
        duration_s = max(int(cmd.duration_ms), 1) / 1000.0
        t0 = time.monotonic()
        # 起始点：能读到就用回读，读不到/读到离谱值就当中立位
        start_pwm = self._read_pwm(int(cmd.id))
        if not (PWM_ELECTRIC_MIN <= start_pwm <= PWM_ELECTRIC_MAX):
            start_pwm = CENTER_PWM
        canceled = False
        while True:
            elapsed = time.monotonic() - t0
            progress = min(elapsed / duration_s, 1.0)
            if goal_handle.is_cancel_requested:
                canceled = True
                break
            fb = MoveServo.Feedback()
            fb.pwm_current = int(start_pwm + (int(cmd.pwm) - start_pwm) * progress)
            fb.progress = float(progress)

            # 回读降频：PRAD 每 4 个周期（5 Hz）问一次，避免把半双工总线读爆
            self._tick += 1
            if self._tick % PRAD_EVERY == 0:
                readback = self._read_pwm(int(cmd.id))
                if PWM_ELECTRIC_MIN <= readback <= PWM_ELECTRIC_MAX:
                    fb.pwm_current = int(readback)

            goal_handle.publish_feedback(fb)

            if progress >= 1.0:
                break
            time.sleep(FEEDBACK_PERIOD)

        if canceled:
            # 取消 -> 立刻停发、回中位
            center = ServoCommand()
            center.id = cmd.id
            center.pwm = CENTER_PWM
            center.duration_ms = int(self.get_parameter("default_duration_ms").value)
            self._send_to_hardware(center)
            goal_handle.canceled()
            result.success = False
            result.prad_actual = 0
            result.message = "canceled"
            self._publish_state(cmd, 0, False, "canceled")
            self.get_logger().warn(f"动作被取消，id={cmd.id} 已回中位 {CENTER_PWM}")
            return result

        # ---------- 收尾：读 PRAD 校验 ----------
        actual = 0
        if goal_handle.request.verify_on_finish:
            time.sleep(0.15)   # 舵机走到位再读，别在运动末尾抢读（真机也一样）
            actual = self._read_pwm(int(cmd.id))
        result.prad_actual = int(actual)

        if actual == 0:
            # 真舵机可能不回包：不判死，但要在 message 里说清楚
            result.success = True
            result.message = "ok(no_readback)"
            self._publish_state(cmd, 0, True, "ok(no_readback)")
        else:
            diff = abs(int(actual) - int(cmd.pwm))
            if diff <= PRAD_TOLERANCE:
                result.success = True
                result.message = f"ok(diff={diff})"
                self._publish_state(cmd, actual, True, f"ok(diff={diff})")
            else:
                result.success = False
                result.message = f"not_reached:{diff}"
                self._publish_state(cmd, actual, False, f"not_reached:{diff}")

        goal_handle.succeed()
        self.get_logger().info(
            f"完成 id={int(cmd.id)} 目标={int(cmd.pwm)} 读回={int(actual)} -> {result.message}"
        )
        return result

    # ---------------------------------------------------- /servo/command 订阅
    def _on_command_msg(self, msg: ServoCommand) -> None:
        reason = self._check_command(msg)
        if reason is not None:
            self.get_logger().warn(f"/servo/command 被忽略：{reason}")
            self._publish_state(msg, 0, False, "out_of_range")
            return
        self._send_to_hardware(msg)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ServoDriver()
    # 必须多线程：单线程 + time.sleep 会把 cancel 回调饿死（细则 Q11）
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node._link is not None:
            node._link.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
