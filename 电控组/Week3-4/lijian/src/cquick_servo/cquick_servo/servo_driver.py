"""总线舵机驱动节点（任务 2 + 加分项）。

串口协议（8-N-1，波特率见 baudrate 参数，指令以 CRLF 结尾）：
    #001P1500T1000!   移动到 1500，用时 1000 ms
    #001PULR!         恢复扭力
    #001PULK!         卸力
    #001PRAD!         读回当前位置

加分项：
    * PRAD 闭环校验 + 未到位自动重发一次
    * 安全看门狗：watchdog_enabled=true 时，空闲超时 PULK 并锁死，/servo/recover 恢复
    * 动作组抢占：同一舵机的新 goal 打断旧 goal
    * 串口掉线自动重连
    * udev 固定设备名（系统层，见 README）
"""

import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand


def build_move_cmd(servo_id: int, pwm: int, duration_ms: int) -> str:
    """拼一条移动指令，例如 #001P1700T1000!"""
    return f"#{servo_id:03d}P{pwm:04d}T{duration_ms:04d}!"


class ServoDriver(Node):
    def __init__(self):
        super().__init__("servo_driver")

        # ---------------- 参数 ----------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("pwm_min", 900)
        self.declare_parameter("pwm_max", 2100)
        self.declare_parameter("default_duration_ms", 800)
        self.declare_parameter("servo_ids", [1, 2, 3, 4])
        self.declare_parameter("startup_reset", True)
        self.declare_parameter("watchdog_enabled", False)
        self.declare_parameter("watchdog_timeout_s", 5.0)

        # ---------------- 运行状态 ----------------
        self._serial_lock = threading.Lock()
        self.ser = None

        self.is_locked = False
        self.last_cmd_time = time.time()
        self._last_reconnect = 0.0
        self._active = {}
        self._active_lock = threading.Lock()

        self._cb_group = ReentrantCallbackGroup()
        self._open_serial()

        # ---------------- 接口 ----------------
        self._action_server = ActionServer(
            self,
            MoveServo,
            "move_servo",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.create_subscription(
            ServoCommand,
            "/servo/command",
            self.command_callback,
            10,
            callback_group=self._cb_group,
        )

        self._cmd_pub = self.create_publisher(ServoCommand, "/servo/command", 10)
        self._state_pub = self.create_publisher(ServoCommand, "/servo/state", 10)

        self._current_state = ServoCommand()
        self._current_state.id = 0
        self._current_state.pwm = 1500
        self._current_state.duration_ms = 0

        self.create_timer(0.5, self.publish_state)
        self.create_timer(0.1, self.watchdog_loop)
        self.add_on_set_parameters_callback(self.param_callback)
        self.create_service(Trigger, "/servo/recover", self.recover_callback)

        if self.get_parameter("startup_reset").value:
            self.startup_reset()

        self.get_logger().info("servo_driver ready (bonus 版)")

    # ================= 串口 =================
    def _open_serial(self):
        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        try:
            self.ser = serial.Serial(
                port, baud,
                bytesize=8, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.3,
            )
            self.get_logger().info(f"serial opened: {port} @ {baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"open serial failed: {e}")

    def _raw_write(self, cmd: str) -> bool:
        if self.ser is None:
            self.get_logger().error("serial not open")
            return False
        with self._serial_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd.encode("ascii") + b"\r\n")
                self.ser.flush()
                self.last_cmd_time = time.time()
                return True
            except Exception as e:
                self.get_logger().error(f"serial write failed: {e}")
                self.ser = None
                return False

    # ================= 协议 =================
    def send_move(self, servo_id: int, pwm: int, duration_ms: int) -> bool:
        return self._raw_write(build_move_cmd(servo_id, pwm, duration_ms))

    def send_pulr(self, servo_id: int):
        return self._raw_write(f"#{servo_id:03d}PULR!")

    def read_prad(self, servo_id: int):
        if self.ser is None:
            return None
        with self._serial_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(f"#{servo_id:03d}PRAD!\r\n".encode("ascii"))
                self.ser.flush()
                time.sleep(0.15)
                data = self.ser.read(64)
                self.last_cmd_time = time.time()
            except Exception as e:
                self.get_logger().error(f"read_prad failed: {e}")
                self.ser = None
                return None
        if not data:
            return None
        text = data.decode("ascii", errors="ignore").strip()
        try:
            if text.startswith("#") and "P" in text:
                return int(text.split("P", 1)[1].rstrip("!"))
        except (ValueError, IndexError):
            pass
        return None

    # ================= 参数校验 =================
    def param_callback(self, params):
        for p in params:
            if p.name == "pwm_min":
                v = p.value
                if not isinstance(v, int) or not (500 <= v <= 2500):
                    return SetParametersResult(
                        successful=False, reason=f"pwm_min={v} 超出 500~2500")
                if v >= self.get_parameter("pwm_max").value:
                    return SetParametersResult(
                        successful=False, reason="pwm_min 必须小于 pwm_max")
            elif p.name == "pwm_max":
                v = p.value
                if not isinstance(v, int) or not (500 <= v <= 2500):
                    return SetParametersResult(
                        successful=False, reason=f"pwm_max={v} 超出 500~2500")
                if v <= self.get_parameter("pwm_min").value:
                    return SetParametersResult(
                        successful=False, reason="pwm_max 必须大于 pwm_min")
            elif p.name == "baudrate":
                v = p.value
                if v not in (9600, 19200, 38400, 57600, 115200):
                    return SetParametersResult(
                        successful=False, reason=f"不支持 baudrate={v}")
            elif p.name == "watchdog_timeout_s":
                v = p.value
                if not isinstance(v, (int, float)) or isinstance(v, bool) or v <= 0:
                    return SetParametersResult(
                        successful=False, reason="watchdog_timeout_s 必须 > 0")
            elif p.name == "watchdog_enabled":
                if not isinstance(p.value, bool):
                    return SetParametersResult(
                        successful=False, reason="watchdog_enabled 必须是 bool")
                if p.value:
                    self.last_cmd_time = time.time()
        return SetParametersResult(successful=True)

    # ================= Action =================
    def goal_callback(self, goal_request):
        if self.is_locked:
            self.get_logger().warn(
                "看门狗已锁死，拒绝新动作！请调用 /servo/recover 恢复")
            return GoalResponse.REJECT

        pwm_min = self.get_parameter("pwm_min").value
        pwm_max = self.get_parameter("pwm_max").value
        servo_ids = self.get_parameter("servo_ids").value

        if goal_request.id not in servo_ids:
            self.get_logger().warn(
                f"reject: id={goal_request.id} 不在 servo_ids={servo_ids}")
            return GoalResponse.REJECT
        if not (pwm_min <= goal_request.pwm <= pwm_max):
            self.get_logger().warn(
                f"reject: pwm={goal_request.pwm} 不在 [{pwm_min},{pwm_max}]")
            return GoalResponse.REJECT
        if goal_request.duration_ms > 10000:
            self.get_logger().warn("reject: duration_ms 超过 10000")
            return GoalResponse.REJECT

        # 加分项：抢占，只打断同一舵机的旧 goal
        with self._active_lock:
            old = self._active.get(goal_request.id)
        if old is not None and old.is_active:
            self.get_logger().warn(
                f"抢占触发！id={goal_request.id} 打断旧动作，从当前位置接上")
            try:
                old.abort()
            except Exception:
                pass
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        servo_id = goal.id
        target_pwm = goal.pwm
        duration_ms = goal.duration_ms
        if duration_ms == 0:
            duration_ms = self.get_parameter("default_duration_ms").value

        with self._active_lock:
            self._active[servo_id] = goal_handle

        result = MoveServo.Result()
        result.success = False
        result.final_pwm = 0
        result.message = ""

        try:
            start_pwm = self.read_prad(servo_id)
            if start_pwm is None:
                start_pwm = 1500
                self.get_logger().warn("起始 PRAD 读失败，假定 1500")

            self.send_pulr(servo_id)
            time.sleep(0.05)

            if not goal_handle.is_active:
                result.message = "preempted"
                return result

            duration_s = max(0.001, duration_ms / 1000.0)
            final_pwm = 0
            diff = 9999

            for attempt in range(2):
                self._cmd_pub.publish(ServoCommand(
                    id=servo_id, pwm=target_pwm, duration_ms=duration_ms))

                if not self.send_move(servo_id, target_pwm, duration_ms):
                    result.message = "serial error"
                    if goal_handle.is_active:
                        goal_handle.abort()
                    return result

                t0 = time.time()
                while True:
                    if not goal_handle.is_active:
                        result.message = "preempted"
                        return result
                    if goal_handle.is_cancel_requested:
                        self.send_move(servo_id, 1500, 300)
                        time.sleep(0.35)
                        self._raw_write(f"#{servo_id:03d}PULK!")
                        result.message = "canceled"
                        goal_handle.canceled()
                        return result

                    progress = min(1.0, (time.time() - t0) / duration_s)
                    current_pwm = int(start_pwm + (target_pwm - start_pwm) * progress)
                    if progress >= 1.0:
                        current_pwm = target_pwm

                    self._current_state.id = servo_id
                    self._current_state.pwm = current_pwm

                    try:
                        goal_handle.publish_feedback(MoveServo.Feedback(
                            current_pwm=current_pwm, progress=float(progress)))
                    except Exception:
                        result.message = "preempted"
                        return result

                    if progress >= 1.0:
                        break
                    time.sleep(0.1)

                time.sleep(0.2)
                final_pwm = self.read_prad(servo_id)
                if final_pwm is None:
                    final_pwm = 0
                    diff = 9999
                    self.get_logger().warn("PRAD 读回失败，按未到位处理")
                else:
                    diff = abs(final_pwm - target_pwm)

                self._current_state.id = servo_id
                self._current_state.pwm = final_pwm

                if diff <= 30:
                    result.success = True
                    result.final_pwm = final_pwm
                    result.message = f"ok (attempt={attempt + 1}, diff={diff})"
                    if goal_handle.is_active:
                        goal_handle.succeed()
                    return result

                if attempt == 0:
                    self.get_logger().warn(
                        f"闭环校验未到位: target={target_pwm} actual={final_pwm} "
                        f"diff={diff}，重发一次...")
                    if final_pwm:
                        start_pwm = final_pwm

            result.success = False
            result.final_pwm = final_pwm
            result.message = f"未到位: diff={diff}（已重发）"
            if goal_handle.is_active:
                goal_handle.abort()
            return result
        finally:
            with self._active_lock:
                if self._active.get(servo_id) is goal_handle:
                    del self._active[servo_id]
            self.last_cmd_time = time.time()

    # ================= /servo/command =================
    def command_callback(self, msg: ServoCommand):
        if self.is_locked:
            self.get_logger().warn("看门狗已锁死，忽略 /servo/command")
            return
        self.send_pulr(msg.id)
        self.send_move(msg.id, msg.pwm, msg.duration_ms)

    # ================= /servo/state =================
    def publish_state(self):
        self._state_pub.publish(self._current_state)

    # ================= 看门狗 + 串口重连 =================
    def watchdog_loop(self):
        if self.ser is None:
            now = time.time()
            if now - self._last_reconnect < 2.0:
                return
            self._last_reconnect = now
            self.get_logger().warn("串口断开，尝试自动重连...")
            self._open_serial()
            return

        if not self.get_parameter("watchdog_enabled").value:
            return
        if self.is_locked:
            return

        timeout_s = float(self.get_parameter("watchdog_timeout_s").value)
        if time.time() - self.last_cmd_time > timeout_s:
            self.get_logger().error(
                f"看门狗触发：超过 {timeout_s:.1f} 秒无指令，PULK 卸力并锁死")
            for sid in self.get_parameter("servo_ids").value:
                self._raw_write(f"#{sid:03d}PULK!")
            self.is_locked = True

    def recover_callback(self, request, response):
        self.is_locked = False
        self.last_cmd_time = time.time()
        response.success = True
        response.message = "看门狗已复位"
        self.get_logger().info("收到 /servo/recover，看门狗已复位")
        return response

    # ================= 启动复位 =================
    def startup_reset(self):
        servo_ids = self.get_parameter("servo_ids").value
        duration_ms = self.get_parameter("default_duration_ms").value
        for sid in servo_ids:
            self.send_pulr(sid)
            self.send_move(sid, 1500, duration_ms)
        time.sleep(1.0)
        self.last_cmd_time = time.time()


def main():
    rclpy.init()
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
