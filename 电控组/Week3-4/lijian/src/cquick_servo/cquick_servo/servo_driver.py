import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import serial

from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo_interfaces.action import MoveServo


class ServoDriver(Node):
    def __init__(self):
        super().__init__("servo_driver")

        # ---- 声明参数（名字和 yaml 完全一致） ----
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("pwm_min", 900)
        self.declare_parameter("pwm_max", 2100)
        self.declare_parameter("default_duration_ms", 800)
        self.declare_parameter("servo_ids", [1])
        self.declare_parameter("startup_reset", True)

        self._serial_lock = threading.Lock()
        self.ser = None
        self._open_serial()

        # ReentrantCallbackGroup：允许 Action 的 cancel 在 execute 运行中进来
        self._cb_group = ReentrantCallbackGroup()

        # ---- Action Server ----
        self._action_server = ActionServer(
            self,
            MoveServo,
            "move_servo",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        # ---- /servo/command 订阅（bag 回放用，和 Action 共用 send_move） ----
        self.create_subscription(
            ServoCommand,
            "/servo/command",
            self.command_callback,
            10,
            callback_group=self._cb_group,
        )


        # ---- /servo/state 发布 ----
        self._state_pub = self.create_publisher(ServoCommand, "/servo/state", 10)
        self._current_state = ServoCommand()
        self._current_state.id = 0
        self._current_state.pwm = 1500
        self._current_state.duration_ms = 0
        self.create_timer(0.5, self.publish_state)

        # ---- 参数校验回调 ----
        self.add_on_set_parameters_callback(self.param_callback)

        # ---- 启动复位 ----
        if self.get_parameter("startup_reset").value:
            self.startup_reset()

        self.get_logger().info("servo_driver ready")
        self._cmd_pub = self.create_publisher(ServoCommand, "/servo/command", 10)
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
        """底层发送：Action 和 /servo/command 共用这一个函数"""
        if self.ser is None:
            self.get_logger().error("serial not open")
            return False
        with self._serial_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd.encode("ascii") + b"\r\n")
                self.ser.flush()
                return True
            except Exception as e:
                self.get_logger().error(f"serial write failed: {e}")
                return False

    # ================= 协议 =================
    def send_move(self, servo_id: int, pwm: int, duration_ms: int) -> bool:
        return self._raw_write(f"#{servo_id:03d}P{pwm:04d}T{duration_ms:04d}!")

    def send_pulr(self, servo_id: int):
        return self._raw_write(f"#{servo_id:03d}PULR!")

    def send_pulk(self, servo_id: int):
        return self._raw_write(f"#{servo_id:03d}PULK!")

    def read_prad(self, servo_id: int):
        """发 PRAD 读当前位置，返回 int 或 None"""
        if self.ser is None:
            return None
        with self._serial_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(f"#{servo_id:03d}PRAD!\r\n".encode("ascii"))
                self.ser.flush()
                time.sleep(0.15)
                self.ser.timeout = 0.3
                data = self.ser.read(64)
            except Exception as e:
                self.get_logger().error(f"read_prad failed: {e}")
                return None
        if not data:
            return None
        text = data.decode("ascii", errors="ignore").strip()
        try:
            if text.startswith("#") and "P" in text:
                body = text.split("P", 1)[1].rstrip("!")
                return int(body)
        except Exception:
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
                cur_max = self.get_parameter("pwm_max").value
                if v >= cur_max:
                    return SetParametersResult(
                        successful=False, reason=f"pwm_min({v}) >= pwm_max({cur_max})")
            elif p.name == "pwm_max":
                v = p.value
                if not isinstance(v, int) or not (500 <= v <= 2500):
                    return SetParametersResult(
                        successful=False, reason=f"pwm_max={v} 超出 500~2500")
                cur_min = self.get_parameter("pwm_min").value
                if v <= cur_min:
                    return SetParametersResult(
                        successful=False, reason=f"pwm_max({v}) <= pwm_min({cur_min})")
            elif p.name == "baudrate":
                v = p.value
                if v not in (9600, 19200, 38400, 57600, 115200):
                    return SetParametersResult(
                        successful=False, reason=f"不支持 baudrate={v}")
        return SetParametersResult(successful=True)

    # ================= Action =================
    def goal_callback(self, goal_request):
        # 每次实时读参数，不用 __init__ 里的副本
        pwm_min = self.get_parameter("pwm_min").value
        pwm_max = self.get_parameter("pwm_max").value
        servo_ids = self.get_parameter("servo_ids").value

        if goal_request.id not in servo_ids:
            self.get_logger().warn(
                f"reject: id={goal_request.id} not in servo_ids={servo_ids}")
            return GoalResponse.REJECT
        if not (pwm_min <= goal_request.pwm <= pwm_max):
            self.get_logger().warn(
                f"reject: pwm={goal_request.pwm} not in [{pwm_min},{pwm_max}]")
            return GoalResponse.REJECT
        if goal_request.duration_ms > 10000:
            self.get_logger().warn(
                f"reject: duration_ms={goal_request.duration_ms} > 10000")
            return GoalResponse.REJECT
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

        result = MoveServo.Result()
        result.success = False
        result.final_pwm = 0
        result.message = ""

        # 1) 读起始位置
        start_pwm = self.read_prad(servo_id)
        if start_pwm is None:
            start_pwm = 1500
            self.get_logger().warn("起始 PRAD 读失败，假定 1500")

        # 2) 恢复扭力
        self.send_pulr(servo_id)
        time.sleep(0.05)

        # 3) 发目标，同时发布到 /servo/command 供 bag 录制
        cmd_msg = ServoCommand()
        cmd_msg.id = servo_id
        cmd_msg.pwm = target_pwm
        cmd_msg.duration_ms = duration_ms
        self._cmd_pub.publish(cmd_msg)

        if not self.send_move(servo_id, target_pwm, duration_ms):
            result.message = "串口写失败"
            goal_handle.abort()
            return result

        # 4) 循环发 feedback（10 Hz，满足 ≥5Hz）
        t0 = time.time()
        duration_s = duration_ms / 1000.0
        while True:
            if goal_handle.is_cancel_requested:
                # 立刻停发、回中位、返回 canceled
                self.send_move(servo_id, 1500, 300)
                time.sleep(0.35)
                self.send_pulk(servo_id)
                result.success = False
                result.final_pwm = 1500
                result.message = "canceled"
                goal_handle.canceled()
                return result

            elapsed = time.time() - t0
            progress = min(1.0, elapsed / duration_s) if duration_s > 0 else 1.0
            current_pwm = int(start_pwm + (target_pwm - start_pwm) * progress)

            fb = MoveServo.Feedback()
            fb.current_pwm = current_pwm
            fb.progress = float(progress)
            goal_handle.publish_feedback(fb)

            self._current_state.id = servo_id
            self._current_state.pwm = current_pwm

            if progress >= 1.0:
                break
            time.sleep(0.1)

        # 5) 读最终位置，±30 判定
        final_pwm = self.read_prad(servo_id)
        if final_pwm is None:
            result.success = False
            result.final_pwm = 0
            result.message = "读 PRAD 失败"
            goal_handle.abort()
            return result

        diff = abs(final_pwm - target_pwm)
        result.final_pwm = final_pwm
        self._current_state.id = servo_id
        self._current_state.pwm = final_pwm

        if diff <= 30:
            result.success = True
            result.message = f"ok (diff={diff})"
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f"未到位: target={target_pwm} actual={final_pwm} diff={diff}"
            goal_handle.abort()
        return result

    # ================= /servo/command =================
    def command_callback(self, msg: ServoCommand):
        # bag 回放：和 Action 走同一个 send_move
        self.send_move(msg.id, msg.pwm, msg.duration_ms)

    # ================= /servo/state =================
    def publish_state(self):
        self._state_pub.publish(self._current_state)
import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import serial
from std_srvs.srv import Trigger

from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo_interfaces.action import MoveServo

class ServoDriver(Node):
    def __init__(self):
        super().__init__("servo_driver")
        
        # 参数
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("pwm_min", 900)
        self.declare_parameter("pwm_max", 2100)
        self.declare_parameter("default_duration_ms", 800)
        self.declare_parameter("servo_ids", [1, 2, 3, 4])
        self.declare_parameter("startup_reset", True)

        self._serial_lock = threading.Lock()
        self.ser = None
        self._open_serial()

        self._cb_group = ReentrantCallbackGroup()

        # 加分项：安全看门狗与抢占控制变量
        self.last_cmd_time = time.time()
        self.is_locked = False # 看门狗锁死标志
        self.current_goal_handle = None # 用于动作组抢占

        self._action_server = ActionServer(
            self, MoveServo, "move_servo",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.create_subscription(ServoCommand, "/servo/command", self.command_callback, 10, callback_group=self._cb_group)
        self._cmd_pub = self.create_publisher(ServoCommand, "/servo/command", 10)
        self._state_pub = self.create_publisher(ServoCommand, "/servo/state", 10)
        
        self._current_state = ServoCommand()
        self._current_state.id = 0
        self._current_state.pwm = 1500
        self._current_state.duration_ms = 0
        
        # 定时器：0.5s发心跳，0.1s检查看门狗和串口重连
        self.create_timer(0.5, self.publish_state)
        self.create_timer(0.1, self.watchdog_and_reconnect_loop)

        self.add_on_set_parameters_callback(self.param_callback)
        
        # 服务：解除看门狗锁定
        self.create_service(Trigger, "/servo/recover", self.recover_callback)

        if self.get_parameter("startup_reset").value:
            self.startup_reset()
        self.get_logger().info("servo_driver ready")

    def _open_serial(self):
        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        try:
            self.ser = serial.Serial(port, baud, bytesize=8, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.3)
            self.get_logger().info(f"serial opened: {port} @ {baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"open serial failed: {e}")

    def _raw_write(self, cmd: str) -> bool:
        if self.ser is None:
            return False
        with self._serial_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd.encode("ascii") + b"\r\n")
                self.ser.flush()
                self.last_cmd_time = time.time() # 收到指令，刷新看门狗
                return True
            except Exception as e:
                self.get_logger().error(f"serial write failed, closing port for reconnect: {e}")
                self.ser = None # 加分项：串口掉线触发重连
                return False

    def send_move(self, servo_id: int, pwm: int, duration_ms: int) -> bool:
        return self._raw_write(f"#{servo_id:03d}P{pwm:04d}T{duration_ms:04d}!")

    def send_pulr(self, servo_id: int):
        return self._raw_write(f"#{servo_id:03d}PULR!")

    def send_pulk(self, servo_id: int):
        return self._raw_write(f"#{servo_id:03d}PULK!")

    def read_prad(self, servo_id: int):
        if self.ser is None: return None
        with self._serial_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(f"#{servo_id:03d}PRAD!\r\n".encode("ascii"))
                self.ser.flush()
                time.sleep(0.15)
                data = self.ser.read(64)
                self.last_cmd_time = time.time() # 刷新看门狗
            except Exception as e:
                self.get_logger().error(f"read_prad failed, closing port: {e}")
                self.ser = None # 加分项：串口掉线触发重连
                return None
        if not data: return None
        text = data.decode("ascii", errors="ignore").strip()
        try:
            if text.startswith("#") and "P" in text:
                return int(text.split("P", 1)[1].rstrip("!"))
        except: pass
        return None

    def param_callback(self, params):
        for p in params:
            if p.name == "pwm_min":
                v = p.value
                if not (500 <= v <= 2500) or v >= self.get_parameter("pwm_max").value:
                    return SetParametersResult(successful=False, reason="Invalid pwm_min")
            elif p.name == "pwm_max":
                v = p.value
                if not (500 <= v <= 2500) or v <= self.get_parameter("pwm_min").value:
                    return SetParametersResult(successful=False, reason="Invalid pwm_max")
        return SetParametersResult(successful=True)

    def goal_callback(self, goal_request):
        if self.is_locked:
            self.get_logger().warn("看门狗已锁死，拒绝新动作！请调用 /servo/recover 恢复")
            return GoalResponse.REJECT

        pwm_min = self.get_parameter("pwm_min").value
        pwm_max = self.get_parameter("pwm_max").value
        servo_ids = self.get_parameter("servo_ids").value

        if goal_request.id not in servo_ids or not (pwm_min <= goal_request.pwm <= pwm_max):
            return GoalResponse.REJECT

        # 加分项：动作组抢占（新 goal 到来时直接取消旧的）
        if self.current_goal_handle and self.current_goal_handle.is_active:
            self.get_logger().warn(f"抢占触发！打断当前动作，执行新指令 id={goal_request.id} pwm={goal_request.pwm}")
            self.current_goal_handle.abort()
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.current_goal_handle = goal_handle # 记录当前执行的动作
        goal = goal_handle.request
        servo_id, target_pwm, duration_ms = goal.id, goal.pwm, goal.duration_ms
        if duration_ms == 0: duration_ms = self.get_parameter("default_duration_ms").value

        result = MoveServo.Result()
        start_pwm = self.read_prad(servo_id) or 1500
        
        self.send_pulr(servo_id)
        time.sleep(0.05)

        # 加分项：PRAD 闭环校验（最多尝试 2 次：首发 + 重发）
        for attempt in range(2):
            cmd_msg = ServoCommand(id=servo_id, pwm=target_pwm, duration_ms=duration_ms)
            self._cmd_pub.publish(cmd_msg)
            if not self.send_move(servo_id, target_pwm, duration_ms):
                result.message = "serial error"
                goal_handle.abort()
                return result

            t0 = time.time()
            while time.time() - t0 < duration_ms / 1000.0:
                if goal_handle.is_cancel_requested:
                    self.send_move(servo_id, 1500, 300)
                    time.sleep(0.35)
                    self.send_pulk(servo_id)
                    result.message = "canceled"
                    goal_handle.canceled()
                    return result
                progress = (time.time() - t0) / (duration_ms / 1000.0)
                goal_handle.publish_feedback(MoveServo.Feedback(current_pwm=int(start_pwm + (target_pwm-start_pwm)*progress), progress=progress))
                time.sleep(0.1)
            
            final_pwm = self.read_prad(servo_id) or 0
            diff = abs(final_pwm - target_pwm)
            if diff <= 30:
                result.success = True
                result.final_pwm = final_pwm
                result.message = f"ok (attempt={attempt+1}, diff={diff})"
                goal_handle.succeed()
                return result
            else:
                self.get_logger().warn(f"闭环校验未到位: target={target_pwm} actual={final_pwm} diff={diff}，准备重发...")
        
        # 两次都没到位
        result.success = False
        result.final_pwm = final_pwm
        result.message = f"failed after retry, diff={diff}"
        goal_handle.abort()
        return result

    def command_callback(self, msg: ServoCommand):
        if not self.is_locked:
            self.send_move(msg.id, msg.pwm, msg.duration_ms)

    def publish_state(self):
        self._state_pub.publish(self._current_state)

    def watchdog_and_reconnect_loop(self):
        # 1. 串口掉线重连
        if self.ser is None:
            self.get_logger().info("检测到串口断开，尝试自动重连...")
            self._open_serial()
            return # 本轮不检查看门狗

        # 2. 看门狗（超 1 秒无指令卸力并锁死）
        if not self.is_locked and (time.time() - self.last_cmd_time > 1.0):
            self.get_logger().error("看门狗触发！超过 1 秒无指令，自动 PULK 卸力！")
            servo_ids = self.get_parameter("servo_ids").value
            for sid in servo_ids:
                self._raw_write(f"#{sid:03d}PULK!")
            self.is_locked = True

    def recover_callback(self, request, response):
        """解除看门狗锁定"""
        self.is_locked = False
        self.last_cmd_time = time.time() # 重置时间
        response.success = True
        response.message = "看门狗已复位，恢复可用"
        self.get_logger().info("收到 /servo/recover 请求，看门狗已复位")
        return response

    def startup_reset(self):
        servo_ids = self.get_parameter("servo_ids").value
        for sid in servo_ids:
            self.send_pulr(sid)
            self.send_move(sid, 1500, 800)
        time.sleep(1.0)

def main():
    rclpy.init()
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
