"""舵机驱动节点：把 /move_servo Action、/servo/command 话题翻译成总线舵机 ASCII 指令。

设计要点（对着考核文档 2.1~2.6 逐条落）：
  1. 全部参数外置，代码里不出现端口/范围/时间字面量；
  2. goal_callback 做 id / pwm 校验，越界直接 REJECT；
  3. Feedback ≥5 Hz，current_pwm 来自 PRAD 实测，不是自己算的；
  4. 取消：循环里每轮先看 is_cancel_requested，收到就回中位 + 卸力 + canceled()；
  5. 闭环：结束时 PRAD 与目标差 >±30 判没到位，message 写清差多少；
  6. 参数动态生效：每次都实时 get_parameter，不缓存副本。

executor 必须是 MultiThreadedExecutor：execute_cb 里有 sleep 和串口等待，
单线程会让 cancel_callback 排不上队，表现就是「Ctrl+C 了但舵机还在转」。

串口必须加锁：一个 step 里 2 个 goal 是并行下发的，两个线程共用一条半双工总线。
不加锁时 A 的 reset_input_buffer 会把 B 等着的 PRAD 回读冲掉，读空 → 判掉线 →
关端口 → 另一个线程 write 得到 Bad file descriptor → 再重连，日志里就是
「串口掉线 / 串口恢复可用」刷屏，feedback 里 current_pwm 一会儿 0 一会儿真值。
"""
import threading
import time

import rclpy
import serial
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from std_srvs.srv import Trigger

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoState, ServoCommand

from .servo_protocol import (
    cmd_move,
    cmd_prad,
    cmd_torque_off,
    cmd_torque_on,
    parse_prad,
    smooth_duration_ms,
)

POSITION_TOLERANCE = 30   # PRAD 读回与目标差超过这个就算没到位
RETRY_EXTRA_MS = 1500     # 重发给足时间：舵机按命令时长匀速走（实测 T=4000 时约 31 PWM/s），
# 给 500ms 只挪十几 PWM，重发也追不上，日志会永远停在「未到位」（踩过一次）

MID_PWM = 1500            # 中立位是协议常量（1.5 ms 脉宽），不算配置项
FEEDBACK_PERIOD = 0.2     # 5 Hz，考核要求 feedback 不低于 5 Hz
PRAD_TIMEOUT = 0.10       # 单次 PRAD 读取上限；太长会把 feedback 节拍拖到 2 Hz
CANCEL_RETURN_MS = 300    # 取消回中位用的时间，别用 800，要「立刻」回去
CANCEL_SETTLE_S = 0.35    # 回中位后等它停稳再卸力，避免半路卸力留个歪姿态
RESET_DURATION_MS = 4000  # 上电回中立位：必须给足。带载实测 ~30-50 PWM/s，
# 给 800ms 只能走 30 PWM 左右，判「未到位」再重发一次，
# 现象就是「上电先顿一下、过一秒又补一下」——上电乱动的真正原因


class ServoDriverNode(Node):
    def __init__(self):
        super().__init__("servo_driver")
        self.cb_group = ReentrantCallbackGroup()

        # 声明全部参数，禁止硬编码
        self.declare_parameter(
            "port", "/dev/cq_servo",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            "baudrate", 115200,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "pwm_min", 900,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "pwm_max", 2100,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "default_duration_ms", 800,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "servo_ids", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))
        self.declare_parameter(
            "startup_reset", True,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter(
            "speed_pwm_per_s", 600,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "watchdog_timeout_s", 1.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            "prad_retry", 1,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))

        # 参数动态校验回调，用于 ros2 param set 实时校验
        self.add_on_set_parameters_callback(self.param_validate_cb)

        self.ser = None
        self.link_ok = False
        self.ever_connected = False
        # RLock：read_prad 会调用 send_raw，同线程需要可重入
        self.io_lock = threading.RLock()
        self.servo_lock = False
        self.active_goals = {}
        self._preempt_flags = {}
        self._hb_stamp = time.monotonic()
        self._exec_stamp = time.monotonic()
        self.connect_serial()

        # 心跳发布（/servo/state）
        self.servo_state_pub = self.create_publisher(ServoState, "/servo/state", 10)
        self.timer_state = self.create_timer(0.2, self.state_timer_cb)

        # 看门狗巡检
        self.timer_watchdog = self.create_timer(0.2, self.watchdog_cb)

        # 串口在线巡检：掉线后自动重连
        self.timer_link = self.create_timer(2.0, self.link_check_cb)

        # 订阅 /servo/command（bag 回放用）
        self.cmd_sub = self.create_subscription(
            ServoCommand,
            "/servo/command",
            self.on_servo_command,
            10,
            callback_group=self.cb_group,
        )

        # 全部卸力（任务 4 下电用）+ 看门狗人工复位
        self.srv_torque_off = self.create_service(
            Trigger, "~/torque_off", self.torque_off_cb)
        self.srv_watchdog_reset = self.create_service(
            Trigger, "~/watchdog_reset", self.watchdog_reset_cb)
        # 手动触发一次看门狗，用来演示「卸力 + 锁死 + 人工解锁」全链路
        self.srv_watchdog_trip = self.create_service(
            Trigger, "~/watchdog_trip", self.watchdog_trip_cb)
        # 一次读回全部舵机实测位置（message 里是 "id:pwm id:pwm …"），
        # 给 pose_tool 找站姿用；不改接口包，借用 Trigger 的 message 字段
        self.srv_read_all = self.create_service(
            Trigger, "~/read_all", self.read_all_cb)

        self.current_goal_handle = None
        self._action_server = ActionServer(
            self,
            MoveServo,
            "move_servo",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group,
        )

        # 开机总是先上扭力：PULR 只恢复输出，不产生位移，
        # 这一步和「回中位」要分开，否则关掉 startup_reset 后会连扭力都没有
        # 整段用 try 包住：上电自检失败也不能让节点直接死掉，否则整机失控还看不到原因
        try:
            for sid in self.get_parameter("servo_ids").value:
                self.send_raw(cmd_torque_on(sid))

            # 回中位是可选项：整机装好后关掉它，直接由 home.yaml 走到站姿，
            # 免得「先被拽回 1500、再摆到站姿」多抖一次（上电乱动的根源）
            if self.get_parameter("startup_reset").value:
                self.get_logger().info(
                    f"Startup reset: all servos go to 1500 in {RESET_DURATION_MS} ms")
                time.sleep(0.1)
                for sid in self.get_parameter("servo_ids").value:
                    self.send_raw(cmd_move(sid, MID_PWM, RESET_DURATION_MS))
            else:
                self.get_logger().info(
                    "跳过回中位（startup_reset=false），交给 home.yaml 直接摆站姿")
        except Exception:
            import traceback
            traceback.print_exc()
            self.get_logger().error("上电自检失败，节点继续运行（串口巡检会重连）")
        self.get_logger().info("servo_driver 就绪，等待 /move_servo goal")

    # ---------------- 串口 ----------------
    def connect_serial(self):
        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        # 巡检线程也会调这里，必须和读写互斥，否则会把别人正用的句柄换掉
        with self.io_lock:
            try:
                # timeout 必须小：read(1) 超时太长会把 PRAD 轮询和 feedback 节拍一起拖垮
                self.ser = serial.Serial(port, baud, timeout=0.02)
                # 第一次打开和「掉线后恢复」要分开报，否则首启也会打成「恢复可用」
                if self.ever_connected:
                    self.get_logger().warn(f"串口恢复可用：{port} @{baud}")
                else:
                    self.get_logger().info(f"Serial open {port} @{baud}")
                self.ever_connected = True
                self.link_ok = True
                return True
            except Exception as e:
                self.ser = None
                if self.link_ok:
                    self.get_logger().error(f"串口掉线：{e}")
                else:
                    self.get_logger().warn(f"串口打开失败，2 秒后重试：{e}")
                self.link_ok = False
                return False

    def drop_serial(self, reason=""):
        """掉线时调用：关句柄、标不可用，节点本身不退出。"""
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        if self.link_ok:
            self.get_logger().warn(f"串口掉线，节点继续运行，等待自动重连{reason}")
        self.link_ok = False

    def link_check_cb(self):
        """巡检：句柄丢了就重连，恢复后打一条日志。"""
        if self.servo_lock:
            return
        if self.ser is not None and getattr(self.ser, "is_open", False):
            return
        self.connect_serial()

    def send_raw(self, cmd: str):
        # 一条半双工总线只能有一个线程在用：并行 step 的两个 goal 会同时抢它
        with self.io_lock:
            if self.servo_lock:
                self.get_logger().warn(f"安全锁生效，拒绝发送：{cmd}")
                return False
            if self.ser is None or not getattr(self.ser, "is_open", False):
                self.connect_serial()
                if self.ser is None or not getattr(self.ser, "is_open", False):
                    self.get_logger().warn(f"串口不可用，指令丢弃：{cmd}")
                    return False
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd.encode())
                self.ser.flush()   # 必须 flush，否则指令可能压在缓冲里（话题路径曾因此不动）
            except Exception as e:
                # 注意：拔掉 USB 时 pyserial 会漏出 termios.error，它不是 OSError 也不是
                # SerialException，用窄 except 会直接冒到 execute_cb 把 goal 打断（曾经踩过）
                self.drop_serial(f"（写入异常：{e}）")
                return False
        self.get_logger().info(f"TX: {cmd}")
        return True

    def read_prad(self, sid: int, timeout: float = PRAD_TIMEOUT):
        """读回实际位置。轮询到 '!' 立即返回，超时返回 None。

        不用固定 sleep：固定 sleep 会让周期变成「0.15 + 0.2」= 2.8 Hz，达不到 5 Hz。
        整段「发 PRAD + 收回显」必须在一个锁里完成，否则并行 step 的两个 goal
        会互相把对方的回显冲掉（曾经表现为串口反复掉线）。
        """
        with self.io_lock:
            if self.servo_lock:
                return None
            if not self.send_raw(cmd_prad(sid)):
                return None
            buf = b""
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    d = self.ser.read(1)
                except Exception as e:
                    self.drop_serial(f"（读取异常：{e}）")
                    return None
                if not d:
                    continue
                buf += d
                if b"!" in buf:
                    break
            return parse_prad(buf)

    def read_prad_stable(self, sid: int, tries: int = 3, gap: float = 0.05):
        """判定成败时用：总线忙时单次 PRAD 可能读空，多试几次再下结论。"""
        for _ in range(max(int(tries), 1)):
            p = self.read_prad(sid)
            if p is not None:
                return p
            time.sleep(gap)
        return None

    # ---------------- 参数 ----------------
    def param_validate_cb(self, params):
        pmin = self.get_parameter("pwm_min").value
        pmax = self.get_parameter("pwm_max").value
        baud = self.get_parameter("baudrate").value
        dur = self.get_parameter("default_duration_ms").value
        for p in params:
            if p.name == "pwm_max":
                pmax = p.value
            if p.name == "pwm_min":
                pmin = p.value
            if p.name == "baudrate":
                baud = p.value
            if p.name == "default_duration_ms":
                dur = p.value
        if baud not in [9600, 115200]:
            return SetParametersResult(
                successful=False, reason="baud only support 9600/115200")
        if pmin >= pmax:
            return SetParametersResult(
                successful=False, reason="pwm_min cannot >= pwm_max")
        if pmax < 500 or pmax > 2500 or pmin < 500 or pmin > 2500:
            return SetParametersResult(
                successful=False, reason="pwm range limit 500~2500")
        if dur <= 0:
            return SetParametersResult(
                successful=False, reason="default_duration_ms must be > 0")
        return SetParametersResult(successful=True)

    # ---------------- 心跳 / 看门狗 ----------------
    def state_timer_cb(self):
        msg = ServoState()
        msg.alive = not self.servo_lock
        self.servo_state_pub.publish(msg)
        if not self.servo_lock:
            self._hb_stamp = time.monotonic()

    def watchdog_cb(self):
        if self.servo_lock:
            return
        timeout = float(self.get_parameter("watchdog_timeout_s").value)
        now = time.monotonic()
        if now - self._hb_stamp > timeout:
            self.trip_watchdog("心跳中断")
            return
        gh = self.current_goal_handle
        if gh is not None and getattr(gh, "is_active", False):
            if now - self._exec_stamp > timeout:
                self.trip_watchdog("动作执行停摆")

    def trip_watchdog(self, why: str):
        self.get_logger().error(
            f"看门狗触发：{why} 超过 {self.get_parameter('watchdog_timeout_s').value}s，"
            "全部舵机卸力并锁定，需人工确认后恢复")
        for sid in self.get_parameter("servo_ids").value:
            self.send_raw(cmd_torque_off(sid))
        self.servo_lock = True
        self.get_logger().error("安全锁已置位：拒绝一切 goal 与话题指令")

    def torque_off_cb(self, request, response):
        """下电前手动卸力：ros2 service call /servo_driver/torque_off std_srvs/srv/Trigger"""
        self.get_logger().info("收到卸力请求，全部舵机 PULK")
        for sid in self.get_parameter("servo_ids").value:
            self.send_raw(cmd_torque_off(sid))
        response.success = True
        response.message = "all servos torque off"
        return response

    def read_all_cb(self, request, response):
        """一次读回全部舵机实测位置，message = "id:pwm id:pwm …"（读不到记 -1）。"""
        parts = []
        for sid in self.get_parameter("servo_ids").value:
            p = self.read_prad_stable(sid, tries=2)
            parts.append(f"{sid}:{p if p is not None else -1}")
        response.success = True
        response.message = " ".join(parts)
        self.get_logger().info(f"全机读数：{response.message}")
        return response

    def watchdog_trip_cb(self, request, response):
        """手动触发看门狗，演示「卸力 + 锁死 + 人工解锁」全链路。"""
        self.trip_watchdog("手动触发测试")
        response.success = True
        response.message = "watchdog tripped on demand"
        return response

    def watchdog_reset_cb(self, request, response):
        """外部确认后解除安全锁（必须人来点，不许节点自己恢复）。"""
        if not self.servo_lock:
            response.success = False
            response.message = "not locked"
            return response
        self.servo_lock = False
        self._hb_stamp = time.monotonic()
        self._exec_stamp = time.monotonic()
        self.get_logger().warn("外部确认收到：看门狗已复位，可以重新下发指令")
        response.success = True
        response.message = "watchdog reset, servo unlocked"
        return response

    # ---------------- 话题调试总线（bag 回放） ----------------
    def on_servo_command(self, cmd: ServoCommand):
        self.get_logger().info("===== Got /servo/command topic msg =====")
        if self.servo_lock:
            self.get_logger().warn("Servo locked, reject command")
            return
        sid = cmd.id
        target_pwm = cmd.pwm
        dur = cmd.duration_ms
        if dur <= 0:
            dur = self.get_parameter("default_duration_ms").value
        valid_ids = self.get_parameter("servo_ids").value
        pmin = self.get_parameter("pwm_min").value
        pmax = self.get_parameter("pwm_max").value
        self.get_logger().info(f"sid={sid}, pwm={target_pwm}, dur={dur}")
        if sid not in valid_ids or not (pmin <= target_pwm <= pmax):
            self.get_logger().warn(
                f"Command rejected: id={sid}, pwm={target_pwm} out of range")
            return
        self.send_raw(cmd_torque_on(sid))
        time.sleep(0.05)
        self.send_raw(cmd_move(sid, target_pwm, dur))

    # ---------------- Action ----------------
    def goal_cb(self, goal_req):
        if self.servo_lock:
            self.get_logger().error("Reject goal: watchdog locked")
            return GoalResponse.REJECT
        sid = goal_req.id
        pwm = goal_req.pwm
        pmin = self.get_parameter("pwm_min").value
        pmax = self.get_parameter("pwm_max").value
        id_list = self.get_parameter("servo_ids").value
        if sid not in id_list:
            self.get_logger().error(f"Goal reject: id={sid} not in servo_ids")
            return GoalResponse.REJECT
        if not (pmin <= pwm <= pmax):
            self.get_logger().error(
                f"Goal reject: pwm={pwm} out of range {pmin}~{pmax}")
            return GoalResponse.REJECT
        # duration_ms 为 0 不算错，交给 default_duration_ms 兜底（考核 2.2）

        # 抢占：同一舵机上的新 goal 打断旧 goal，旧 goal 不再 PULK，直接交棒
        old = self.active_goals.get(sid)
        if old is not None and getattr(old, "is_active", False):
            self.get_logger().warn(
                f"id={sid} 新 goal 抢占旧 goal，从当前位置平滑接上")
            self._preempt_flags[sid] = True
            old.cancel()
        return GoalResponse.ACCEPT

    def _release_goal(self, sid, goal_handle):
        """只注销自己这一条，避免抢占时把新 goal 的记录误删。"""
        if self.active_goals.get(sid) is goal_handle:
            self.active_goals.pop(sid, None)
        if self.current_goal_handle is goal_handle:
            self.current_goal_handle = None

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def _run_motion(self, goal_handle, sid, dur_ms):
        """按 5 Hz 发反馈直到走满 dur_ms；返回 True 表示被取消。

        每轮第一件事就是查 cancel，保证取消能在「下一个 0.2 s」内被看见，
        而不是等整段动作跑完。
        """
        total_s = max(int(dur_ms), 1) / 1000.0
        t_start = time.monotonic()
        t_end = t_start + total_s
        fb = MoveServo.Feedback()
        last_known = 0   # 读失败时沿用上一次真值，别把 0 当成位置发出去

        while True:
            if goal_handle.is_cancel_requested:
                return True

            tick = time.monotonic()
            cur = self.read_prad(sid)
            if cur is not None:
                last_known = cur
            fb.current_pwm = last_known
            fb.progress = float(min((tick - t_start) / total_s, 1.0))
            goal_handle.publish_feedback(fb)
            self._exec_stamp = time.monotonic()

            if tick >= t_end:
                return False

            # 补节拍：读 PRAD 花了多久就从 0.2 s 里扣掉，保证 5 Hz 而不是「读多久算多久」
            wait = FEEDBACK_PERIOD - (time.monotonic() - tick)
            if wait > 0:
                time.sleep(wait)

    def execute_cb(self, goal_handle: ServerGoalHandle):
        sid = goal_handle.request.id
        target_pwm = goal_handle.request.pwm
        requested_ms = goal_handle.request.duration_ms
        if requested_ms <= 0:
            requested_ms = self.get_parameter("default_duration_ms").value

        self.active_goals[sid] = goal_handle
        self.current_goal_handle = goal_handle
        self._exec_stamp = time.monotonic()

        result = MoveServo.Result()

        # 抢占/普通执行都从「当前位置」起步，避免跳变
        current_p = self.read_prad_stable(sid, tries=2)
        if current_p is None:
            # 读不到起点就别按 0 去算时长（会把 800ms 撑成 2500ms），直接用请求值
            dur = requested_ms
        else:
            dur = smooth_duration_ms(
                current_p, target_pwm, requested_ms,
                self.get_parameter("speed_pwm_per_s").value)

        if current_p is not None and abs(target_pwm - current_p) <= POSITION_TOLERANCE:
            self.get_logger().info(
                f"id={sid} 已在目标位 current={current_p}，无需动作")
            result.success = True
            result.final_pwm = current_p
            result.message = "already in place"
            goal_handle.succeed()
            self._release_goal(sid, goal_handle)
            return result

        self.send_raw(cmd_torque_on(sid))
        self.send_raw(cmd_move(sid, target_pwm, dur))
        self.get_logger().info(
            f"id={sid} 目标 {target_pwm}，起点 {current_p}，计划 {dur}ms")

        canceled = self._run_motion(goal_handle, sid, dur)
        if canceled:
            preempted = self._preempt_flags.pop(sid, False)
            if preempted:
                # 抢占不是「取消」，扭力保持，新 goal 从当前位置接着走
                self.get_logger().warn(
                    f"id={sid} 被新 goal 抢占，保持扭力，从当前位置交棒")
                result.success = False
                result.message = "preempted"
            else:
                # 考核 2.3：立刻停发 → 回中位 → 卸力 → canceled()
                self.get_logger().warn(
                    f"id={sid} 收到取消：停动作，回中位 {MID_PWM}，再卸力")
                self.send_raw(cmd_move(sid, MID_PWM, CANCEL_RETURN_MS))
                time.sleep(CANCEL_SETTLE_S)
                self.send_raw(cmd_torque_off(sid))
                result.success = False
                result.message = "canceled"
            goal_handle.canceled()
            cur = self.read_prad_stable(sid)
            result.final_pwm = cur if cur is not None else 0
            self.get_logger().info(
                f"id={sid} 取消完成，final_pwm={result.final_pwm}")
            self._release_goal(sid, goal_handle)
            return result

        # 闭环校验：没到位就重发一次
        final_p = self.read_prad_stable(sid)
        retry = int(self.get_parameter("prad_retry").value)
        while retry > 0 and final_p is not None \
                and abs(final_p - target_pwm) > POSITION_TOLERANCE:
            self.get_logger().warn(
                f"id={sid} 未到位 target={target_pwm} actual={final_p}，重发一次")
            retry_dur = max(dur, RETRY_EXTRA_MS)
            self.send_raw(cmd_move(sid, target_pwm, retry_dur))
            self._run_motion(goal_handle, sid, retry_dur)
            final_p = self.read_prad_stable(sid)
            retry -= 1
            if final_p is not None and abs(final_p - target_pwm) <= POSITION_TOLERANCE:
                self.get_logger().info(
                    f"id={sid} 重发后到位 actual={final_p}")

        if final_p is None:
            result.success = False
            result.message = "serial read fail"
            result.final_pwm = 0
        else:
            diff = abs(final_p - target_pwm)
            result.final_pwm = final_p
            if diff > POSITION_TOLERANCE:
                result.success = False
                result.message = (
                    f"Not in place, target:{target_pwm}, actual:{final_p}, diff:{diff}")
                self.get_logger().warn(result.message)
            else:
                result.success = True
                result.message = "success"

        goal_handle.succeed()
        self._release_goal(sid, goal_handle)
        return result


def main(args=None):
    rclpy.init(args=args)
    # 构造阶段任何异常都要把 traceback 打出来：launch 只会报 process has died，
    # 看不到真因，调起来纯靠猜（曾经踩过，日志只有 exit code 1）
    try:
        node = ServoDriverNode()
    except Exception:
        import traceback
        traceback.print_exc()
        rclpy.shutdown()
        raise
    # 必须多线程：execute_cb 里有 sleep / 串口等待，单线程会挡住 cancel_callback，
    # 现象就是「客户端 Ctrl+C 了，舵机还在转，取消请求排队到动作结束才回 3」
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
