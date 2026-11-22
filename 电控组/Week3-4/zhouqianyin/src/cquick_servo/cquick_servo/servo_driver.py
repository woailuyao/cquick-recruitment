#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CQuick 舵机驱动节点（Action Server + 内部调试总线）

数据流：goal → 参数校验 → 拼 ASCII 指令 → 串口 → 控制板 → 舵机 → PRAD 回读 → result

设计要点（对应任务书 2.2 ~ 2.7）：
  * 全部可配置项走 ROS 参数，代码里只剩协议常量；
  * 对外控制口只有 Action，不用话题/服务；
  * /servo/command 话题复用同一个底层发送函数，保证 bag 回放与实机一致；
  * 参数设置回调做范围与互相约束校验，越界直接拒绝；
  * 判断限位时每次重新 get_parameter，改参数当帧生效。
"""

import re
import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

# ---------------------------------------------------------------------------
# 协议常量：这些是指令集本身，不属于配置，允许写死
# ---------------------------------------------------------------------------
FRAME_START = '#'
FRAME_END = '!'
ID_WIDTH = 3
CMD_READ_ID = 'PID'
CMD_READ_POS = 'PRAD'
CMD_TORQUE_ON = 'PULR'
CMD_TORQUE_OFF = 'PULK'
POSITION_TEMPLATE = 'P{pwm}T{duration}'
RESPONSE_TIMEOUT = 0.15        # 单条指令等待回显的最长时间
POSITION_PATTERN = re.compile(r'#\d{3}P(\d{4})')   # PRAD 回显形如 #001P1505!

# 电气极限与波特率合法区间（用于参数校验，不是运行配置）
HARDWARE_PWM_MIN, HARDWARE_PWM_MAX = 500, 2500
MIN_BAUDRATE, MAX_BAUDRATE = 1200, 4000000
MAX_DURATION_MS = 30000
MIN_ID, MAX_ID = 0, 254   # 众灵 24 路板为 0-based 通道号：000/001 是独立舵机（已实测非广播）


def build_command(servo_id, body):
    """拼一条 ASCII 指令：# + 3 位补零 ID + 指令体 + !"""
    return f'{FRAME_START}{servo_id:0{ID_WIDTH}d}{body}{FRAME_END}'


class SerialLink:
    """串口收发封装。所有访问加锁：Action 线程与话题回调会同时用到它。"""

    def __init__(self, logger):
        self._logger = logger
        self._lock = threading.Lock()
        self._ser = None

    def open(self, port, baudrate):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001 - 关闭失败不影响重开
                    pass
                self._ser = None
            self._ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=RESPONSE_TIMEOUT,
                write_timeout=RESPONSE_TIMEOUT,
            )
            self._ser.reset_input_buffer()

    def close(self):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                finally:
                    self._ser = None

    @property
    def is_open(self):
        return self._ser is not None

    def transact(self, command, expect_response=True):
        """发一条指令，按需读回显。返回原始字节（无回显返回 b''）。"""
        with self._lock:
            if self._ser is None:
                raise RuntimeError('串口未打开')
            self._ser.reset_input_buffer()
            self._ser.write(command.encode('ascii'))
            self._ser.flush()
            if not expect_response:
                return b''
            buf = b''
            deadline = time.monotonic() + RESPONSE_TIMEOUT
            while time.monotonic() < deadline and FRAME_END.encode() not in buf:
                chunk = self._ser.read(self._ser.in_waiting or 1)
                if chunk:
                    buf += chunk
            return buf


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')
        self._declare_parameters()

        self._loaded_ids = set()          # 已恢复过扭力的舵机
        self._last_pwm = {}               # 每个舵机最后一次目标位置（反馈用）
        self._moving_until = {}           # 每个舵机预计动作结束时刻，心跳期间跳过
        self._watchdog_tripped = False    # 看门狗闩锁：触发后拒绝一切动作，直到外部确认
        self._last_hb = time.monotonic()  # 最近一次 /servo/state 心跳时刻
        self._cmd_lock = threading.Lock()  # 串行化动作，避免两条指令交叉
        self._active_port = None
        self._active_baud = None
        self._open_error_logged = False

        self._link = SerialLink(self.get_logger())
        self._try_open_serial()

        cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )
        self._cmd_sub = self.create_subscription(
            ServoCommand, '/servo/command', self.command_callback, 10,
            callback_group=cb_group)
        self._state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)

        self.add_on_set_parameters_callback(self.on_set_parameters)
        # port / baudrate 改了以后异步重开串口（参数回调里不能阻塞）
        self.create_timer(0.5, self._apply_serial_params)
        # 位置帧心跳：板子在帧执行完后会停发 PWM，舵机失信号就开始滴滴响并泄力
        hb = float(self.param('heartbeat_period'))
        if hb > 0:
            self.create_timer(max(0.1, hb), self._heartbeat)
        # 安全看门狗：独立线程监视 /servo/state 心跳，超时自动 PULK 并闩锁
        self._last_hb = time.monotonic()   # 线程启动前刷新（VMware 打开 CH340 可能耗时 >1s，别算成失联）
        self._wd_boot = time.monotonic()   # 启动宽限期起点：首跳心跳发出前不判定
        self._wd_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._wd_thread.start()

        if self.param('startup_reset'):
            self.startup_reset()

        self.get_logger().info(
            f'servo_driver 就绪: port={self.param("port")} baud={self.param("baudrate")} '
            f'pwm=[{self.param("pwm_min")},{self.param("pwm_max")}] '
            f'ids={list(self.param("servo_ids"))}')

    # ------------------------------------------------------------------
    # 参数
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        self.declare_parameter('port', '/dev/ttyUSB0',
                               ParameterDescriptor(description='串口设备名'))
        self.declare_parameter('baudrate', 115200,
                               ParameterDescriptor(description='波特率，需与舵机控制板一致'))
        self.declare_parameter('pwm_min', 900,
                               ParameterDescriptor(description='安全下限（结构实测）'))
        self.declare_parameter('pwm_max', 2100,
                               ParameterDescriptor(description='安全上限（结构实测）'))
        self.declare_parameter('default_duration_ms', 800,
                               ParameterDescriptor(description='客户端未给 duration 时的默认值'))
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6],
                               ParameterDescriptor(description='本机在线舵机 ID 列表'))
        self.declare_parameter('startup_reset', True,
                               ParameterDescriptor(description='启动时是否全部回中位'))
        # 附加参数（非必需，便于调参）
        self.declare_parameter('position_tolerance', 30,
                               ParameterDescriptor(description='PRAD 到位判定容差'))
        self.declare_parameter('feedback_period', 0.2,
                               ParameterDescriptor(description='反馈周期 s，≤0.2 才满足 5Hz'))
        self.declare_parameter('heartbeat_period', 0.5,
                               ParameterDescriptor(
                                   description='位置帧心跳周期 s；0 表示关闭。'
                                               '板子帧执行完会停发 PWM，需周期重发维持信号'))
        self.declare_parameter('watchdog_timeout', 1.0,
                               ParameterDescriptor(
                                   description='安全看门狗：/servo/state 心跳超过该秒数未更新，'
                                               '自动 PULK 全部舵机并闩锁'))
        self.declare_parameter('watchdog_reset', False,
                               ParameterDescriptor(
                                   description='外部确认恢复：看门狗闩锁后置 true 才允许恢复动作'))

    def param(self, name):
        """每次都重新读，保证 ros2 param set 当帧生效。"""
        return self.get_parameter(name).value

    @property
    def center_pwm(self):
        return (int(self.param('pwm_min')) + int(self.param('pwm_max'))) // 2

    # ------------------------------------------------------------------
    # 串口
    # ------------------------------------------------------------------
    def _try_open_serial(self):
        port, baud = self.param('port'), int(self.param('baudrate'))
        try:
            self._link.open(port, baud)
            self._active_port, self._active_baud = port, baud
            self._open_error_logged = False
            self._loaded_ids.clear()
            self.get_logger().info(f'串口已打开: {port} @ {baud}')
            return True
        except (serial.SerialException, OSError, ValueError) as exc:
            if not self._open_error_logged:
                self.get_logger().error(f'串口打开失败 {port}: {exc}（检查设备名/权限/是否被占用）')
                self._open_error_logged = True
            return False

    def _apply_serial_params(self):
        port, baud = self.param('port'), int(self.param('baudrate'))
        if (port, baud) == (self._active_port, self._active_baud) and self._link.is_open:
            return
        self.get_logger().warn(f'检测到串口参数变化 -> {port} @ {baud}，重开串口')
        self._try_open_serial()

    def close(self):
        self._link.close()

    # ------------------------------------------------------------------
    # 参数校验回调
    # ------------------------------------------------------------------
    def on_set_parameters(self, params):
        current_min = int(self.param('pwm_min'))
        current_max = int(self.param('pwm_max'))
        for p in params:
            # 看门狗外部确认恢复：必须在 bool 快速跳过之前处理
            if p.name == 'watchdog_reset':
                if p.value is True and self._watchdog_tripped:
                    self._watchdog_tripped = False
                    self._last_hb = time.monotonic()
                    self.get_logger().warn('看门狗已由外部确认复位，恢复允许动作')
                continue
            if isinstance(p.value, bool):
                continue
            if p.name == 'pwm_max':
                if not isinstance(p.value, int):
                    return SetParametersResult(successful=False, reason='pwm_max 必须是整数')
                if not HARDWARE_PWM_MIN <= p.value <= HARDWARE_PWM_MAX:
                    return SetParametersResult(
                        successful=False,
                        reason=f'pwm_max={p.value} 超出电气极限 {HARDWARE_PWM_MIN}~{HARDWARE_PWM_MAX}')
                if p.value <= current_min:
                    return SetParametersResult(
                        successful=False,
                        reason=f'pwm_max({p.value}) 必须大于 pwm_min({current_min})')
            elif p.name == 'pwm_min':
                if not isinstance(p.value, int):
                    return SetParametersResult(successful=False, reason='pwm_min 必须是整数')
                if not HARDWARE_PWM_MIN <= p.value <= HARDWARE_PWM_MAX:
                    return SetParametersResult(
                        successful=False,
                        reason=f'pwm_min={p.value} 超出电气极限 {HARDWARE_PWM_MIN}~{HARDWARE_PWM_MAX}')
                if p.value >= current_max:
                    return SetParametersResult(
                        successful=False,
                        reason=f'pwm_min({p.value}) 必须小于 pwm_max({current_max})')
            elif p.name == 'baudrate':
                if not isinstance(p.value, int):
                    return SetParametersResult(successful=False, reason='baudrate 必须是整数')
                if not MIN_BAUDRATE <= p.value <= MAX_BAUDRATE:
                    return SetParametersResult(
                        successful=False,
                        reason=f'baudrate={p.value} 不支持，合法区间 {MIN_BAUDRATE}~{MAX_BAUDRATE}')
            elif p.name == 'servo_ids':
                ids = list(p.value) if not isinstance(p.value, str) else []
                if not ids:
                    return SetParametersResult(successful=False, reason='servo_ids 不能为空')
                bad = [i for i in ids if not MIN_ID <= int(i) <= MAX_ID]
                if bad:
                    return SetParametersResult(
                        successful=False, reason=f'servo_ids 含非法 ID {bad}（合法 {MIN_ID}~{MAX_ID}）')
            elif p.name == 'port':
                if not isinstance(p.value, str) or not p.value.strip():
                    return SetParametersResult(successful=False, reason='port 不能为空')
            elif p.name == 'default_duration_ms':
                if not isinstance(p.value, int) or not 0 < p.value <= MAX_DURATION_MS:
                    return SetParametersResult(
                        successful=False, reason=f'default_duration_ms 必须在 1~{MAX_DURATION_MS}')
            elif p.name == 'position_tolerance':
                if not isinstance(p.value, int) or p.value <= 0:
                    return SetParametersResult(successful=False, reason='position_tolerance 必须为正整数')
            elif p.name == 'feedback_period':
                if not isinstance(p.value, float) or not 0.02 <= p.value <= 1.0:
                    return SetParametersResult(
                        successful=False, reason='feedback_period 必须在 0.02~1.0 s')
            elif p.name == 'watchdog_timeout':
                if not isinstance(p.value, float) or not 0.2 <= p.value <= 10.0:
                    return SetParametersResult(
                        successful=False, reason='watchdog_timeout 必须在 0.2~10.0 s')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # 底层动作（Action 与话题共用这一条路径）
    # ------------------------------------------------------------------
    def _ensure_torque(self, servo_id):
        if False:  # 实测舵机动作数轮后会掉力，每次移动前必须重新 PULR（缓存判断已废弃）
            return
        self._link.transact(build_command(servo_id, CMD_TORQUE_ON), expect_response=False)
        self._loaded_ids.add(servo_id)
        time.sleep(0.05)  # PULR 与位置帧背靠背会被板子丢帧，留 50ms 间隔
        self.get_logger().info(f'舵机 {servo_id:03d} 恢复扭力 PULR')

    def send_move(self, servo_id, pwm, duration_ms):
        """真正下发一条位置指令。Action 与 /servo/command 都走这里。"""
        if self._watchdog_tripped:
            raise RuntimeError(
                'watchdog_latched: 看门狗已触发并卸力，拒绝一切动作；'
                '需外部确认：ros2 param set /servo_driver watchdog_reset true')
        with self._cmd_lock:
            self._ensure_torque(servo_id)
            cmd = build_command(servo_id, POSITION_TEMPLATE.format(pwm=pwm, duration=duration_ms))
            self._link.transact(cmd, expect_response=False)
            self._last_pwm[servo_id] = pwm
            # 记录预计结束时刻，心跳在这之前不打扰（否则会把正在走的斜坡重新计时）
            self._moving_until[servo_id] = time.monotonic() + duration_ms / 1000.0 + 0.3
            self.get_logger().info(f'TX {cmd}')
            self._publish_state(servo_id, pwm)

    def _heartbeat(self):
        """位置帧心跳 + /servo/state 心跳。

        两件事：
        1. 重发每个舵机最后一次目标位置（板子帧走完会停发 PWM，舵机失信号会
           滴滴响并泄力）；正在执行动作的舵机跳过，避免打断它自己的斜坡。
        2. 持续发布 /servo/state —— 这同时是安全看门狗监视的心跳源，
           超过 watchdog_timeout 没更新就会触发 PULK 卸力闩锁。
        """
        period = float(self.param('heartbeat_period'))
        if period <= 0:
            return
        # 先刷 ROS 心跳：定时器还能响 = 节点活着（串口没就绪也要发）
        self._last_hb = time.monotonic()
        center = self.center_pwm
        now = time.monotonic()
        duration = max(1000, int(self.param('default_duration_ms')))
        for servo_id in list(self.param('servo_ids')):
            pwm = self._last_pwm.get(servo_id, center)
            self._publish_state(servo_id, pwm)
            if not self._link.is_open or now < self._moving_until.get(servo_id, 0.0):
                continue
            try:
                with self._cmd_lock:
                    cmd = build_command(
                        servo_id, POSITION_TEMPLATE.format(pwm=pwm, duration=duration))
                    self._link.transact(cmd, expect_response=False)
            except (RuntimeError, serial.SerialException, OSError) as exc:
                self.get_logger().warn(f'心跳重发失败 id={servo_id}: {exc}')

    def _watchdog_loop(self):
        """安全看门狗（独立线程）：监视 /servo/state 心跳。

        超过 watchdog_timeout 没有心跳 → 自动给所有舵机 PULK 卸力、打 ERROR、
        置闩锁。闩锁后 send_move 直接拒绝，节点"之后不许自己乱动"，
        必须外部确认（ros2 param set /servo_driver watchdog_reset true）才允许恢复。
        """
        while True:
            time.sleep(0.1)
            if self._watchdog_tripped:
                continue
            if time.monotonic() - self._wd_boot < 3.0:
                continue   # 启动宽限期：等心跳定时器发出第一跳
            timeout = float(self.param('watchdog_timeout'))
            if time.monotonic() - self._last_hb <= timeout:
                continue
            self._watchdog_tripped = True
            self.get_logger().error(
                f'安全看门狗触发：/servo/state 心跳超过 {timeout:.1f}s 未更新，'
                '全部舵机 PULK 卸力并闩锁；恢复需外部确认：'
                'ros2 param set /servo_driver watchdog_reset true')
            for servo_id in list(self.param('servo_ids')):
                try:
                    with self._cmd_lock:
                        self._link.transact(
                            build_command(servo_id, CMD_TORQUE_OFF), expect_response=False)
                except (RuntimeError, serial.SerialException, OSError) as exc:
                    self.get_logger().error(f'看门狗 PULK 失败 id={servo_id}: {exc}')



    def read_position(self, servo_id):
        # PRAD frames sent back-to-back with position frames get dropped by
        # the board; retry up to 3 times with a 0.3s gap between attempts.
        for _ in range(3):
            raw = self._link.transact(build_command(servo_id, CMD_READ_POS))
            match = POSITION_PATTERN.search(raw.decode("ascii", errors="ignore"))
            if match:
                return int(match.group(1))
            time.sleep(0.3)
        self.get_logger().warn("PRAD no valid echo after 3 tries, id=%03d" % servo_id)
        return None

    def _publish_state(self, servo_id, pwm):
        msg = ServoCommand()
        msg.id = servo_id
        msg.pwm = pwm
        msg.duration_ms = 0
        self._state_pub.publish(msg)

    def startup_reset(self):
        """上电复位：每个舵机 PULR + 回中位，避免"舵机纹丝不动"。"""
        ids = list(self.param('servo_ids'))
        duration = int(self.param('default_duration_ms'))
        center = self.center_pwm
        if not self._link.is_open:
            self.get_logger().warn('串口未就绪，跳过启动复位（串口打开后请手动回中位或重启节点）')
            return
        for servo_id in ids:
            try:
                self._ensure_torque(servo_id)
                cmd = build_command(servo_id, POSITION_TEMPLATE.format(pwm=center, duration=duration))
                self._link.transact(cmd, expect_response=False)
                self._last_pwm[servo_id] = center
            except (RuntimeError, serial.SerialException) as exc:
                self.get_logger().error(f'启动复位失败 id={servo_id}: {exc}')
                return
        self.get_logger().info(f'启动复位完成：{ids} -> 中位 {center}')

    def torque_off(self, servo_id):
        self._link.transact(build_command(servo_id, CMD_TORQUE_OFF), expect_response=False)
        self._loaded_ids.discard(servo_id)

    # ------------------------------------------------------------------
    # 校验（goal 与话题共用）
    # ------------------------------------------------------------------
    def validate(self, servo_id, pwm, duration_ms):
        """返回 (ok, reason)。每次读参数，保证 param set 当帧生效。"""
        ids = [int(i) for i in self.param('servo_ids')]
        pwm_min, pwm_max = int(self.param('pwm_min')), int(self.param('pwm_max'))
        if servo_id not in ids:
            return False, f'ID {servo_id} 不在 servo_ids {ids} 中'
        if not pwm_min <= pwm <= pwm_max:
            return False, f'pwm={pwm} 超出安全区间 {pwm_min}~{pwm_max}'
        if duration_ms < 0 or duration_ms > MAX_DURATION_MS:
            return False, f'duration_ms={duration_ms} 非法（0~{MAX_DURATION_MS}）'
        return True, ''

    # ------------------------------------------------------------------
    # Action 回调
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request):
        ok, reason = self.validate(int(goal_request.id), int(goal_request.pwm),
                                   int(goal_request.duration_ms))
        if not ok:
            self.get_logger().warn(f'goal 被拒绝: {reason}')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'goal 接受: id={goal_request.id} pwm={goal_request.pwm} '
            f'duration={goal_request.duration_ms}')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('收到取消请求，立即接受')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        servo_id, target = int(req.id), int(req.pwm)
        duration_ms = int(req.duration_ms) or int(self.param('default_duration_ms'))
        period = float(self.param('feedback_period'))
        tolerance = int(self.param('position_tolerance'))

        start_pwm = self._last_pwm.get(servo_id, self.center_pwm)
        result = MoveServo.Result()
        feedback = MoveServo.Feedback()

        self.get_logger().info(
            f'执行: id={servo_id} {start_pwm} -> {target}，{duration_ms} ms')
        try:
            self.send_move(servo_id, target, duration_ms)
        except (RuntimeError, serial.SerialException) as exc:
            result.success = False
            result.final_pwm = 0
            result.message = f'serial_error: {exc}'
            goal_handle.abort()
            return result

        started = time.monotonic()
        total = duration_ms / 1000.0
        while True:
            elapsed = time.monotonic() - started
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f'动作被取消，回中位 {self.center_pwm}')
                try:
                    self.send_move(servo_id, self.center_pwm, int(self.param('default_duration_ms')))
                except (RuntimeError, serial.SerialException):
                    pass
                goal_handle.canceled()
                result.success = False
                result.final_pwm = self.center_pwm
                result.message = 'canceled'
                return result
            if elapsed >= total:
                break
            progress = min(1.0, elapsed / total) if total > 0 else 1.0
            feedback.current_pwm = int(round(start_pwm + (target - start_pwm) * progress))
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        # 到位校验：PRAD 实测
        try:
            actual = self.read_position(servo_id)
        except (RuntimeError, serial.SerialException) as exc:
            result.success = False
            result.final_pwm = 0
            result.message = f'serial_error: {exc}'
            goal_handle.abort()
            return result

        if actual is None:
            result.success = False
            result.final_pwm = 0
            result.message = 'PRAD 无回显，无法确认到位'
        elif abs(actual - target) > tolerance:
            # 实测：PULR 使能需约 1s，紧随其后的首帧会被板子忽略（回中位必挂）。
            # 补救：等使能完成后重发一次位置帧（舵机已上力，不再 PULR），再重读。
            pwm = target; time.sleep(0.6)
            self._link.transact(
                build_command(servo_id, POSITION_TEMPLATE.format(pwm=pwm, duration=duration_ms)),
                expect_response=False)
            self._last_pwm[servo_id] = pwm
            time.sleep(duration_ms / 1000.0 + 0.2)
            try:
                actual = self.read_position(servo_id)
            except (RuntimeError, serial.SerialException) as exc:
                result.success = False
                result.final_pwm = 0
                result.message = f'serial_error: {exc}'
                goal_handle.abort()
                return result
            if actual is None:
                result.success = False
                result.final_pwm = 0
                result.message = 'PRAD 无回显(补救后)'
            elif abs(actual - target) > tolerance:
                result.success = False
                result.final_pwm = actual
                result.message = f'未到位(补救后): 目标 {target} 实际 {actual}，偏差 {actual - target}'
            else:
                result.success = True
                result.final_pwm = actual
                result.message = f'ok(补救): 目标 {target} 实际 {actual}'
        else:
            result.success = True
            result.final_pwm = actual
            result.message = f'ok: 目标 {target} 实际 {actual}'
        goal_handle.succeed()
        self.get_logger().info(f'完成: {result.message}')
        return result

    # ------------------------------------------------------------------
    # /servo/command：内部调试总线，bag 回放靠它（与 Action 同一发送路径）
    # ------------------------------------------------------------------
    def command_callback(self, msg):
        servo_id, pwm = int(msg.id), int(msg.pwm)
        duration_ms = int(msg.duration_ms) or int(self.param('default_duration_ms'))
        ok, reason = self.validate(servo_id, pwm, duration_ms)
        if not ok:
            self.get_logger().warn(f'/servo/command 忽略: {reason}')
            return
        try:
            self.send_move(servo_id, pwm, duration_ms)
        except (RuntimeError, serial.SerialException) as exc:
            self.get_logger().error(f'/servo/command 发送失败: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    # num_threads 必须 ≥ 并发 goal 数 + 定时器数：VM 核数少时默认线程数会被
    # 并发 execute_callback 占满，心跳定时器饿死导致看门狗误触发
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
