#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

FRAME_START = '#'
FRAME_END = '!'
ID_WIDTH = 3
CMD_READ_ID = 'PID'
CMD_READ_POS = 'PRAD'
CMD_TORQUE_ON = 'PULR'
CMD_TORQUE_OFF = 'PULK'
POSITION_TEMPLATE = 'P{pwm}T{duration}'
RESPONSE_TIMEOUT = 0.15
POSITION_PATTERN = re.compile(r'#\d{3}P(\d{4})')

HARDWARE_PWM_MIN, HARDWARE_PWM_MAX = 500, 2500
MIN_BAUDRATE, MAX_BAUDRATE = 1200, 4000000
MAX_DURATION_MS = 30000
MIN_ID, MAX_ID = 0, 254


def build_command(servo_id, body):
    return f'{FRAME_START}{servo_id:0{ID_WIDTH}d}{body}{FRAME_END}'


class SerialLink:

    def __init__(self, logger):
        self._logger = logger
        self._lock = threading.Lock()
        self._ser = None

    def open(self, port, baudrate):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
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

        self._loaded_ids = set()
        self._last_pwm = {}
        self._moving_until = {}
        self._watchdog_tripped = False
        self._last_hb = time.monotonic()
        self._cmd_lock = threading.Lock()
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
        self.create_timer(0.5, self._apply_serial_params)
        hb = float(self.param('heartbeat_period'))
        if hb > 0:
            self.create_timer(max(0.1, hb), self._heartbeat)
        self._last_hb = time.monotonic()
        self._wd_boot = time.monotonic()
        self._wd_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._wd_thread.start()

        if self.param('startup_reset'):
            self.startup_reset()

        self.get_logger().info(
            f'servo_driver 就绪: port={self.param("port")} baud={self.param("baudrate")} '
            f'pwm=[{self.param("pwm_min")},{self.param("pwm_max")}] '
            f'ids={list(self.param("servo_ids"))}')

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
        self.declare_parameter('position_tolerance', 30,
                               ParameterDescriptor(description='PRAD 到位判定容差'))
        self.declare_parameter('feedback_period', 0.2,
                               ParameterDescriptor(description='反馈周期 s'))
        self.declare_parameter('heartbeat_period', 0.5,
                               ParameterDescriptor(description='位置帧心跳周期 s；0 表示关闭'))
        self.declare_parameter('watchdog_timeout', 1.0,
                               ParameterDescriptor(
                                   description='看门狗阈值 s：/servo/state 超时未更新则'
                                               'PULK 并闩锁'))
        self.declare_parameter('watchdog_reset', False,
                               ParameterDescriptor(description='看门狗闩锁后外部确认恢复'))

    def param(self, name):
        return self.get_parameter(name).value

    @property
    def center_pwm(self):
        return (int(self.param('pwm_min')) + int(self.param('pwm_max'))) // 2

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

    def on_set_parameters(self, params):
        current_min = int(self.param('pwm_min'))
        current_max = int(self.param('pwm_max'))
        for p in params:
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

    def _ensure_torque(self, servo_id):
        self._link.transact(build_command(servo_id, CMD_TORQUE_ON), expect_response=False)
        self._loaded_ids.add(servo_id)
        time.sleep(0.05)
        self.get_logger().info(f'舵机 {servo_id:03d} 恢复扭力 PULR')

    def send_move(self, servo_id, pwm, duration_ms):
        if self._watchdog_tripped:
            raise RuntimeError(
                'watchdog_latched: 看门狗已触发并卸力，拒绝一切动作；'
                '需外部确认：ros2 param set /servo_driver watchdog_reset true')
        with self._cmd_lock:
            self._ensure_torque(servo_id)
            cmd = build_command(servo_id, POSITION_TEMPLATE.format(pwm=pwm, duration=duration_ms))
            self._link.transact(cmd, expect_response=False)
            self._last_pwm[servo_id] = pwm
            self._moving_until[servo_id] = time.monotonic() + duration_ms / 1000.0 + 0.3
            self.get_logger().info(f'TX {cmd}')
            self._publish_state(servo_id, pwm)

    def _heartbeat(self):
        period = float(self.param('heartbeat_period'))
        if period <= 0:
            return
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
        while True:
            time.sleep(0.1)
            if self._watchdog_tripped:
                continue
            if time.monotonic() - self._wd_boot < 3.0:
                continue
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

    def validate(self, servo_id, pwm, duration_ms):
        ids = [int(i) for i in self.param('servo_ids')]
        pwm_min, pwm_max = int(self.param('pwm_min')), int(self.param('pwm_max'))
        if servo_id not in ids:
            return False, f'ID {servo_id} 不在 servo_ids {ids} 中'
        if not pwm_min <= pwm <= pwm_max:
            return False, f'pwm={pwm} 超出安全区间 {pwm_min}~{pwm_max}'
        if duration_ms < 0 or duration_ms > MAX_DURATION_MS:
            return False, f'duration_ms={duration_ms} 非法（0~{MAX_DURATION_MS}）'
        return True, ''

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
