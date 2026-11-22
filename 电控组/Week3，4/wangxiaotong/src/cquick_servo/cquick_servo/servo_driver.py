"""servo_driver：串口总线舵机驱动 + MoveServo Action。

数据流：
    goal → goal_callback 校验 → handle_accepted_callback → execute_callback
        → _send_move（唯一发送出口）→ "#001P1700T1000!" → 串口
        → feedback 循环（≥5Hz）
        → 结束前 PRAD 校验：差 > ±prad_tolerance → 自动重发（加分项 +3）
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

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand, ServoState
from std_msgs.msg import Empty

from cquick_servo.protocol import (
    DURATION_MAX_MS,
    ID_MAX,
    ID_MIN,
    ProtocolError,
    build_move,
    build_query,
    build_torque_on,
    build_unload,
    parse_pwm,
)

NODE_NAME = 'servo_driver'
ACTION_NAME = 'move_servo'
ALLOWED_BAUDRATES = (9600, 115200, 1000000)
PWM_ELEC_MIN = 500
PWM_ELEC_MAX = 2500


class ServoDriver(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # MultiThreadedExecutor 下，command 回调和 execute 回调会真的并行。
        # 没有这把锁，两帧 ASCII 会交错写进串口，舵机直接乱走。
        self._io_lock = threading.RLock()

        self._ser = None
        self._torque_on = set()
        self._last_pwm = {}
        self._last_heartbeat = time.monotonic()
        self._watchdog_armed = False

        self._declare_parameters()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._cb_group = ReentrantCallbackGroup()

        self._open_serial()

        self._self_check()

        self._state_pub = self.create_publisher(ServoState, '/servo/state', 10)
        self._command_sub = self.create_subscription(
            ServoCommand, '/servo/command', self._command_cb, 10,
            callback_group=self._cb_group)

        self._action_server = ActionServer(
            self, MoveServo, ACTION_NAME,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            handle_accepted_callback=self._handle_accepted_cb,
            callback_group=self._cb_group,
        )

        self.create_timer(1.0 / self._param('state_rate_hz'), self._publish_state)
        self.create_subscription(
            Empty, '/servo/heartbeat', self._heartbeat_cb, 10,
            callback_group=self._cb_group)
        self.create_timer(0.2, self._watchdog_tick)
        self.create_timer(2.0, self._try_reopen_serial)

        if self._param('startup_reset'):
            self._startup_reset()

    # ------------------------------------------------------------ 参数

    def _param(self, name):
        return self.get_parameter(name).value

    def _servo_ids(self):
        return [int(i) for i in self._param('servo_ids')]

    def _declare_parameters(self):
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('center_pwm', 1500)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)
        self.declare_parameter('feedback_hz', 5.0)
        self.declare_parameter('prad_tolerance', 30)
        self.declare_parameter('prad_retry', 1)
        self.declare_parameter('serial_timeout', 0.2)
        self.declare_parameter('state_rate_hz', 5.0)
        self.declare_parameter('watchdog_enabled', True)
        self.declare_parameter('watchdog_timeout', 1.0)

    def _self_check(self):
        names = ('port', 'baudrate', 'pwm_min', 'pwm_max', 'center_pwm',
                 'default_duration_ms', 'servo_ids', 'startup_reset',
                 'feedback_hz', 'prad_tolerance', 'prad_retry',
                 'serial_timeout', 'state_rate_hz',
                 'watchdog_enabled', 'watchdog_timeout')
        cur = {n: self._param(n) for n in names}
        self.get_logger().info(
            '参数自检：' + ', '.join(f'{k}={v}' for k, v in cur.items()))

        lo, hi, c = cur['pwm_min'], cur['pwm_max'], cur['center_pwm']
        ids = [int(i) for i in cur['servo_ids']]
        problems = []
        if lo >= hi:
            problems.append(f'pwm_min({lo}) 必须小于 pwm_max({hi})')
        if not (lo <= c <= hi):
            problems.append(f'center_pwm({c}) 不在 [{lo}, {hi}] 内')
        if not ids:
            problems.append('servo_ids 为空')
        if len(set(ids)) != len(ids):
            problems.append('servo_ids 有重复')
        if cur['baudrate'] not in (9600, 115200, 1000000):
            problems.append(f'baudrate({cur["baudrate"]}) 不在常用档位')

        if problems:
            self.get_logger().error('参数自检：FAIL - ' + '；'.join(problems))
        else:
            self.get_logger().info(
                f'参数自检：PASS（{len(ids)} 个舵机，行程 {lo}~{hi}，'
                f'中心 {c}，{cur["baudrate"]} 8-N-1）')
    def _on_set_parameters(self, params):
        pending = {p.name: p.value for p in params}

        bounds = {}
        for name in ('pwm_min', 'pwm_max', 'center_pwm'):
            value = pending.get(name, self._param(name))
            try:
                bounds[name] = int(value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False, reason=f'{name} 必须是整数')
            if not PWM_ELEC_MIN <= bounds[name] <= PWM_ELEC_MAX:
                return SetParametersResult(
                    successful=False,
                    reason=f'{name}={bounds[name]} 超出电气范围 '
                           f'{PWM_ELEC_MIN}~{PWM_ELEC_MAX}')
        if bounds['pwm_min'] >= bounds['pwm_max']:
            return SetParametersResult(
                successful=False, reason='pwm_min 必须小于 pwm_max')
        if 'baudrate' in pending and pending['baudrate'] not in ALLOWED_BAUDRATES:
            return SetParametersResult(
                successful=False, reason=f'不支持的波特率 {pending["baudrate"]}')
        if 'port' in pending and not str(pending['port']):
            return SetParametersResult(successful=False, reason='port 不能为空')

        self.get_logger().info(f'参数已更新：{list(pending.keys())}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ 串口

    def _open_serial(self):
        try:
            self._ser = serial.serial_for_url(
                str(self._param('port')),
                baudrate=int(self._param('baudrate')),
                timeout=float(self._param('serial_timeout')),
                write_timeout=1.0,
            )
        except (serial.SerialException, OSError, ValueError) as exc:
            self._ser = None
            self.get_logger().error(f'打开串口失败：{exc}（节点继续运行）')
            return False
        self.get_logger().info(
            f'串口已打开：{self._param("port")} @ {self._param("baudrate")} 8-N-1')
        return True

    def _ensure_serial(self):
        return self._ser is not None and self._ser.is_open

    def _write(self, command):
        with self._io_lock:
            if not self._ensure_serial():
                self.get_logger().warn(f'串口不可用，丢弃：{command}')
                return False
            try:
                self._ser.write(command.encode('ascii'))
                self._ser.flush()
                time.sleep(0.012)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'写串口失败：{exc}')
                return False
        return True

    def _read_frame(self):
        """读到 '!' 返回整帧，超时返回 None。绝不抛异常。"""
        with self._io_lock:
            if not self._ensure_serial():
                return None
            deadline = time.monotonic() + float(self._param('serial_timeout'))
            buf = ''
            while time.monotonic() < deadline:
                chunk = self._ser.read(1)
                if not chunk:
                    continue
                ch = chunk.decode('ascii', 'ignore')
                buf += ch
                if ch == '!':
                    return buf
            return buf or None

    def _query_pwm(self, servo_id):
        if not self._ensure_serial():
            return None
        with self._io_lock:
            self._ser.reset_input_buffer()
        if not self._write(build_query(servo_id, 'PRAD')):
            return None
        pwm = parse_pwm(self._read_frame())
        if pwm is not None:
            self._last_pwm[servo_id] = pwm
        return pwm

    # ------------------------------------------------------------ 发送

    def _ensure_torque(self, servo_id):
        if servo_id in self._torque_on:
            return True
        if not self._write(build_torque_on(servo_id)):
            return False
        self._torque_on.add(servo_id)
        return True

    def _send_move(self, servo_id, pwm, duration_ms):
        """唯一的移动发送出口。Action 和 /servo/command 都走这里。"""
        if not self._ensure_torque(servo_id):
            return False
        try:
            frame = build_move(servo_id, pwm, duration_ms)
        except ProtocolError as exc:
            self.get_logger().error(f'非法指令：{exc}')
            return False
        self.get_logger().info(f'SEND {frame}')
        if not self._write(frame):
            return False
        self._last_pwm[servo_id] = pwm
        return True

    # ------------------------------------------------------------ 校验

    def _validate(self, servo_id, pwm, duration_ms):
        """返回 None 表示合法，否则返回给人看的原因。每次都实时读参数。"""
        if servo_id not in self._servo_ids():
            return f'id {servo_id} 不在 servo_ids {self._servo_ids()} 里'
        low = int(self._param('pwm_min'))
        high = int(self._param('pwm_max'))
        if not low <= pwm <= high:
            return f'pwm {pwm} 超出安全区间 [{low}, {high}]'
        if duration_ms <= 0:
            return f'duration_ms {duration_ms} 必须 > 0'
        if duration_ms > DURATION_MAX_MS:
            return f'duration_ms {duration_ms} 超过上限 {DURATION_MAX_MS}'
        return None

    # ------------------------------------------------------------ Action

    def _goal_cb(self, goal_request):
        reason = self._validate(
            int(goal_request.id), int(goal_request.pwm), int(goal_request.duration_ms))
        if reason:
            self.get_logger().warn(f'goal 被拒绝：{reason}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn('收到取消请求')
        return CancelResponse.ACCEPT

    def _handle_accepted_cb(self, goal_handle):
        # 不调 execute()，goal 会永远停在 pending，客户端一直等
        goal_handle.execute()

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        servo_id = int(goal.id)
        target = int(goal.pwm)
        duration_ms = int(goal.duration_ms) or int(self._param('default_duration_ms'))
        start_pwm = self._last_pwm.get(servo_id, int(self._param('center_pwm')))

        # ACCEPT 之后、执行之前，参数可能已被 ros2 param set 改过，必须再校验一次
        reason = self._validate(servo_id, target, duration_ms)
        if reason:
            goal_handle.abort()
            result = MoveServo.Result()
            result.success = False
            result.message = f'rejected: {reason}'
            return result

        if not self._send_move(servo_id, target, duration_ms):
            goal_handle.abort()
            result = MoveServo.Result()
            result.success = False
            result.message = 'serial error: 指令没发出去'
            return result

        if not self._feedback_loop(goal_handle, servo_id, target, duration_ms, start_pwm):
            return self._finish_cancel(goal_handle, servo_id)
        return self._verify_and_finish(goal_handle, servo_id, target, duration_ms)

    def _feedback_loop(self, goal_handle, servo_id, target, duration_ms, start_pwm):
        """按 duration 走完，期间按 feedback_hz 发进度。返回 False 表示被取消。"""
        total = max(0.001, duration_ms / 1000.0)
        period = 1.0 / max(1.0, float(self._param('feedback_hz')))
        start = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                return False

            elapsed = time.monotonic() - start
            progress = min(1.0, elapsed / total)

            fb = MoveServo.Feedback()
            # 插值而非每帧 PRAD：每帧 PRAD 要等满 0.2s 超时，1 秒的动作会拖成 2 秒
            fb.current_pwm = int(max(0, min(65535, start_pwm + (target - start_pwm) * progress)))
            fb.progress = float(progress)
            goal_handle.publish_feedback(fb)

            if elapsed >= total:
                return True
            time.sleep(period)

    def _verify_and_finish(self, goal_handle, servo_id, target, duration_ms):
        tolerance = int(self._param('prad_tolerance'))
        retries = int(self._param('prad_retry'))
        result = MoveServo.Result()

        actual = self._query_pwm(servo_id)
        attempt = 0
        while (actual is not None and abs(actual - target) > tolerance
               and attempt < retries):
            attempt += 1
            self.get_logger().warn(
                f'#{servo_id:03d} 回读 {actual}，目标 {target}，'
                f'差 {actual - target:+d} 超 ±{tolerance} → 重发第 {attempt} 次')
            if not self._send_move(servo_id, target, duration_ms):
                break
            time.sleep(duration_ms / 1000.0 + 0.1)
            actual = self._query_pwm(servo_id)

        if actual is None:
            goal_handle.abort()
            result.success = False
            result.message = 'PRAD 无回显，无法确认是否到位'
        elif abs(actual - target) > tolerance:
            goal_handle.abort()
            result.success = False
            result.final_pwm = int(actual)
            result.message = (f'未到位：目标 {target} 实际 {actual}'
                              f'（差 {actual - target:+d}，阈值 ±{tolerance}）')
        else:
            goal_handle.succeed()
            result.success = True
            result.final_pwm = int(actual)
            result.message = 'ok'

        self.get_logger().info(
            f'#{servo_id:03d} result: success={result.success} '
            f'final_pwm={result.final_pwm} message={result.message}')
        return result

    def _finish_cancel(self, goal_handle, servo_id):
        center = int(self._param('center_pwm'))
        duration = int(self._param('default_duration_ms'))
        self.get_logger().warn(f'#{servo_id:03d} 取消 → 立即回中位 {center}')
        self._send_move(servo_id, center, duration)
        goal_handle.canceled()
        result = MoveServo.Result()
        result.success = False
        result.final_pwm = center
        result.message = 'canceled'
        return result

    # ------------------------------------------------------------ 话题

    def _command_cb(self, msg):
        duration_ms = int(msg.duration_ms) or int(self._param('default_duration_ms'))
        reason = self._validate(int(msg.id), int(msg.pwm), duration_ms)
        if reason:
            self.get_logger().warn(f'/servo/command 丢弃：{reason}')
            return
        self._send_move(int(msg.id), int(msg.pwm), duration_ms)

    def _publish_state(self):
        for servo_id in self._servo_ids():
            msg = ServoState()
            msg.id = servo_id
            msg.pwm = self._last_pwm.get(servo_id, int(self._param('center_pwm')))
            msg.torque_enabled = servo_id in self._torque_on
            msg.online = self._ensure_serial()
            self._state_pub.publish(msg)
    # ------------------------------------------------------------ 看门狗

    def _heartbeat_cb(self, _msg):
        self._last_heartbeat = time.monotonic()
        self._watchdog_armed = True

    def _watchdog_tick(self):
        if not self._watchdog_armed:
            return
        if not bool(self._param('watchdog_enabled')):
            return
        if not self._torque_on:
            return
        silence = time.monotonic() - self._last_heartbeat
        if silence < float(self._param('watchdog_timeout')):
            return
        self.get_logger().error(f'看门狗：{silence:.1f}s 无心跳，全部卸力')
        for servo_id in list(self._torque_on):
            self._write(build_unload(servo_id))
        self._torque_on.clear()
        
    def _try_reopen_serial(self):
        """串口掉线后每 2 秒尝试重开一次。

        _ser 为 None 或已关闭时才会进来；打开用的是实时参数，
        所以拔插后设备号变了（ttyUSB0 -> ttyUSB2）也不用重启节点。
        """
        if self._ser is not None and self._ser.is_open:
            return

        port = self._param('port')
        try:
            self._ser = serial.serial_for_url(
                port,
                baudrate=int(self._param('baudrate')),
                timeout=float(self._param('serial_timeout')),
                write_timeout=1.0,
            )
            self._reopen_attempts = 0
            self.get_logger().info(
                f'串口已重新打开：{port} @ {int(self._param("baudrate"))} 8-N-1')
        except (serial.SerialException, OSError) as exc:
            self._reopen_attempts = getattr(self, '_reopen_attempts', 0) + 1
            if self._reopen_attempts <= 3:
                self.get_logger().warn(
                    f'重连失败（第 {self._reopen_attempts} 次）：{exc}')
            else:
                self.get_logger().debug(f'重连失败：{exc}')

    # ------------------------------------------------------------ 其他

    def _startup_reset(self):
        if not self._ensure_serial():
            self.get_logger().warn('串口未打开，跳过启动回中位')
            return
        center = int(self._param('center_pwm'))
        duration = int(self._param('default_duration_ms'))
        self.get_logger().info(f'启动回中位：{self._servo_ids()} -> {center}')
        for servo_id in self._servo_ids():
            self._send_move(servo_id, center, duration)

    def emergency_unload(self):
        for servo_id in self._servo_ids():
            self._write(build_unload(servo_id))
        self._torque_on.clear()
        if self._ser is not None:
            self._ser.close()
            self._ser = None


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
        node.emergency_unload()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
