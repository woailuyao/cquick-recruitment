#!/usr/bin/env python3
"""CQuick 舵机驱动节点。

对外**只暴露 Action**（/move_servo），另外订阅一个内部调试话题 /servo/command，
唯一作用是让 ros2 bag 能回放。两者走同一个底层发送函数 _send_move()。

七项配置全部来自参数，代码里不出现设备名 / 波特率 / 安全限位的字面量。
"""

import re
import threading
import time
from collections import deque

import rclpy
import serial
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

# ---------------------------------------------------------------------------
# 协议常量：这是控制板协议本身，不是配置，允许写死
# ---------------------------------------------------------------------------
FRAME_START = '#'
FRAME_END = '!'
CMD_READ_ID = 'PID'
CMD_READ_POS = 'PRAD'
CMD_TORQUE_ON = 'PULR'
CMD_TORQUE_OFF = 'PULK'
CMD_TARGET_PWM = 'P'
CMD_DURATION = 'T'
ID_WIDTH = 3
ID_MIN = 1
ID_MAX = 254

# 舵机中立位（协议规定的 1500），用于取消后回中位
NEUTRAL_PWM = 1500

# 电气极限：舵机能接受的 PWM 范围（数据手册），不是安全范围，不可配置
PWM_ELECTRICAL_MIN = 500
PWM_ELECTRICAL_MAX = 2500

# 波特率最低/最高与步进（常见波特率都是 1200 的整数倍）
BAUDRATE_MIN = 1200
BAUDRATE_MAX = 4000000
BAUDRATE_STEP = 1200

# 一次动作最长时长，超过认为是非法 duration
DURATION_LIMIT_MS = 30000

# 反馈频率：不低于 5 Hz，这里给 10 Hz
FEEDBACK_PERIOD_S = 0.1
# 反馈循环里读 PRAD 的单次超时，读不到就退回插值，不能把 5 Hz 拖死
FEEDBACK_QUERY_TIMEOUT_S = 0.05

# 动作结束后等舵机停稳再读回
SETTLE_TIME_S = 0.15
# 到位判定：连续两次 PRAD 差 ≤ 这个值，认为已经停住；最多等 STILL_TIMEOUT_S
STILL_TOL = 4
STILL_TIMEOUT_S = 1.0
# 取消时：卸力后等多久再重新上扭力（给控制板时间退出当前动作）
TORQUE_OFF_SETTLE_S = 0.15
# 单次查询应答超时
RESPONSE_TIMEOUT_S = 0.3

# 到位判据：读回值与目标的允许偏差（默认值，可被参数 arrive_tolerance 覆盖）
#
# 30 太紧：实测 ID 5（右髋偏航）目标 1650 稳定停在 1616~1617，误差 33~34，
# 只超 3 格就判"未到位"整个动作组中止。机械虚位 + 负载下的稳态误差属正常，
# 所以默认放宽到 50（≈6°），需要更严时用参数调回去。
ARRIVE_TOLERANCE = 50

# 启动后延迟多久执行 startup_reset（等串口和板卡就绪）
STARTUP_DELAY_S = 1.0


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        # ---------------- 参数：只在这里出现名字 ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('min_duration_ms', 1000)
        self.declare_parameter('arrive_tolerance', 50)
        self.declare_parameter('cancel_return_ms', 500)   # 取消后回中位用多久，要快
        self.declare_parameter('servo_ids', list(range(0, 11)))  # 整机 1~10；0 保留给任务2单轴的 bag 回放
        self.declare_parameter('startup_reset', True)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ---------------- 串口状态 ----------------
        self._serial = None
        self._serial_dirty = False          # port / baudrate 改了，下次用时重开
        self._serial_lock = threading.RLock()
        self._torque_ids = set()            # 已经发过 PULR 的舵机

        # ---------------- ROS 接口 ----------------
        # ReentrantCallbackGroup + MultiThreadedExecutor：取消回调才进得来
        self._cb_group = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        # 指令镜像：Action 收到的指令同步发到这里，bag 才能录到、才能回放
        self._command_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        # 回声抑制：记住自己刚镜像出去的若干条。
        # 不能只存"最后一条" —— 同一步里并发下发 2 个 goal 时，
        # 第二条会把第一条覆盖掉，第一条的回声就被当成外部指令再执行一遍。
        self._mirrored_pending = deque(maxlen=32)
        self._command_sub = self.create_subscription(
            ServoCommand, '/servo/command', self._on_command, 10,
            callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        # 不阻塞构造函数，延迟一点再做启动复位
        self._startup_timer = self.create_timer(
            STARTUP_DELAY_S, self._on_startup_tick,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            'servo_driver 已启动：端口=%s 波特率=%s 安全范围=%s~%s 舵机=%s'
            % (self.get_parameter('port').value,
               self.get_parameter('baudrate').value,
               self.get_parameter('pwm_min').value,
               self.get_parameter('pwm_max').value,
               list(self.get_parameter('servo_ids').value)))

    # =====================================================================
    # 参数
    # =====================================================================
    def _on_set_parameters(self, params):
        """唯一能拒绝非法参数的地方。候选值先合并再整体校验。"""
        candidate = {
            'port': self.get_parameter('port').value,
            'baudrate': int(self.get_parameter('baudrate').value),
            'pwm_min': int(self.get_parameter('pwm_min').value),
            'pwm_max': int(self.get_parameter('pwm_max').value),
            'default_duration_ms': int(self.get_parameter('default_duration_ms').value),
            'servo_ids': [int(v) for v in self.get_parameter('servo_ids').value],
            'startup_reset': bool(self.get_parameter('startup_reset').value),
        }
        for param in params:
            if param.name in candidate:
                candidate[param.name] = param.value

        for param in params:
            name = param.name
            if name not in candidate:
                continue
            value = candidate[name]

            if name == 'port':
                if not isinstance(value, str) or not value.strip():
                    return SetParametersResult(
                        successful=False, reason='port 必须是非空字符串，例如 /dev/ttyUSB0')
            elif name == 'baudrate':
                if not isinstance(value, int) or isinstance(value, bool):
                    return SetParametersResult(successful=False, reason='baudrate 必须是整数')
                if not (BAUDRATE_MIN <= value <= BAUDRATE_MAX) or value % BAUDRATE_STEP != 0:
                    return SetParametersResult(
                        successful=False,
                        reason='baudrate 必须在 %d~%d 之间且为 %d 的整数倍，收到 %d'
                               % (BAUDRATE_MIN, BAUDRATE_MAX, BAUDRATE_STEP, value))
            elif name in ('pwm_min', 'pwm_max'):
                if not isinstance(value, int) or isinstance(value, bool):
                    return SetParametersResult(successful=False, reason='%s 必须是整数' % name)
                if not (PWM_ELECTRICAL_MIN <= value <= PWM_ELECTRICAL_MAX):
                    return SetParametersResult(
                        successful=False,
                        reason='%s=%d 超出舵机电气极限 %d~%d'
                               % (name, value, PWM_ELECTRICAL_MIN, PWM_ELECTRICAL_MAX))
            elif name == 'default_duration_ms':
                if not isinstance(value, int) or isinstance(value, bool):
                    return SetParametersResult(
                        successful=False, reason='default_duration_ms 必须是整数')
                if not (0 <= value <= DURATION_LIMIT_MS):
                    return SetParametersResult(
                        successful=False,
                        reason='default_duration_ms 必须在 0~%d 之间，收到 %d'
                               % (DURATION_LIMIT_MS, value))
            elif name == 'servo_ids':
                if not value:
                    return SetParametersResult(successful=False, reason='servo_ids 不能为空')
                for servo_id in value:
                    if not isinstance(servo_id, int) or isinstance(servo_id, bool):
                        return SetParametersResult(
                            successful=False, reason='servo_ids 里只能是整数，收到 %r' % (servo_id,))
                    if not (ID_MIN <= servo_id <= ID_MAX):
                        return SetParametersResult(
                            successful=False,
                            reason='舵机 ID %d 超出 %d~%d' % (servo_id, ID_MIN, ID_MAX))
                if len(set(value)) != len(value):
                    return SetParametersResult(successful=False, reason='servo_ids 有重复项')
            elif name == 'startup_reset':
                if not isinstance(value, bool):
                    return SetParametersResult(
                        successful=False, reason='startup_reset 必须是 bool')

        # 组合校验：两个单看都合法，凑一起才打架
        if candidate['pwm_min'] >= candidate['pwm_max']:
            return SetParametersResult(
                successful=False,
                reason='pwm_min(%d) 必须小于 pwm_max(%d)'
                       % (candidate['pwm_min'], candidate['pwm_max']))

        # 端口/波特率变了，旧句柄作废
        for param in params:
            if param.name in ('port', 'baudrate'):
                self._serial_dirty = True
                self.get_logger().info('参数 %s -> %s，串口将在下次使用时重开'
                                       % (param.name, param.value))
            else:
                self.get_logger().info('参数 %s -> %s，下一帧起生效'
                                       % (param.name, param.value))
        return SetParametersResult(successful=True)

    # =====================================================================
    # 串口底层
    # =====================================================================
    def _ensure_serial(self):
        with self._serial_lock:
            if self._serial_dirty:
                self._close_serial_locked()
                self._serial_dirty = False
            if self._serial is not None and self._serial.is_open:
                return True
            port = self.get_parameter('port').value
            baudrate = int(self.get_parameter('baudrate').value)
            try:
                self._serial = serial.Serial(
                    port=port, baudrate=baudrate,
                    bytesize=8, parity='N', stopbits=1,
                    timeout=RESPONSE_TIMEOUT_S, write_timeout=RESPONSE_TIMEOUT_S)
                self._torque_ids.clear()
                self.get_logger().info('串口已打开：%s @ %d 8-N-1' % (port, baudrate))
                return True
            except Exception as exc:                     # noqa: BLE001
                self._serial = None
                self.get_logger().error('串口打开失败 %s @ %d：%s' % (port, baudrate, exc))
                return False

    def _close_serial_locked(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:                            # noqa: BLE001
                pass
        self._serial = None
        self._torque_ids.clear()

    def _frame(self, servo_id, body):
        return ('%s%0*d%s%s' % (FRAME_START, ID_WIDTH, servo_id, body, FRAME_END)).encode('ascii')

    def _write(self, servo_id, body):
        with self._serial_lock:
            if self._serial is None or not self._serial.is_open:
                return False
            try:
                self._serial.reset_input_buffer()
                self._serial.write(self._frame(servo_id, body))
                self._serial.flush()
                return True
            except Exception as exc:                     # noqa: BLE001
                self.get_logger().error('串口写入失败：%s' % exc)
                self._serial_dirty = True
                return False

    def _query_int(self, servo_id, body, token, timeout=None):
        """发一条查询指令并按 token 解析回包，失败返回 None。"""
        timeout = RESPONSE_TIMEOUT_S if timeout is None else timeout
        with self._serial_lock:
            if self._serial is None or not self._serial.is_open:
                return None
            try:
                self._serial.reset_input_buffer()
                self._serial.write(self._frame(servo_id, body))
                self._serial.flush()
                deadline = time.monotonic() + timeout
                buf = b''
                while time.monotonic() < deadline:
                    chunk = self._serial.read(self._serial.in_waiting or 1)
                    if chunk:
                        buf += chunk
                        if FRAME_END.encode('ascii') in buf:
                            break
            except Exception as exc:                     # noqa: BLE001
                self.get_logger().error('串口读取失败：%s' % exc)
                self._serial_dirty = True
                return None

        text = buf.decode('ascii', errors='ignore').strip()
        if not text:
            return None
        matched = re.findall(token + r'\s*(\d+)', text)
        if matched:
            return int(matched[-1])
        numbers = re.findall(r'\d+', text)
        if numbers:
            return int(numbers[-1])
        self.get_logger().warn('回包解析不出数字：%r' % text)
        return None

    def _read_position(self, servo_id, timeout=None):
        return self._query_int(servo_id, CMD_READ_POS, CMD_READ_POS, timeout=timeout)

    def _wait_until_still(self, servo_id, target_pwm):
        """等舵机位置稳定下来再返回。

        为什么要它：按 duration 死等然后读一次，读到的常常还差 20~30；
        上位机紧接着发下一条目标时，板子会把新目标丢掉（实测动作完全不动）。
        这里连续两次读数差 ≤ STILL_TOL 就认为停了，最多等 STILL_TIMEOUT_S。
        """
        deadline = time.monotonic() + STILL_TIMEOUT_S
        last = None
        while time.monotonic() < deadline:
            cur = self._read_position(servo_id)
            if cur is None:
                time.sleep(0.05)
                continue
            if last is not None and abs(int(cur) - int(last)) <= STILL_TOL:
                return int(cur)
            last = int(cur)
            time.sleep(0.05)
        return last if last is not None else self._read_position(servo_id)

    def _try_read_position(self, servo_id, timeout=None):
        """反馈用的"抢不到就跳过"版本。

        串口是单通道：10 个舵机同时动时，10 个线程各按 10 Hz 查 PRAD 会互相排队，
        实测把 1.2 s 的动作拖成 9 s。这里改成拿不到锁就直接返回 None，
        调用方用插值顶上 —— 结果判定用的最终读值不受影响（那边是阻塞的）。
        """
        if not self._serial_lock.acquire(blocking=False):
            return None
        try:
            return self._read_position(servo_id, timeout=timeout)
        finally:
            self._serial_lock.release()

    def _torque_on(self, servo_id):
        """上电后第一次动作前必须发 PULR，否则舵机纹丝不动。"""
        if servo_id in self._torque_ids:
            return True
        if not self._write(servo_id, CMD_TORQUE_ON):
            return False
        self._torque_ids.add(servo_id)
        self.get_logger().info('ID %d 恢复扭力（%s）' % (servo_id, CMD_TORQUE_ON))
        return True

    def _torque_off(self, servo_id):
        """PULK 卸力。

        取消时必须先用它把舵机从"正在执行的目标"里拉出来。
        实测（2026-11-21 真机）：运动过程中直接下发新的 P1500，控制板会锁死输出 ——
        PID/PRAD 仍能读，但对任何转动指令都不响应，只能断电恢复。
        """
        ok = self._write(servo_id, CMD_TORQUE_OFF)
        self._torque_ids.discard(servo_id)
        if ok:
            self.get_logger().warn('ID %d 卸力（%s）' % (servo_id, CMD_TORQUE_OFF))
        return ok

    def _send_move(self, servo_id, pwm, duration_ms):
        """唯一的动作发送函数：Action 和 /servo/command 都走这里。"""
        body = '%s%d%s%d' % (CMD_TARGET_PWM, int(pwm), CMD_DURATION, int(duration_ms))
        return self._write(servo_id, body)

    def _effective_duration(self, duration_ms):
        """把 duration 收敛到控制板能接受的区间。

        实测（2026-11-22 真机）：控制板对 T < 1000ms 的目标直接不执行 ——
        发了 1620/800ms，PRAD 一动不动；同样目标给 1000ms 就到 1594。
        所以这里强制下限，避免上层写小值时动作"静默失败"。
        """
        value = int(duration_ms)
        if value <= 0:
            value = int(self.get_parameter('default_duration_ms').value)
        floor = int(self.get_parameter('min_duration_ms').value)
        if value < floor:
            self.get_logger().warn(
                'duration %dms 低于控制板下限，已按 %dms 下发' % (value, floor))
            value = floor
        return value

    # =====================================================================
    # 启动复位
    # =====================================================================
    def _on_startup_tick(self):
        self._startup_timer.cancel()
        if not bool(self.get_parameter('startup_reset').value):
            self.get_logger().info('startup_reset=false，跳过启动回中位')
            return
        if not self._ensure_serial():
            return
        duration = self._effective_duration(0)
        for servo_id in list(self.get_parameter('servo_ids').value):
            self._torque_on(int(servo_id))
            self._send_move(int(servo_id), NEUTRAL_PWM, duration)
        self.get_logger().info('启动复位完成：%s 全部回 %d'
                               % (list(self.get_parameter('servo_ids').value), NEUTRAL_PWM))

    # =====================================================================
    # Action 回调
    # =====================================================================
    def _check_goal(self, request):
        """返回 None 表示合法，否则返回拒绝原因。每次都读实时参数。"""
        servo_ids = [int(v) for v in self.get_parameter('servo_ids').value]
        if int(request.id) not in servo_ids:
            return 'ID %d 不在 servo_ids %s 里' % (request.id, servo_ids)

        pwm_min = int(self.get_parameter('pwm_min').value)
        pwm_max = int(self.get_parameter('pwm_max').value)
        if not (pwm_min <= int(request.pwm) <= pwm_max):
            return 'pwm %d 超出安全范围 %d~%d' % (request.pwm, pwm_min, pwm_max)

        if int(request.duration_ms) > DURATION_LIMIT_MS:
            return 'duration_ms %d 超过上限 %d' % (request.duration_ms, DURATION_LIMIT_MS)
        return None

    def goal_callback(self, goal_request):
        reason = self._check_goal(goal_request)
        if reason is not None:
            self.get_logger().warn('goal 被拒绝：id=%d pwm=%d，原因：%s'
                                   % (goal_request.id, goal_request.pwm, reason))
            return GoalResponse.REJECT
        self.get_logger().info('goal 已接受：id=%d pwm=%d duration=%dms'
                               % (goal_request.id, goal_request.pwm, goal_request.duration_ms))
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # 显式接管：接受之后自己决定何时开始执行
        self.get_logger().info('goal 开始执行（handle_accepted）')
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('收到取消请求，接受')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        servo_id = int(request.id)
        target_pwm = int(request.pwm)
        duration_ms = self._effective_duration(request.duration_ms)

        # 接受之后再复核一次：中间可能有人把 pwm_max 改小了
        reason = self._check_goal(request)
        if reason is not None:
            return self._abort(goal_handle, 0, '参数在执行前发生变化：' + reason)

        if not self._ensure_serial():
            return self._abort(goal_handle, 0, '串口不可用：%s' % self.get_parameter('port').value)
        if not self._torque_on(servo_id):
            return self._abort(goal_handle, 0, '恢复扭力失败（PULR 发不出去）')

        start_pwm = self._read_position(servo_id)
        start_pwm = target_pwm if start_pwm is None else start_pwm

        if not self._send_move(servo_id, target_pwm, duration_ms):
            return self._abort(goal_handle, 0, '动作指令发送失败')
        self._publish_command(servo_id, target_pwm, duration_ms)   # 镜像到话题，供 bag 回放

        started_at = time.monotonic()
        duration_s = duration_ms / 1000.0
        feedback = MoveServo.Feedback()

        while True:
            # 每一轮都查取消：这是 ctrl+c 能停下来的唯一原因
            if goal_handle.is_cancel_requested:
                # 用 cancel_return_ms 而不是原 goal 的 duration：
                # 取消的意义是"快点停下回中位"，沿用一个 6 秒的长 duration 会变成慢慢挪回去
                ret_ms = int(self.get_parameter('cancel_return_ms').value)
                self.get_logger().warn(
                    'goal 被取消：ID %d 停止并回中位（%d ms）' % (servo_id, ret_ms))
                # 先卸力把舵机从当前目标里拉出来，再重新上扭力回中位。
                # 直接下发新目标会让控制板锁死输出（实测），必须这个顺序。
                self._torque_off(servo_id)
                self._wait(TORQUE_OFF_SETTLE_S)
                self._torque_on(servo_id)
                self._wait(0.05)
                self._send_move(servo_id, NEUTRAL_PWM, ret_ms)
                self._wait(ret_ms / 1000.0)
                actual = self._read_position(servo_id)
                goal_handle.canceled()
                result = MoveServo.Result()
                result.success = False
                result.final_pwm = int(actual) if actual is not None else 0
                result.message = 'canceled'
                return result

            elapsed = time.monotonic() - started_at
            progress = 1.0 if duration_s <= 0 else min(elapsed / duration_s, 1.0)

            actual = self._try_read_position(servo_id, timeout=FEEDBACK_QUERY_TIMEOUT_S)
            if actual is None:
                actual = int(start_pwm + (target_pwm - start_pwm) * progress)
            feedback.current_pwm = int(actual)
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)
            self._publish_state(servo_id, feedback.current_pwm, duration_ms)

            if progress >= 1.0:
                break
            self._wait(min(FEEDBACK_PERIOD_S, max(duration_s - elapsed, 0.01)))

        # 等"位置真的不动了"再返回，而不是死等 SETTLE_TIME_S。
        # 实测：按时间判定时，PRAD 还差 20~30 就返回，
        # 上位机紧接着发下一步，板子会丢掉这条新目标（动作直接不动）。
        final_pwm = self._wait_until_still(servo_id, target_pwm)

        result = MoveServo.Result()
        if final_pwm is None:
            result.success = False
            result.final_pwm = 0
            result.message = 'PRAD 读回失败，无法确认是否到位'
            goal_handle.abort()
            return result

        diff = int(final_pwm) - target_pwm
        result.final_pwm = int(final_pwm)
        tol = int(self.get_parameter('arrive_tolerance').value)
        if abs(diff) > tol:
            result.success = False
            result.message = '未到位：目标 %d，PRAD 读回 %d，差 %+d（允许 ±%d）' % (
                target_pwm, final_pwm, diff, tol)
            self.get_logger().warn(result.message)
            goal_handle.abort()
            return result

        result.success = True
        result.message = '到位：目标 %d，PRAD 读回 %d，差 %+d' % (target_pwm, final_pwm, diff)
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    def _abort(self, goal_handle, final_pwm, message):
        self.get_logger().error('goal 失败：%s' % message)
        goal_handle.abort()
        result = MoveServo.Result()
        result.success = False
        result.final_pwm = int(final_pwm)
        result.message = message
        return result

    @staticmethod
    def _wait(seconds):
        if seconds > 0:
            time.sleep(seconds)

    # =====================================================================
    # 调试话题（bag 回放用）
    # =====================================================================
    def _publish_state(self, servo_id, pwm, duration_ms):
        state = ServoCommand()
        state.id = int(servo_id)
        state.pwm = int(max(min(pwm, PWM_ELECTRICAL_MAX), PWM_ELECTRICAL_MIN))
        state.duration_ms = int(duration_ms)
        self._state_pub.publish(state)

    def _publish_command(self, servo_id, pwm, duration_ms):
        """把 Action 收到的指令镜像到 /servo/command。
        作用：bag 只录话题、录不到 Action，镜像后 bag play 才能复现动作。
        注意：本节点自己也订阅这个话题，所以要记下刚发的那条做回声抑制。
        """
        cmd = ServoCommand()
        cmd.id = int(servo_id)
        cmd.pwm = int(pwm)
        cmd.duration_ms = int(duration_ms)
        self._mirrored_pending.append((cmd.id, cmd.pwm, cmd.duration_ms))
        self._command_pub.publish(cmd)

    def _on_command(self, msg):
        servo_id = int(msg.id)
        # 回声抑制：这条是本节点刚镜像出去的，不再执行一次。
        # 用"待消费队列"而不是单值：并发的多条镜像指令各自都能对上，
        # 且只消费一次（同一条外部指令重复发两次仍会被执行两次）。
        key = (servo_id, int(msg.pwm), int(msg.duration_ms))
        if key in self._mirrored_pending:
            self._mirrored_pending.remove(key)
            return
        servo_ids = [int(v) for v in self.get_parameter('servo_ids').value]
        if servo_id not in servo_ids:
            self.get_logger().warn('/servo/command 收到未知 ID %d，丢弃' % servo_id)
            return
        if not self._ensure_serial():
            return
        if not self._torque_on(servo_id):
            return
        duration_ms = self._effective_duration(msg.duration_ms)
        if self._send_move(servo_id, msg.pwm, duration_ms):
            self.get_logger().info('/servo/command -> ID %d pwm %d %dms'
                                   % (servo_id, msg.pwm, duration_ms))

    def destroy_node(self):
        with self._serial_lock:
            self._close_serial_locked()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    # 线程数必须显式给：默认 = CPU 核数（这台 2 vCPU），
    # 10 个舵机同时动时只有 2 条线程跑 execute_callback，
    # goal 会被排队，1.2 s 的动作实测拖到 8 s。
    # 留出余量：最多 24 个舵机 + 取消/参数回调。
    executor = MultiThreadedExecutor(num_threads=26)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
