#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cquick_servo 串口驱动节点（第 3/4 周 · 任务 2）

对外只暴露 Action：/move_servo（MoveServo）
另外两个话题只为调试和 bag 回放服务：
  /servo/command  (ServoCommand)  ← 订阅：走和 Action 同一个底层发送函数，bag 回放靠它
  /servo/state    (ServoCommand)  → 发布：当前 PWM 状态，bag 要录它

硬编码红线（细则 2.2）：设备名 / 波特率 / 安全限位 / ID 表 / 默认时长 一律来自参数，
代码里只允许出现协议常量（# ! PID PRAD PULR PULK、P、T、ID 补 3 位）。
"""
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo.servo_protocol import (
    BAUD_CHOICES, CMD_ID, CMD_POS, CMD_TORQUE_OFF, CMD_TORQUE_ON,
    ELECTRIC_MAX, ELECTRIC_MIN, FEEDBACK_HZ, MAX_DURATION_MS,
    RECONNECT_INTERVAL_S, REPLY_TIMEOUT_S, STATE_HZ,
    build_move, build_query, parse_pwm)


class ServoSerialError(RuntimeError):
    """串口不可用 / 写失败。"""


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        # ---------------- 参数声明（名字只出现这一次）----------------
        # 固定设备名由 udev/99-cquick-servo.rules 绑定；没装规则的机器用
        # ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB0 覆盖
        self.declare_parameter('port', '/dev/cquick_servo')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.declare_parameter('startup_reset', True)

        self._lock = threading.Lock()
        self._ser = None
        self._last_pwm = {}
        self._cb_group = ReentrantCallbackGroup()
        # 串口掉线后限频重连，拔线期间不要把日志刷爆
        self._reconnect_lock = threading.Lock()
        self._last_reconnect = 0.0

        # 参数校验：只有注册了这个回调，才有权力拒绝非法值
        self.add_on_set_parameters_callback(self._on_param_set)

        self._open_serial()

        neutral = self._neutral()
        for sid in self._servo_ids():
            self._last_pwm[sid] = neutral

        self._state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        self.create_subscription(
            ServoCommand, '/servo/command', self._on_command, 10,
            callback_group=self._cb_group)
        self.create_timer(1.0 / STATE_HZ, self._publish_all_state,
                          callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            MoveServo,
            '/move_servo',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )

        if bool(self._param('startup_reset')):
            self._startup_reset()

        self.get_logger().info(
            'servo_driver 就绪：port=%s baud=%s 限位=[%d,%d] ids=%s'
            % (self._param('port'), self._param('baudrate'),
               self._pwm_min(), self._pwm_max(), self._servo_ids()))

    # ==================== 参数读取（每次都重新读，保证当帧生效）====================
    def _param(self, name):
        return self.get_parameter(name).value

    def _pwm_min(self):
        return int(self._param('pwm_min'))

    def _pwm_max(self):
        return int(self._param('pwm_max'))

    def _servo_ids(self):
        return [int(x) for x in self._param('servo_ids')]

    def _default_duration(self):
        return int(self._param('default_duration_ms'))

    def _neutral(self):
        """中立位 = 安全范围中点（限位改了它也自动跟着改，不写死 1500）。"""
        return (self._pwm_min() + self._pwm_max()) // 2

    # ==================== 参数校验回调 ====================
    def _on_param_set(self, params):
        for p in params:
            if p.name in ('pwm_min', 'pwm_max'):
                value = int(p.value)
                if not (ELECTRIC_MIN <= value <= ELECTRIC_MAX):
                    return SetParametersResult(
                        successful=False,
                        reason='%s=%d out of electrical range %d~%d'
                               % (p.name, value, ELECTRIC_MIN, ELECTRIC_MAX))
                lo = value if p.name == 'pwm_min' else self._pwm_min()
                hi = value if p.name == 'pwm_max' else self._pwm_max()
                if lo >= hi:
                    return SetParametersResult(
                        successful=False,
                        reason='pwm_min(%d) must be less than pwm_max(%d)' % (lo, hi))
            elif p.name == 'baudrate':
                if int(p.value) not in BAUD_CHOICES:
                    return SetParametersResult(
                        successful=False,
                        reason='unsupported baudrate %d, choose from %s' % (int(p.value), BAUD_CHOICES))
            elif p.name == 'default_duration_ms':
                if not (1 <= int(p.value) <= MAX_DURATION_MS):
                    return SetParametersResult(
                        successful=False,
                        reason='default_duration_ms must be within 1~%d' % MAX_DURATION_MS)
            elif p.name == 'servo_ids':
                ids = [int(x) for x in p.value]
                if not ids:
                    return SetParametersResult(successful=False, reason='servo_ids must not be empty')
                # ID 0 是出厂默认值（任务 1、2 阶段用），协议里合法；
                # 允许它是为了能把单舵机阶段录的 bag 重新挂回去复现，不影响整机默认 [1..10]
                bad = [x for x in ids if not (0 <= x <= 254)]
                if bad:
                    return SetParametersResult(
                        successful=False, reason='servo_ids contains invalid id: %s' % bad)
        return SetParametersResult(successful=True)

    # ==================== 串口 ====================
    def _open_serial(self):
        if serial is None:
            self.get_logger().error('缺少 pyserial：sudo apt install python3-serial')
            self._ser = None
            return
        port = str(self._param('port'))
        baud = int(self._param('baudrate'))
        try:
            self._ser = serial.Serial(port, baud, timeout=REPLY_TIMEOUT_S)
            self.get_logger().info('串口已打开：%s @ %d' % (port, baud))
        except Exception as exc:  # noqa: BLE001
            self._ser = None
            self.get_logger().error('串口打不开 %s @ %d：%s（会自动限频重连）'
                                    % (port, baud, exc))

    def _try_reconnect(self):
        """串口掉线后限频重开；节点不崩，插回 USB 后下一次调用就能恢复。"""
        with self._reconnect_lock:
            if self._ser is not None:
                return True
            now = time.monotonic()
            if now - self._last_reconnect < RECONNECT_INTERVAL_S:
                return False
            self._last_reconnect = now
            if serial is None:
                return False
            port = str(self._param('port'))
            baud = int(self._param('baudrate'))
            self.get_logger().warn('串口掉线，尝试重连 %s @ %d' % (port, baud))
            try:
                self._ser = serial.Serial(port, baud, timeout=REPLY_TIMEOUT_S)
                self.get_logger().info('串口已恢复：%s @ %d' % (port, baud))
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('串口重连失败：%s（%.1fs 后再试）'
                                      % (exc, RECONNECT_INTERVAL_S))
                return False

    def _transact(self, text):
        """发一条协议指令并读回包。Action 和 /servo/command 都走这里。"""
        with self._lock:
            if self._ser is None and not self._try_reconnect():
                raise ServoSerialError('串口未打开，等待重连')
            payload = (text + '\r\n').encode('ascii')
            try:
                self._ser.reset_input_buffer()
                self._ser.write(payload)
                self._ser.flush()
                raw = self._ser.read_until(b'!', size=64)
            except Exception as exc:  # noqa: BLE001
                # 拔 USB 时这里抛异常：标记掉线并转成可控错误，节点不崩，
                # 下一次 _transact 会走 _try_reconnect 自动接回来
                self._ser = None
                self.get_logger().warn('串口写失败，已标记掉线：%s' % exc)
                raise ServoSerialError('串口写失败：%s' % exc) from exc
        return raw.decode('utf-8', 'replace').strip()

    # --- 协议拼装：统一走 servo_protocol，pytest 不接硬件就能测这些函数 ---
    def _cmd_id(self, sid):
        return build_query(sid, CMD_ID)

    def _cmd_pos(self, sid):
        return build_query(sid, CMD_POS)

    def _cmd_torque(self, sid, on=True):
        return build_query(sid, CMD_TORQUE_ON if on else CMD_TORQUE_OFF)

    def _cmd_move(self, sid, pwm, duration_ms):
        return build_move(sid, pwm, duration_ms)

    @staticmethod
    def _parse_pwm(reply):
        return parse_pwm(reply)

    # ==================== 唯一底层发送函数（Action 与话题共用）====================
    def send_command(self, sid, pwm, duration_ms):
        self._transact(self._cmd_torque(sid, True))       # 先恢复扭力，否则舵机不动
        reply = self._transact(self._cmd_move(sid, pwm, duration_ms))
        self._last_pwm[int(sid)] = int(pwm)
        self._publish_state(int(sid), int(pwm))
        return reply

    def read_position(self, sid):
        reply = self._transact(self._cmd_pos(sid))
        return self._parse_pwm(reply)

    # ==================== 话题 ====================
    def _on_command(self, msg):
        """bag 回放通道：必须和 Action 走同一个 send_command。"""
        sid, pwm = int(msg.id), int(msg.pwm)
        if not (ELECTRIC_MIN <= pwm <= ELECTRIC_MAX):
            self.get_logger().warn('/servo/command 丢弃：pwm=%d 超出电气范围' % pwm)
            return
        duration = int(msg.duration_ms) or self._default_duration()
        try:
            self.send_command(sid, pwm, duration)
            self.get_logger().info('/servo/command 执行 id=%d pwm=%d dur=%d'
                                   % (sid, pwm, duration))
        except ServoSerialError as exc:
            self.get_logger().error('/servo/command 失败：%s' % exc)

    def _publish_state(self, sid, pwm):
        msg = ServoCommand()
        msg.id = int(sid)
        msg.pwm = int(pwm)
        msg.duration_ms = 0
        self._state_pub.publish(msg)

    def _publish_all_state(self):
        for sid, pwm in list(self._last_pwm.items()):
            self._publish_state(sid, pwm)

    # ==================== Action 四个回调 ====================
    def _on_goal(self, goal_request):
        sid = int(goal_request.id)
        pwm = int(goal_request.pwm)
        duration = int(goal_request.duration_ms)

        ids = self._servo_ids()
        if sid not in ids:
            self.get_logger().warn('拒绝 goal：ID %d 不在 servo_ids %s' % (sid, ids))
            return GoalResponse.REJECT
        lo, hi = self._pwm_min(), self._pwm_max()
        if not (lo <= pwm <= hi):
            self.get_logger().warn('拒绝 goal：pwm=%d 超出安全范围 [%d,%d]' % (pwm, lo, hi))
            return GoalResponse.REJECT
        if duration < 0 or duration > MAX_DURATION_MS:
            self.get_logger().warn('拒绝 goal：duration_ms=%d 非法' % duration)
            return GoalResponse.REJECT
        if self._ser is None and not self._try_reconnect():
            self.get_logger().error('拒绝 goal：串口没打开（port=%s），等待重连'
                                    % self._param('port'))
            return GoalResponse.REJECT
        self.get_logger().info('接受 goal：id=%d pwm=%d duration=%d'
                               % (sid, pwm, duration or self._default_duration()))
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle):
        # 交给 executor 的线程去跑，别堵住 goal 处理线程（否则 cancel 和 Ctrl+C 会失灵）
        goal_handle.execute()

    def _on_cancel(self, goal_handle):
        self.get_logger().info('收到取消请求，接受')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        req = goal_handle.request
        sid = int(req.id)
        target = int(req.pwm)
        duration = int(req.duration_ms) or self._default_duration()

        result = MoveServo.Result()
        feedback = MoveServo.Feedback()

        start = int(self._last_pwm.get(sid, self._neutral()))
        neutral = self._neutral()

        try:
            self._transact(self._cmd_torque(sid, True))
            self._transact(self._cmd_move(sid, target, duration))
        except ServoSerialError as exc:
            result.success = False
            result.final_pwm = start
            result.message = 'serial error: %s' % exc
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        steps = max(5, int(duration / 1000.0 * FEEDBACK_HZ))
        step_s = duration / 1000.0 / steps

        for i in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                try:
                    self._transact(self._cmd_move(sid, neutral, self._default_duration()))
                except ServoSerialError:
                    pass
                self._last_pwm[sid] = neutral
                self._publish_state(sid, neutral)
                goal_handle.canceled()
                result.success = False
                result.final_pwm = neutral
                result.message = 'canceled'
                self.get_logger().warn('goal 被取消，已回中位 %d' % neutral)
                return result

            frac = float(i) / steps
            feedback.current_pwm = int(start + (target - start) * frac)
            feedback.progress = frac
            goal_handle.publish_feedback(feedback)
            self._publish_state(sid, feedback.current_pwm)
            time.sleep(step_s)

        # 走完读回校验：舵机是「用 T 毫秒平滑转过去」，刚好到点就读会读到上一个位置，
        # 所以先等一小会儿；还没到位就自动重发一次指令，再读回来确认。
        # 一次到位时不打任何额外日志，正常演示的终端输出和录屏保持一致。
        time.sleep(0.35)
        actual = None
        resent = False
        for attempt in range(3):
            try:
                actual = self.read_position(sid)
            except ServoSerialError as exc:
                result.success = False
                result.final_pwm = target
                result.message = 'PRAD read failed: %s' % exc
                goal_handle.abort()
                return result
            if actual is not None and abs(actual - target) <= 30:
                if resent:
                    self.get_logger().info('重发后到位：PRAD %d，目标 %d' % (actual, target))
                break
            self.get_logger().warn('未到位（PRAD %s，目标 %d）' % (actual, target))
            if not resent:
                resent = True
                resend_ms = min(self._default_duration(), 500)
                frame = self._cmd_move(sid, target, resend_ms)
                self.get_logger().warn('未到位 → 重发 %s' % frame)
                try:
                    self._transact(self._cmd_torque(sid, True))
                    self._transact(frame)
                except ServoSerialError as exc:
                    result.success = False
                    result.final_pwm = target
                    result.message = 'PRAD resend failed: %s' % exc
                    goal_handle.abort()
                    return result
                time.sleep(resend_ms / 1000.0)
            else:
                self.get_logger().info('PRAD 再读一次（%d/3）' % (attempt + 1))
                time.sleep(0.25)

        if actual is None:
            result.success = False
            result.final_pwm = target
            result.message = 'PRAD no reply, cannot confirm position'
            goal_handle.abort()
            return result

        diff = actual - target
        self._last_pwm[sid] = actual
        self._publish_state(sid, actual)
        if abs(diff) > 30:
            result.success = False
            result.final_pwm = actual
            result.message = 'NOT REACHED: target %d, PRAD %d, diff %d' % (target, actual, diff)
            self.get_logger().warn(result.message)
            goal_handle.abort()
            return result

        result.success = True
        result.final_pwm = actual
        result.message = 'OK: target %d, read %d, diff %d' % (target, actual, diff)
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    # ==================== 启动/退出 ====================
    def _startup_reset(self):
        for sid in self._servo_ids():
            try:
                self._transact(self._cmd_torque(sid, True))
                self._transact(self._cmd_move(sid, self._neutral(), self._default_duration()))
                self._last_pwm[sid] = self._neutral()
                self.get_logger().info('启动回中位：id=%d' % sid)
            except ServoSerialError as exc:
                self.get_logger().warn('启动回中位失败 id=%d：%s' % (sid, exc))
                break

    def destroy_node(self):
        # 安全下电：只关串口，不发 PULK。
        # 实测这台舵机收到 PULK 后会锁死，PULR 也叫不回来，只能断电重启，
        # 所以退出时不卸力（真要卸力用 parameter 里的 reset_on_shutdown 保持默认 false）。
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


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
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
