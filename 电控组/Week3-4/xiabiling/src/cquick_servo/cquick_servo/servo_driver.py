#!/usr/bin/env python3
"""CQuick 舵机驱动：Action Server + /servo/command 调试总线"""

import re
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
from cquick_servo_interfaces.msg import ServoCommand

# 协议常量（协议本身，不是配置）
START = '#'
END = '!'
C_TORQUE_ON = 'PULR'
C_TORQUE_OFF = 'PULK'
C_READ_PWM = 'PRAD'
C_MOVE = 'P'

# 电气绝对极限（不可配置，安全区由 pwm_min/pwm_max 决定）
PWM_ABS_MIN = 500
PWM_ABS_MAX = 2500

FEEDBACK_HZ = 10.0
PRAD_TOL = 30
PRAD_RETRIES = 3


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4])
        self.declare_parameter('startup_reset', True)
        self.declare_parameter('center_pwm', 1500)
        self.declare_parameter(
            'allowed_baudrates',
            [9600, 19200, 38400, 57600, 128000, 256000])

        self._lock = threading.RLock()
        self._ser = None
        self._cur_port = None
        self._cur_baud = None
        self._last = {}
        self._ignore_topic_until = 0.0

        self._cb = ReentrantCallbackGroup()
        self.add_on_set_parameters_callback(self._on_set_params)

        self._state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        self._cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self.create_subscription(ServoCommand, '/servo/command',
                                 self._on_debug_cmd, 10,
                                 callback_group=self._cb)

        self._srv = ActionServer(
            self, MoveServo, 'move_servo',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            handle_accepted_callback=self._accepted_cb,
            callback_group=self._cb)

        self.create_timer(1.0, self._heartbeat)

        if bool(self.p('startup_reset')):
            self._reset_timer = self.create_timer(0.8, self._reset_once)

        self.get_logger().info(
            'servo_driver up: port=%s baud=%d pwm=[%d,%d] ids=%s center=%d'
            % (self.p('port'), self.p('baudrate'), self.p('pwm_min'),
               self.p('pwm_max'), list(self.p('servo_ids')), self.p('center_pwm')))

    # ---------- 参数：永远实时读，禁止缓存 ----------
    def p(self, name):
        return self.get_parameter(name).value

    def _on_set_params(self, params):
        pend = {}
        for x in params:
            n, v = x.name, x.value
            if n in ('pwm_min', 'pwm_max', 'center_pwm'):
                if isinstance(v, bool) or not isinstance(v, int):
                    return SetParametersResult(successful=False, reason='%s 必须是整数' % n)
                if not (PWM_ABS_MIN <= v <= PWM_ABS_MAX):
                    return SetParametersResult(
                        successful=False,
                        reason='%s=%d 超出电气范围 %d~%d' % (n, v, PWM_ABS_MIN, PWM_ABS_MAX))
                pend[n] = v
            elif n == 'baudrate':
                allowed = [int(x) for x in self.p('allowed_baudrates')]
                if v not in allowed:
                    return SetParametersResult(
                        successful=False,
                        reason='不支持的波特率 %s，允许 %s' % (v, allowed))
            elif n == 'default_duration_ms':
                if not (1 <= v <= 10000):
                    return SetParametersResult(
                        successful=False, reason='default_duration_ms 必须在 1~10000')
            elif n == 'servo_ids':
                ids = list(v)
                if not ids:
                    return SetParametersResult(successful=False, reason='servo_ids 不能为空')
                for i in ids:
                    if isinstance(i, bool) or not isinstance(i, int) or not (1 <= i <= 254):
                        return SetParametersResult(
                            successful=False, reason='ID 必须是 1~254 的整数: %s' % ids)
                if len(set(ids)) != len(ids):
                    return SetParametersResult(
                        successful=False, reason='servo_ids 有重复: %s' % ids)
            elif n == 'port':
                if not isinstance(v, str) or not v:
                    return SetParametersResult(successful=False, reason='port 不能为空')

        pmin = pend.get('pwm_min', int(self.p('pwm_min')))
        pmax = pend.get('pwm_max', int(self.p('pwm_max')))
        if pmin >= pmax:
            return SetParametersResult(
                successful=False,
                reason='pwm_min(%d) 必须小于 pwm_max(%d)' % (pmin, pmax))
        return SetParametersResult(successful=True)

    # ---------- 串口 ----------
    def _serial(self):
        port, baud = str(self.p('port')), int(self.p('baudrate'))
        with self._lock:
            if (self._ser is not None and self._ser.is_open
                    and self._cur_port == port and self._cur_baud == baud):
                return self._ser
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
            self._ser = serial.Serial(port, baud, bytesize=8, parity='N',
                                      stopbits=1, timeout=0.3)
            self._cur_port, self._cur_baud = port, baud
            self.get_logger().info('串口打开 %s @ %d 8-N-1' % (port, baud))
            return self._ser

    def _send(self, cmd):
        with self._lock:
            ser = self._serial()
            ser.write((cmd + '\r\n').encode('ascii'))
            ser.flush()

    def _transact(self, cmd, wait=0.8, retries=PRAD_RETRIES):
        for _ in range(retries):
            with self._lock:
                ser = self._serial()
                ser.reset_input_buffer()
                ser.write((cmd + '\r\n').encode('ascii'))
                ser.flush()
                t0, buf = time.monotonic(), b''
                while time.monotonic() - t0 < wait:
                    n = ser.in_waiting
                    if n:
                        buf += ser.read(n)
                        if END.encode() in buf:
                            break
                    else:
                        time.sleep(0.01)
            if buf.strip():
                return buf.strip()
            time.sleep(0.2)
        return b''

    @staticmethod
    def _addr(sid):
        return '%s%03d' % (START, int(sid))

    def _cmd(self, sid, kind, value=None, dur=None):
        if kind == C_MOVE:
            return '%s%s%dT%d%s' % (self._addr(sid), C_MOVE, int(value), int(dur), END)
        return '%s%s%s' % (self._addr(sid), kind, END)

    def _center(self):
        c = int(self.p('center_pwm'))
        return max(int(self.p('pwm_min')), min(int(self.p('pwm_max')), c))

    def _read_pwm(self, sid):
        raw = self._transact(self._cmd(sid, C_READ_PWM))
        m = re.search(rb'P(\d{3,4})', raw or b'')
        return int(m.group(1)) if m else None

    # ---------- 校验 ----------
    def _validate(self, sid, pwm):
        ids = [int(i) for i in self.p('servo_ids')]
        pmin, pmax = int(self.p('pwm_min')), int(self.p('pwm_max'))
        if sid not in ids:
            self.get_logger().warn('拒绝: id=%d 不在 servo_ids=%s' % (sid, ids))
            return False
        if not (pmin <= pwm <= pmax):
            self.get_logger().warn('拒绝: pwm=%d 超出安全区间 [%d,%d]' % (pwm, pmin, pmax))
            return False
        return True

    # ---------- Action 回调 ----------
    def _goal_cb(self, req):
        if not self._validate(int(req.id), int(req.pwm)):
            return GoalResponse.REJECT
        if int(req.duration_ms) > 10000:
            self.get_logger().warn('拒绝: duration_ms=%d 超过 10000' % req.duration_ms)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn('收到取消请求，接受')
        return CancelResponse.ACCEPT

    def _accepted_cb(self, goal_handle):
        goal_handle.execute()

    def _execute(self, gh):
        sid = int(gh.request.id)
        target = int(gh.request.pwm)
        dur = int(gh.request.duration_ms) or int(self.p('default_duration_ms'))
        res = MoveServo.Result()
        self.get_logger().info('[goal] id=%d pwm=%d dur=%dms' % (sid, target, dur))
        # 镜像到调试总线，让 bag 能录到、回放时能重演
        self._cmd_pub.publish(self._make_cmd(sid, target, dur))
        self._ignore_topic_until = time.monotonic() + dur / 1000.0 + 1.0
        try:
            self._send(self._cmd(sid, C_TORQUE_ON))
            time.sleep(0.30)
            start = self._read_pwm(sid)
            if start is None:
                start = target
            self._send(self._cmd(sid, C_MOVE, target, dur))

            fb = MoveServo.Feedback()
            t0 = time.monotonic()
            total = max(dur / 1000.0, 0.001)
            period = 1.0 / FEEDBACK_HZ
            while True:
                if gh.is_cancel_requested:
                    self.get_logger().warn('[cancel] id=%d 停发、卸力、回中位' % sid)
                    self._send(self._cmd(sid, C_TORQUE_OFF))
                    time.sleep(0.1)
                    self._send(self._cmd(sid, C_TORQUE_ON))
                    time.sleep(0.1)
                    self._send(self._cmd(sid, C_MOVE, self._center(), 600))
                    gh.canceled()
                    res.success = False
                    res.final_pwm = 0
                    res.message = 'canceled'
                    return res

                prog = min(1.0, (time.monotonic() - t0) / total)
                fb.current_pwm = int(round(start + (target - start) * prog))
                fb.progress = float(prog)
                gh.publish_feedback(fb)
                if prog >= 1.0:
                    break
                time.sleep(period)

            final = self._read_pwm(sid)
            if final is None or abs(final - target) > PRAD_TOL:
                self.get_logger().warn(
                    '[PRAD] id=%d 目标%d 读回%s → 重发一次' % (sid, target, final))
                self._send(self._cmd(sid, C_TORQUE_ON))
                time.sleep(0.30)
                self._send(self._cmd(sid, C_MOVE, target, max(600, dur)))
                time.sleep(max(600, dur) / 1000.0)
                final = self._read_pwm(sid)

            if final is None:
                res.success = False
                res.final_pwm = 0
                res.message = 'PRAD 读回失败（串口无响应）'
                gh.abort()
                return res
            res.final_pwm = int(final)
            if abs(final - target) > PRAD_TOL:
                res.success = False
                res.message = '未到位: 目标%d 实际%d 差%d>±%d' % (
                    target, final, abs(final - target), PRAD_TOL)
                gh.abort()
                return res

            res.success = True
            res.message = '到位: 目标%d 实际%d' % (target, final)
            gh.succeed()
            self._last[sid] = int(final)
            self.get_logger().info('[result] id=%d success=True final=%d' % (sid, final))
            return res
        except Exception as exc:
            self.get_logger().error('[goal] 异常 %s' % exc)
            res.success = False
            res.final_pwm = 0
            res.message = '串口错误: %s' % exc
            gh.abort()
            return res

    # ---------- 调试总线：与 Action 同一个 _send ----------
    def _on_debug_cmd(self, msg):
        sid, pwm = int(msg.id), int(msg.pwm)
        dur = int(msg.duration_ms) or int(self.p('default_duration_ms'))
        if time.monotonic() < self._ignore_topic_until:
            self.get_logger().debug('/servo/command 回环忽略（本节点 Action 刚发过）')
            return
        if not self._validate(sid, pwm):
            return
        try:
            self.get_logger().info('[/servo/command] id=%d pwm=%d dur=%d' % (sid, pwm, dur))
            self._send(self._cmd(sid, C_TORQUE_ON))
            time.sleep(0.30)
            self._send(self._cmd(sid, C_MOVE, pwm, dur))
            self._last[sid] = pwm
        except Exception as exc:
            self.get_logger().error('/servo/command 失败: %s' % exc)

    def _make_cmd(self, sid, pwm, dur):
        m = ServoCommand()
        m.id = int(sid)
        m.pwm = int(pwm)
        m.duration_ms = int(dur)
        return m

    def _heartbeat(self):
        for sid, pwm in list(self._last.items()):
            m = ServoCommand()
            m.id = sid
            m.pwm = pwm
            m.duration_ms = 0
            self._state_pub.publish(m)

    def _reset_once(self):
        self._reset_timer.cancel()
        ids = [int(i) for i in self.p('servo_ids')]
        c = self._center()
        self.get_logger().info('启动回中位: %s -> %d' % (ids, c))
        for sid in ids:
            try:
                self._send(self._cmd(sid, C_TORQUE_ON))
                time.sleep(0.30)
                self._send(self._cmd(sid, C_MOVE, c, int(self.p('default_duration_ms'))))
            except Exception as exc:
                self.get_logger().error('回中位失败 id=%d: %s' % (sid, exc))
                return


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
