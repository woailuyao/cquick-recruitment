#!/usr/bin/env python3
"""舵机串口驱动：Action 服务端 + 全参数化 + 调试话题 + 掉线重连 + PRAD 重发 + 看门狗"""

import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

try:
    import serial
except ImportError:
    serial = None

# ===== 协议常量（协议本身规定，允许写死）=====
PROTO_START = '#'
PROTO_END = '!'
CMD_MOVE = 'P'
CMD_TIME = 'T'
CMD_READ_ID = 'PID'
CMD_READ_POS = 'PRAD'
CMD_TORQUE_ON = 'PULR'
CMD_TORQUE_OFF = 'PULK'


def build_move_cmd(servo_id, pwm, duration_ms):
    return (PROTO_START + '%03d' % servo_id + CMD_MOVE + '%04d' % pwm
            + CMD_TIME + '%04d' % duration_ms + PROTO_END).encode('ascii')


def build_simple_cmd(servo_id, code):
    return (PROTO_START + '%03d' % servo_id + code + PROTO_END).encode('ascii')


def parse_pwm(reply):
    if not reply:
        return None
    m = re.search(rb'(\d{4})', reply)
    return int(m.group(1)) if m else None


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # ===== 参数（配置只在这里出现一次）=====
        self.declare_parameter('port', '/dev/cq_servo')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('allowed_baudrates', [9600, 19200, 38400, 57600, 115200, 230400])
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('home_pwm', 1500)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('prad_tolerance', 30)
        self.declare_parameter('prad_retry', True)
        self.declare_parameter('read_timeout_ms', 250)
        self.declare_parameter('feedback_period_ms', 200)
        self.declare_parameter('state_period_ms', 200)
        self.declare_parameter('cmd_timeout_ms', 0)
        self.declare_parameter('watchdog_enable', False)
        self.declare_parameter('watchdog_timeout_ms', 1000)
        self.declare_parameter('watchdog_reset', False)
        self.declare_parameter('servo_ids', [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.declare_parameter('startup_reset', True)

        self.add_on_set_parameters_callback(self._on_param_set)

        self.ser = None
        self._io_lock = threading.RLock()   # 串口 IO 互斥锁（并行 goal 时防回包交错）
        self._last_sent = None
        self._last_reopen_try = 0.0
        self._last_cmd_time = time.monotonic()
        self._last_heartbeat = time.monotonic()
        self._tripped = False
        self._open_serial()
        if self.p('startup_reset'):
            self._startup_reset()

        self._cg = ReentrantCallbackGroup()

        # ===== 调试话题（bag 录制/回放）=====
        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self.state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        self.create_subscription(ServoCommand, '/servo/heartbeat',
                                 self._on_heartbeat, 10, callback_group=self._cg)
        self.create_subscription(ServoCommand, '/servo/command',
                                 self._on_command_topic, 10, callback_group=self._cg)
        self.create_timer(self.p('state_period_ms') / 1000.0,
                          self._publish_state, callback_group=self._cg)
        # ===== 安全看门狗 =====
        self.create_timer(0.5, self._watchdog, callback_group=self._cg)

        self._action_server = ActionServer(
            self, MoveServo, 'move_servo',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            handle_accepted_callback=self.handle_accepted_cb,
            callback_group=self._cg)

        self.get_logger().info(
            'servo_driver 就绪 port=%s pwm=[%s, %s] ids=%s timeout=%s'
            % (self.p('port'), self.p('pwm_min'), self.p('pwm_max'),
               self.p('servo_ids'), self.p('cmd_timeout_ms')))

    # ---------- 参数 ----------
    def p(self, name):
        return self.get_parameter(name).value

    def _on_param_set(self, params):
        keys = ('pwm_min', 'pwm_max', 'home_pwm', 'baudrate', 'prad_tolerance',
                'read_timeout_ms', 'feedback_period_ms', 'state_period_ms',
                'cmd_timeout_ms', 'watchdog_timeout_ms')
        cur = {k: self.p(k) for k in keys}
        for prm in params:
            if prm.name in cur:
                cur[prm.name] = prm.value

        for prm in params:
            if prm.name in ('pwm_min', 'pwm_max', 'home_pwm'):
                if not (500 <= prm.value <= 2500):
                    return SetParametersResult(successful=False,
                                               reason='%s=%s 超出电气范围 500~2500'
                                               % (prm.name, prm.value))
            if prm.name == 'baudrate' and prm.value not in self.p('allowed_baudrates'):
                return SetParametersResult(successful=False, reason='不支持的波特率 %s' % prm.value)
            if prm.name in ('prad_tolerance', 'read_timeout_ms', 'feedback_period_ms',
                            'state_period_ms', 'cmd_timeout_ms',
                            'watchdog_timeout_ms'):
                if prm.value < 0 or (prm.value == 0 and prm.name != 'cmd_timeout_ms'):
                    return SetParametersResult(successful=False, reason='%s 必须大于 0' % prm.name)
            if prm.name == 'servo_ids':
                if not prm.value or any(not (0 <= i <= 254) for i in prm.value):
                    return SetParametersResult(successful=False, reason='servo_ids 必须是 0~254 的非空列表')

        if cur['pwm_min'] >= cur['pwm_max']:
            return SetParametersResult(
                successful=False,
                reason='pwm_min(%s) 必须小于 pwm_max(%s)' % (cur['pwm_min'], cur['pwm_max']))
        return SetParametersResult(successful=True)

    # ---------- 串口 ----------
    def _open_serial(self):
        if serial is None:
            self.get_logger().error('python3-serial 未安装')
            return
        try:
            self.ser = serial.Serial(self.p('port'), self.p('baudrate'), timeout=0.3)
            self.get_logger().info('串口已打开 %s @ %s' % (self.p('port'), self.p('baudrate')))
        except Exception as e:
            self.ser = None
            self.get_logger().warn('串口打开失败（无硬件时可忽略）：%s' % e)

    def _try_reopen(self):
        now = time.monotonic()
        if now - self._last_reopen_try < 2.0:
            return False
        self._last_reopen_try = now
        self.get_logger().warn('尝试重新打开串口…')
        self._open_serial()
        if self.ser is not None:
            self.get_logger().info('串口已恢复')
            return True
        return False

    def _send(self, payload):
        with self._io_lock:
            return self._send_locked(payload)

    def _send_locked(self, payload):
        if self.ser is None:
            self._try_reopen()
        if self.ser is None:
            self.get_logger().warn('串口未打开，丢弃 %r' % payload)
            return False
        try:
            self.ser.write(payload)
            self.ser.flush()
            self._last_sent = (payload, time.monotonic())
            self._last_cmd_time = time.monotonic()
            return True
        except Exception as e:
            self.get_logger().warn('串口异常，标记断开并准备重连：%s' % e)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return False

    def _clear_input(self):
        if self.ser is None:
            return
        try:
            self.ser.reset_input_buffer()
        except Exception:
            self.ser = None

    def _read_frame(self):
        """读到以 ! 结尾的一帧或超时返回（不预先清缓冲！）"""
        timeout = int(self.p('read_timeout_ms')) / 1000.0
        if self.ser is None:
            return b''
        t0 = time.monotonic()
        buf = b''
        while time.monotonic() - t0 < timeout:
            try:
                n = self.ser.in_waiting
            except Exception:
                self.ser = None
                return buf
            if n:
                try:
                    buf += self.ser.read(n)
                except Exception:
                    self.ser = None
                    return buf
                if buf.endswith(b'!'):
                    break
            else:
                time.sleep(0.005)
        return buf

    def _read_pwm(self, servo_id):
        with self._io_lock:
            return self._read_pwm_locked(servo_id)

    def _read_pwm_locked(self, servo_id):
        """先清旧数据 -> 发 PRAD -> 等完整回包 -> 解析"""
        if self.ser is None:
            return None
        self._clear_input()
        if not self._send(build_simple_cmd(servo_id, CMD_READ_POS)):
            return None
        reply = self._read_frame()
        return parse_pwm(reply)

    def _startup_reset(self):
        home = int(self.p('home_pwm'))
        dur = int(self.p('default_duration_ms'))
        for sid in self.p('servo_ids'):
            self._send(build_simple_cmd(sid, CMD_TORQUE_ON))
            self._send(build_move_cmd(sid, home, dur))
        self.get_logger().info(
            'startup_reset：已发送 PULR，并全部回中位 %d（%d ms）' % (home, dur))

    def unload_all(self):
        for sid in self.p('servo_ids'):
            self._send(build_simple_cmd(sid, CMD_TORQUE_OFF))

    # ---------- 心跳 & 看门狗 ----------
    def _publish_state(self):
        msg = ServoCommand()
        msg.id = 0
        msg.pwm = 0
        msg.duration_ms = 0
        self.state_pub.publish(msg)

    def _on_heartbeat(self, msg):
        """外部（上位机/动作组）心跳，收到即刷新"""
        self._last_heartbeat = time.monotonic()

    def _watchdog(self):
        """心跳看门狗：超时无 /servo/heartbeat -> 全部 PULK 并锁死，需外部确认恢复"""
        if not bool(self.p('watchdog_enable')):
            # 旧版「指令超时」逻辑，保留兼容
            timeout = int(self.p('cmd_timeout_ms'))
            if timeout <= 0:
                return
            if time.monotonic() - self._last_cmd_time > timeout / 1000.0:
                self.get_logger().warn(
                    '看门狗触发：%d ms 无新指令，全部 PULK 卸力' % timeout)
                self.unload_all()
                self._last_cmd_time = time.monotonic()
            return

        # 已触发：只有外部确认（watchdog_reset=true）才允许恢复
        if self._tripped:
            if bool(self.p('watchdog_reset')):
                self._tripped = False
                self._last_heartbeat = time.monotonic()
                self.get_logger().warn('看门狗已复位（外部确认），恢复接受指令')
            return

        lim = int(self.p('watchdog_timeout_ms')) / 1000.0
        if time.monotonic() - self._last_heartbeat > lim:
            self.get_logger().error(
                '看门狗触发：%.1f 秒未收到 /servo/heartbeat 心跳，全部舵机 PULK 卸力；'
                '此后拒绝新指令，需 ros2 param set /servo_driver watchdog_reset true 恢复'
                % lim)
            self.unload_all()
            self._tripped = True

    # ---------- 调试话题（bag 回放走这里）----------
    def _on_command_topic(self, msg):
        payload = build_move_cmd(int(msg.id), int(msg.pwm), int(msg.duration_ms))
        if self._last_sent is not None:
            last_payload, last_t = self._last_sent
            if payload == last_payload and time.monotonic() - last_t < 0.2:
                return
        self.get_logger().info('[topic] 回放指令 %r' % payload)
        self._send(payload)

    # ---------- Action 回调 ----------
    def goal_cb(self, goal_request):
        if self._tripped:
            self.get_logger().error(
                '看门狗已触发（舵机已卸力），拒绝新 goal；需外部确认恢复')
            return GoalResponse.REJECT
        sid = int(goal_request.id)
        pwm = int(goal_request.pwm)
        dur = int(goal_request.duration_ms)

        if sid not in self.p('servo_ids'):
            why = 'ID %d 不在 servo_ids 里' % sid
        elif not (self.p('pwm_min') <= pwm <= self.p('pwm_max')):
            why = 'pwm %d 超出安全范围 [%s, %s]' % (pwm, self.p('pwm_min'), self.p('pwm_max'))
        elif dur <= 0:
            why = 'duration_ms %d 非法' % dur
        else:
            self.get_logger().info('接受 goal id=%d pwm=%d dur=%d' % (sid, pwm, dur))
            return GoalResponse.ACCEPT
        self.get_logger().warn('拒绝 goal：%s' % why)
        return GoalResponse.REJECT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn('收到取消请求')
        return CancelResponse.ACCEPT

    def handle_accepted_cb(self, goal_handle):
        goal_handle.execute()

    def execute_cb(self, goal_handle):
        req = goal_handle.request
        sid = int(req.id)
        target = int(req.pwm)
        dur = int(req.duration_ms)
        result = MoveServo.Result()

        self._send(build_simple_cmd(sid, CMD_TORQUE_ON))

        start_pwm = self._read_pwm(sid)
        if start_pwm is None:
            start_pwm = target

        payload = build_move_cmd(sid, target, dur)
        if not self._send(payload):
            result.success = False
            result.final_pwm = 0
            result.message = 'serial_error'
            goal_handle.abort()
            return result

        # 发成功后再发布话题，避免自发自收重复执行
        cmd = ServoCommand()
        cmd.id = sid
        cmd.pwm = target
        cmd.duration_ms = dur
        self.cmd_pub.publish(cmd)

        period_ms = int(self.p('feedback_period_ms'))
        n = max(int(dur / period_ms), 2)
        for i in range(n):
            if goal_handle.is_cancel_requested:
                self._send(build_move_cmd(sid, int(self.p('home_pwm')), period_ms))
                goal_handle.canceled()
                result.success = False
                got = self._read_pwm(sid)
                result.final_pwm = int(got) if got is not None else 0
                result.message = 'canceled'
                self.get_logger().warn('goal 已取消，回中位 %s' % self.p('home_pwm'))
                return result

            progress = (i + 1) / n
            fb = MoveServo.Feedback()
            fb.progress = float(progress)
            fb.current_pwm = int(start_pwm + (target - start_pwm) * progress)
            goal_handle.publish_feedback(fb)

            # 动作执行期间刷新时间戳，避免看门狗误触发
            self._last_cmd_time = time.monotonic()

            t_end = time.monotonic() + period_ms / 1000.0
            while time.monotonic() < t_end:
                time.sleep(0.005)

        tol = int(self.p('prad_tolerance'))
        final = self._read_pwm(sid)

        if self.p('prad_retry') and (final is None or abs(final - target) > tol):
            self.get_logger().warn('未到位（读回 %s vs 目标 %d），重发一次' % (final, target))
            self._send(build_move_cmd(sid, target, max(period_ms, 200)))
            time.sleep(max(period_ms, 200) / 1000.0)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                got = self._read_pwm(sid)
                result.final_pwm = int(got) if got is not None else 0
                result.message = 'canceled'
                return result
            final = self._read_pwm(sid)
            self.get_logger().info('重发后读回 %s' % final)

        if final is None:
            result.success = False
            result.final_pwm = 0
            result.message = 'prad_no_reply: 读不到实际位置'
        else:
            diff = abs(final - target)
            result.final_pwm = int(final)
            if diff > tol:
                result.success = False
                result.message = 'not_reached: 差 %d（目标 %d 实际 %d）' % (diff, target, final)
            else:
                result.success = True
                result.message = 'ok'

        goal_handle.succeed()
        self.get_logger().info('完成 id=%d result=%s' % (sid, result.message))
        return result


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
        node.get_logger().info('退出前卸力 PULK')
        node.unload_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()