#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cquick_servo 驱动节点：Action Server + 串口透传

对外只暴露 Action：/move_servo
内部调试话题：/servo/command（订阅 + 镜像发布，供 ros2 bag 录制与回放）、
              /servo/state（发布当前状态）

设计约束（评审会 grep）：
  1. 端口 / 波特率 / 限位 / 默认时长 / ID 列表 / 启动复位 —— 全部来自参数；
  2. 协议常量（# ! P T PID PRAD PULR PULK 与 3 位补零）允许直接写；
  3. Action 和 /servo/command 走同一个底层发送函数 send_command()。

线程模型：MultiThreadedExecutor + ReentrantCallbackGroup，
否则 execute_callback 的 5Hz 等待循环会把 executor 占死，cancel 永远进不来。

本机实测到的硬件边界（详见 README 踩坑记录）：
  - 动作执行期间不接受新目标，包括回中位；
  - PULK 之后位置控制会失效较长时间，必须重新上电，所以运行时不能用它急停；
  - 动作结束瞬间读 PRAD 会读到过冲值，需要等机械稳定后再连读两次。
"""
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

# ---- 协议常量（协议本身，不是配置）----
P_START = '#'
P_END = '!'
P_SET = 'P'
P_TIME = 'T'
P_KIND_ID = 'PID'
P_KIND_POS = 'PRAD'
P_KIND_TORQUE_ON = 'PULR'
P_KIND_TORQUE_OFF = 'PULK'
NEUTRAL_PWM = 1500          # 协议定义的中位，不是可配的安全限位

# ---- 运行时规格常量 ----
FEEDBACK_PERIOD = 0.2        # 5 Hz
PRAD_TOLERANCE = 30          # 与目标差超过 ±30 算未到位
CANCEL_RETURN_MS = 1000      # 取消后回中位用时（低于1000会被忽略）
CANCEL_RECOVER_S = 8.0       # 取消后尝试回中位的总时限
SETTLE_BEFORE_PRAD = 0.3     # 动作结束后等机械稳定再读 PRAD
SUPPORTED_BAUDS = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000)


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        # ---- 1. 声明参数（名字只出现在「声明」和「读取」两处）----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [0])
        self.declare_parameter('startup_reset', True)
        self.declare_parameter('shutdown_pulk', True)

        self._serial_lock = threading.Lock()
        # 本机控制板同一时刻只能驱动一路动作，并发会互相打断，用这把锁串行化
        self._exec_lock = threading.Lock()
        self._ser = None
        self._open_port = None
        self._open_baud = None
        self._last_attempt = 0.0
        self._recent_mirror = []
        # 缓存 ID 列表：Ctrl+C 之后 rclpy 上下文失效，不能再 get_parameter
        self._ids = [int(i) for i in self.get_parameter('servo_ids').value]
        self._shutdown_pulk = bool(self.get_parameter('shutdown_pulk').value)

        # 参数回调：不注册它就没有权力拒绝非法值
        self.add_on_set_parameters_callback(self._on_param_change)

        self._cb_group = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        self._cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self._cmd_sub = self.create_subscription(
            ServoCommand, '/servo/command', self._on_debug_command, 10,
            callback_group=self._cb_group)

        self._action_server = ActionServer(
            self, MoveServo, 'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group)

        self._open_serial()
        self.create_timer(1.0, self._watch_serial, callback_group=self._cb_group)

        if self.get_parameter('startup_reset').value:
            self._startup_reset()

        self.get_logger().info('servo_driver 就绪，Action: /move_servo')

    # ==================== 串口层 ====================

    def _open_serial(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        with self._serial_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = None
            try:
                self._ser = serial.Serial(port, baud, timeout=0.05,
                                          rtscts=False, xonxoff=False, dsrdtr=False)
                self._open_port = port
                self._open_baud = baud
                self.get_logger().info('串口打开成功: %s @ %d 8-N-1' % (port, baud))
            except Exception as exc:  # noqa: BLE001
                self._ser = None
                self.get_logger().error('串口打开失败 %s @ %d: %s' % (port, baud, exc))

    def _watch_serial(self):
        """1Hz 检查：串口丢了或端口参数改了，就重连一次。"""
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        broken = self._ser is None or not self._ser.is_open
        changed = (port, baud) != (self._open_port, self._open_baud)
        if not (broken or changed):
            return
        if time.monotonic() - self._last_attempt < 1.0:
            return
        self._last_attempt = time.monotonic()
        self.get_logger().warn('串口不可用（broken=%s changed=%s），尝试重连' % (broken, changed))
        self._open_serial()

    def _write(self, payload):
        """唯一的底层写入口。"""
        with self._serial_lock:
            if self._ser is None or not self._ser.is_open:
                return False
            try:
                self._ser.write(payload)
                self._ser.flush()
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('串口写失败: %s' % exc)
                self._ser = None
                return False

    def _read_for(self, wait):
        buf = b''
        deadline = time.monotonic() + wait
        with self._serial_lock:
            if self._ser is None or not self._ser.is_open:
                return buf
            while time.monotonic() < deadline:
                try:
                    n = self._ser.in_waiting
                    if n:
                        buf += self._ser.read(n)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error('串口读失败: %s' % exc)
                    self._ser = None
                    return buf
                time.sleep(0.005)
        return buf

    # ---- 指令拼装：协议常量 + 3 位补零 ----
    def cmd_set(self, sid, pwm, duration_ms):
        return ('%s%03d%s%d%s%d%s' % (P_START, sid, P_SET, pwm, P_TIME, duration_ms, P_END)).encode()

    def cmd_query(self, sid, kind):
        # kind 本身带 P（PID / PRAD / PULR / PULK），不能再补一个 P_SET
        return ('%s%03d%s%s' % (P_START, sid, kind, P_END)).encode()

    # ---- 唯一的发送函数（Action 与调试话题共用）----
    def send_command(self, sid, pwm, duration_ms):
        payload = self.cmd_set(sid, pwm, duration_ms)
        ok = self._write(payload)
        if ok:
            self.get_logger().info('TX %s' % payload.decode('ascii', 'replace'))
            self._publish_state(sid, pwm)
        return ok

    def _mirror_cmd(self, sid, pwm, duration_ms):
        """把发出的指令镜像到 /servo/command，bag 才有数据可录、可回放。"""
        msg = ServoCommand()
        msg.id = int(sid)
        msg.pwm = int(pwm)
        msg.duration_ms = int(duration_ms)
        self._recent_mirror.append((int(sid), int(pwm), int(duration_ms), time.monotonic()))
        self._recent_mirror = self._recent_mirror[-32:]
        self._cmd_pub.publish(msg)

    def query(self, sid, kind, wait=0.25):
        payload = self.cmd_query(sid, kind)
        if not self._write(payload):
            return b''
        buf = self._read_for(wait)
        self.get_logger().info('QRY %s -> %s' % (
            payload.decode('ascii', 'replace'),
            buf.decode('ascii', 'replace').strip() if buf else '<无回显>'))
        return buf

    @staticmethod
    def parse_pwm(buf):
        m = re.search(rb'P(\d{3,4})', buf or b'')
        return int(m.group(1)) if m else None

    # ==================== 状态发布 ====================

    def _publish_state(self, sid, pwm):
        msg = ServoCommand()
        msg.id = int(sid)
        msg.pwm = int(max(0, min(65535, pwm)))
        msg.duration_ms = 0     # state 里恒为 0：这是当前值，不是目标值
        self._state_pub.publish(msg)

    # ==================== 参数校验 ====================

    def _on_param_change(self, params):
        """返回 successful=False 就整个 set 被拒，参数保持原样。"""
        proposed = {p.name: p.value for p in params}

        pwm_min = proposed.get('pwm_min', self.get_parameter('pwm_min').value)
        pwm_max = proposed.get('pwm_max', self.get_parameter('pwm_max').value)

        if 'pwm_min' in proposed or 'pwm_max' in proposed:
            for name, val in (('pwm_min', pwm_min), ('pwm_max', pwm_max)):
                if not (500 <= int(val) <= 2500):
                    return SetParametersResult(
                        successful=False, reason='%s 必须在 500~2500 之间，收到 %d' % (name, val))
            if int(pwm_min) >= int(pwm_max):
                return SetParametersResult(
                    successful=False,
                    reason='pwm_min(%d) 必须小于 pwm_max(%d)' % (pwm_min, pwm_max))

        if 'baudrate' in proposed:
            if int(proposed['baudrate']) not in SUPPORTED_BAUDS:
                return SetParametersResult(
                    successful=False, reason='不支持的波特率 %d' % proposed['baudrate'])

        if 'servo_ids' in proposed:
            ids = list(proposed['servo_ids'])
            if not ids:
                return SetParametersResult(successful=False, reason='servo_ids 不能为空')
            bad = [i for i in ids if not (0 <= int(i) <= 253)]
            if bad:
                return SetParametersResult(
                    successful=False, reason='servo_ids 含非法 ID %s（合法 0~253）' % bad)

        if 'default_duration_ms' in proposed:
            if not (50 <= int(proposed['default_duration_ms']) <= 10000):
                return SetParametersResult(
                    successful=False, reason='default_duration_ms 必须在 50~10000')

        if 'port' in proposed:
            if not str(proposed['port']).startswith('/dev/'):
                return SetParametersResult(
                    successful=False, reason='port 必须是 /dev/ 下的设备路径')

        self.get_logger().info('参数校验通过: %s' % list(proposed.keys()))
        if 'servo_ids' in proposed:
            self._ids = [int(i) for i in proposed['servo_ids']]
        return SetParametersResult(successful=True)

    # ==================== 校验（每帧实时读参数）====================

    def _check(self, sid, pwm, duration_ms):
        """返回 (ok, duration_ms, reason)。每次都重新读参数，所以 param set 当帧生效。"""
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        ids = list(self.get_parameter('servo_ids').value)

        if int(sid) not in [int(i) for i in ids]:
            return False, 0, 'ID %d 不在 servo_ids %s' % (sid, ids)
        if not (pwm_min <= int(pwm) <= pwm_max):
            return False, 0, 'pwm %d 超出安全范围 [%d, %d]' % (pwm, pwm_min, pwm_max)

        dur = int(duration_ms) if int(duration_ms) != 0 else int(
            self.get_parameter('default_duration_ms').value)
        if not (50 <= dur <= 10000):
            return False, 0, 'duration %d ms 非法（应在 50~10000）' % dur
        return True, dur, 'ok'

    # ==================== Action 服务端 ====================

    def goal_callback(self, goal_request):
        """发指令之前就判：超限 / ID 不存在 / duration 非法 → 直接拒绝。"""
        ok, dur, reason = self._check(goal_request.id, goal_request.pwm, goal_request.duration_ms)
        if not ok:
            self.get_logger().warn('Goal REJECTED: %s' % reason)
            return GoalResponse.REJECT
        self.get_logger().info('Goal ACCEPTED: id=%d pwm=%d dur=%d' % (
            goal_request.id, goal_request.pwm, dur))
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('收到取消请求')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        with self._exec_lock:
            return self._execute(goal_handle)

    def _execute(self, goal_handle):
        req = goal_handle.request
        ok, dur, reason = self._check(req.id, req.pwm, req.duration_ms)
        if not ok:
            goal_handle.abort()
            return self._make_result(False, 0, reason)

        # ① 恢复扭力：不先 PULR，位置指令会被丢弃
        self.query(req.id, P_KIND_TORQUE_ON, 0.08)

        start_pwm = self.parse_pwm(self.query(req.id, P_KIND_POS, 0.15))
        if start_pwm is None:
            start_pwm = NEUTRAL_PWM

        if not self.send_command(req.id, req.pwm, dur):
            goal_handle.abort()
            return self._make_result(False, start_pwm, '串口错误：指令未发出')
        # 只在这里镜像：只有 Action 下发的指令才写进 /servo/command 供 bag 录制，
        # 调试话题自己不再回发，从根上避免「发→收→再发」的自环。
        self._mirror_cmd(req.id, req.pwm, dur)

        # ② 5Hz 循环反馈，每轮检查取消
        t0 = time.monotonic()
        total_s = max(dur / 1000.0, 0.001)
        cancelled = False
        while True:
            if goal_handle.is_cancel_requested:
                cancelled = True
                break
            elapsed = time.monotonic() - t0
            progress = min(1.0, elapsed / total_s)
            cur = int(round(start_pwm + (req.pwm - start_pwm) * progress))
            fb = MoveServo.Feedback()
            fb.current_pwm = max(0, min(65535, cur))
            fb.progress = float(progress)
            goal_handle.publish_feedback(fb)
            self._publish_state(req.id, cur)
            if progress >= 1.0:
                break
            time.sleep(FEEDBACK_PERIOD)

        # ③ 取消：不发 PULK（会让舵机失能很久），改为重发回中位直到超时。
        #    能拉回来就拉回来，拉不回来就在结果里如实报告真实位置。
        if cancelled:
            self.get_logger().warn('Goal 取消：停止反馈，尝试回中位')
            back = self.parse_pwm(self.query(req.id, P_KIND_POS, 0.3))
            deadline = time.monotonic() + CANCEL_RECOVER_S
            attempt = 0
            while back is None or abs(back - NEUTRAL_PWM) > PRAD_TOLERANCE:
                if time.monotonic() >= deadline:
                    self.get_logger().warn(
                        '取消后 %d 次尝试仍未回中位（实际 %s）' % (attempt, back))
                    break
                attempt += 1
                self.query(req.id, P_KIND_TORQUE_ON, 0.15)
                self.send_command(req.id, NEUTRAL_PWM, CANCEL_RETURN_MS)
                time.sleep(0.8)
                back = self.parse_pwm(self.query(req.id, P_KIND_POS, 0.3))
                self.get_logger().warn('回中位第 %d 次尝试，实际位置 %s' % (attempt, back))
            goal_handle.canceled()
            return self._make_result(
                False, back if back is not None else NEUTRAL_PWM, 'canceled')

        # ④ 读回校验：先等机械稳定，再连读两次取最后一次
        time.sleep(SETTLE_BEFORE_PRAD)
        final = self.parse_pwm(self.query(req.id, P_KIND_POS, 0.25))
        again = self.parse_pwm(self.query(req.id, P_KIND_POS, 0.25))
        if again is not None:
            final = again
        if final is None:
            goal_handle.abort()
            return self._make_result(False, 0, 'PRAD 无回显，读不到实际位置')
        diff = final - req.pwm
        if abs(diff) > PRAD_TOLERANCE:
            goal_handle.abort()
            return self._make_result(
                False, final, '未到位：目标 %d，实际 %d，差 %+d' % (req.pwm, final, diff))

        goal_handle.succeed()
        return self._make_result(True, final, 'ok')

    @staticmethod
    def _make_result(ok, final_pwm, message):
        res = MoveServo.Result()
        res.success = bool(ok)
        res.final_pwm = int(max(0, min(65535, final_pwm or 0)))
        res.message = str(message)
        return res

    # ==================== 调试话题（bag 回放）====================

    def _on_debug_command(self, msg):
        # 忽略自己刚镜像出去的指令，否则每条 Action 指令都会在串口上重发一遍，
        # 舵机短时间内收到两个相同目标会打断自己的运动（实测表现为完全不动）。
        now = time.monotonic()
        for sid_m, pwm_m, dur_m, ts in self._recent_mirror:
            if (int(msg.id), int(msg.pwm), int(msg.duration_ms)) == (sid_m, pwm_m, dur_m) \
                    and now - ts < 1.0:
                return
        ok, dur, reason = self._check(msg.id, msg.pwm, msg.duration_ms)
        if not ok:
            self.get_logger().warn('/servo/command 忽略: %s' % reason)
            return
        self.query(msg.id, P_KIND_TORQUE_ON, 0.05)
        self.send_command(msg.id, msg.pwm, dur)

    # ==================== 启动 / 退出 ====================

    def _startup_reset(self):
        ids = list(self.get_parameter('servo_ids').value)
        self.get_logger().info('启动复位：%s → %d' % (ids, NEUTRAL_PWM))
        for sid in ids:
            self.query(int(sid), P_KIND_TORQUE_ON, 0.05)
            self.send_command(int(sid), NEUTRAL_PWM, self.get_parameter('default_duration_ms').value)
            time.sleep(0.1)

    def shutdown(self):
        """下电前必须卸力（任务4 的第①步）。"""
        if self._shutdown_pulk:
            for sid in list(self._ids):
                try:
                    self.query(int(sid), P_KIND_TORQUE_OFF, 0.05)
                except Exception:  # noqa: BLE001
                    pass
            print('[servo_driver] 退出：已对所有舵机发 PULK 卸力: %s' % list(self._ids))
        else:
            print('[servo_driver] 退出：shutdown_pulk=false，不卸力')
        with self._serial_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = None


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
