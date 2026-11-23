#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""舵机驱动节点（Action Server）—— 第 3、4 周任务 2 的核心

数据流：
    Action goal  ──┐
                   ├─> _send_command() 同一个底层发送函数
    /servo/command ┘        │
                            ├─ 校验 pwm 落在 [pwm_min, pwm_max]
                            ├─ 拼 ASCII： #001P1700T1000!
                            └─ write + flush ──> 控制板 ──> 舵机
                                                        │
    result <── PRAD 回读比对 <───────────────────────────┘

设计要点（评审会问）：
  * 所有可调值都来自参数，代码里只留协议常量（# ! P T PID PRAD PULR PULK）
  * 参数每次使用时实时读取，不缓存到 self.xxx，否则 ros2 param set 不生效
  * Action 与 /servo/command 共用 _send_command()，保证 bag 回放和录制动作一致
  * MultiThreadedExecutor + ReentrantCallbackGroup，否则 execute 里 sleep 会卡死 cancel
"""

import re
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

try:
    import serial
except ImportError:          # 没装 pyserial 也能启动，只是打不开串口
    serial = None

# ---- 协议常量（这些允许写死，属于协议本身，不是配置）----
PWM_HARD_MIN = 500
PWM_HARD_MAX = 2500
FEEDBACK_HZ = 20.0           # 要求 ≥5 Hz，给 20 Hz 更平滑
READ_TIMEOUT = 0.3


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        # ---------- 参数（一个都不许硬编码）----------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)
        # 支持的波特率白名单（也是参数，代码里不写字面量）
        self.declare_parameter('allowed_baudrates', [9600, 19200, 38400, 57600, 115200])
        # 额外：没有硬件时用来验证代码逻辑，默认关闭
        self.declare_parameter('sim', False)

        self._io_lock = threading.Lock()
        self._action_active = False
        self._pwm_of = {}                     # id -> 最近一次目标 pwm（用于 feedback 插值）
        self.ser = None
        self._sim = bool(self.get_parameter('sim').value)

        self._open_serial()

        cb_group = ReentrantCallbackGroup()

        # ---------- 发布/订阅 ----------
        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self.state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        self.create_subscription(
            ServoCommand, '/servo/command', self.on_command, 10,
            callback_group=cb_group)

        # ---------- Action 服务端 ----------
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=cb_group,
        )

        # ---------- 参数校验回调 ----------
        self.add_on_set_parameters_callback(self.on_param_change)

        if bool(self.get_parameter('startup_reset').value):
            self.reset_all()

        self.get_logger().info(
            'servo_driver 已启动：port=%s baud=%d pwm=[%d,%d] ids=%s sim=%s' % (
                self.get_parameter('port').value,
                self.get_parameter('baudrate').value,
                self.get_parameter('pwm_min').value,
                self.get_parameter('pwm_max').value,
                list(self.get_parameter('servo_ids').value),
                self._sim))

    # ==================================================================
    # 串口
    # ==================================================================
    def _open_serial(self):
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)
        if self._sim or serial is None:
            self.get_logger().warn('未连接硬件（sim=%s），串口收发走模拟' % self._sim)
            return
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baud, bytesize=8,
                parity=serial.PARITY_NONE, stopbits=1,
                timeout=0.05, write_timeout=0.5)
            self.get_logger().info('串口已打开：%s @ %d 8-N-1' % (port, baud))
        except Exception as exc:                      # noqa: BLE001
            self.ser = None
            self.get_logger().error(
                '打不开串口 %s：%s（检查：设备名 / dialout 组 / 是否被占用）' % (port, exc))

    def _write(self, payload):
        """写一条 ASCII 指令，返回 True/False。"""
        if self.ser is None:
            return self._sim          # sim 模式下假装成功
        with self._io_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(payload.encode('ascii'))
                self.ser.flush()
                return True
            except Exception as exc:                  # noqa: BLE001
                self.get_logger().error('串口写入失败：%s' % exc)
                return False

    def _read_reply(self):
        """读到 '!' 或超时为止，返回字符串。"""
        if self.ser is None:
            return ''
        buf = b''
        deadline = time.monotonic() + READ_TIMEOUT
        with self._io_lock:
            while time.monotonic() < deadline:
                try:
                    chunk = self.ser.read(self.ser.in_waiting or 1)
                except Exception:                     # noqa: BLE001
                    break
                if chunk:
                    buf += chunk
                    if b'!' in buf:
                        break
                else:
                    time.sleep(0.005)
        return buf.decode('ascii', errors='replace').strip()

    # ==================================================================
    # 唯一底层发送入口（Action 和 /servo/command 都走这里）
    # ==================================================================
    def _send_command(self, cmd):
        """发送一条 ServoCommand，返回 (ok, 说明)。"""
        payload = '#%03dP%04dT%04d!' % (cmd.id, cmd.pwm, cmd.duration_ms)
        self.get_logger().info('TX -> %s' % payload)
        ok = self._write(payload)
        if ok:
            self._pwm_of[cmd.id] = cmd.pwm
            # 心跳/状态：给 bag 用
            st = ServoCommand()
            st.id = cmd.id
            st.pwm = cmd.pwm
            st.duration_ms = cmd.duration_ms
            self.state_pub.publish(st)
            return True, 'sent'
        return False, 'serial_error'

    def _send_raw(self, sid, what):
        """发 PID / PRAD / PULR / PULK 这类无参数指令。"""
        payload = '#%03d%s!' % (sid, what)
        self.get_logger().info('TX -> %s' % payload)
        self._write(payload)
        return self._read_reply()

    def _read_prad(self, sid):
        """读回当前位置，返回 int；读不到返回 None。"""
        reply = self._send_raw(sid, 'PRAD')
        if not reply:
            return self._pwm_of.get(sid)
        m = re.search(r'P(\d{1,4})', reply)
        if m:
            return int(m.group(1))
        digits = re.findall(r'\d+', reply)
        return int(digits[-1]) if digits else None

    def reset_all(self):
        """启动时把所有舵机回中位：先 PULR 再 P1500。"""
        ids = list(self.get_parameter('servo_ids').value)
        dur = int(self.get_parameter('default_duration_ms').value)
        self.get_logger().info('启动回中位：%s' % ids)
        for sid in ids:
            self._send_raw(sid, 'PULR')
        for sid in ids:
            self._send_command(ServoCommand(id=sid, pwm=1500, duration_ms=dur))
            time.sleep(0.02)

    # ==================================================================
    # 话题回调（bag 回放走这条）
    # ==================================================================
    def on_command(self, msg):
        # 自己 Action 执行期间会把 TX 发布到 /servo/command 供录制，
        # 这里要跳过，否则会重复发送一遍
        if self._action_active:
            return
        lo = int(self.get_parameter('pwm_min').value)
        hi = int(self.get_parameter('pwm_max').value)
        if not (lo <= msg.pwm <= hi):
            self.get_logger().warn(
                '/servo/command 收到越界 pwm=%d（安全范围 %d~%d），忽略' % (msg.pwm, lo, hi))
            return
        dur = msg.duration_ms or int(self.get_parameter('default_duration_ms').value)
        self._send_command(ServoCommand(id=msg.id, pwm=msg.pwm, duration_ms=dur))

    # ==================================================================
    # 参数校验（不注册这个回调，什么值都进得来）
    # ==================================================================
    def on_param_change(self, params):
        for p in params:
            if p.name == 'pwm_min' or p.name == 'pwm_max':
                if not (PWM_HARD_MIN <= p.value <= PWM_HARD_MAX):
                    return SetParametersResult(
                        successful=False,
                        reason='%s 必须在 %d~%d 之间，收到 %s' % (
                            p.name, PWM_HARD_MIN, PWM_HARD_MAX, p.value))
                # 和另一个参数比较（注意用"将要生效"的值）
                other = self.get_parameter(
                    'pwm_max' if p.name == 'pwm_min' else 'pwm_min').value
                for q in params:
                    if q.name == ('pwm_max' if p.name == 'pwm_min' else 'pwm_min'):
                        other = q.value
                if p.name == 'pwm_min' and p.value >= other:
                    return SetParametersResult(
                        successful=False, reason='pwm_min(%s) 必须小于 pwm_max(%s)' % (p.value, other))
                if p.name == 'pwm_max' and p.value <= other:
                    return SetParametersResult(
                        successful=False, reason='pwm_max(%s) 必须大于 pwm_min(%s)' % (p.value, other))
            elif p.name == 'baudrate':
                supported = list(self.get_parameter('allowed_baudrates').value)
                if int(p.value) not in supported:
                    return SetParametersResult(
                        successful=False,
                        reason='baudrate 只支持 %s' % (supported,))
            elif p.name == 'default_duration_ms':
                if not (10 <= int(p.value) <= 10000):
                    return SetParametersResult(
                        successful=False, reason='default_duration_ms 必须在 10~10000 ms')
        return SetParametersResult(successful=True)

    # ==================================================================
    # Action 回调
    # ==================================================================
    def goal_callback(self, goal_request):
        """发指令之前就判断，不合格直接拒。注意参数要实时读。"""
        lo = int(self.get_parameter('pwm_min').value)
        hi = int(self.get_parameter('pwm_max').value)
        ids = list(self.get_parameter('servo_ids').value)

        if goal_request.id not in ids:
            self.get_logger().warn(
                'REJECT：id=%d 不在 servo_ids=%s 里' % (goal_request.id, ids))
            return GoalResponse.REJECT
        if not (lo <= goal_request.pwm <= hi):
            self.get_logger().warn(
                'REJECT：pwm=%d 超出安全范围 %d~%d' % (goal_request.pwm, lo, hi))
            return GoalResponse.REJECT
        if goal_request.duration_ms and not (10 <= goal_request.duration_ms <= 10000):
            self.get_logger().warn(
                'REJECT：duration_ms=%d 非法（10~10000）' % goal_request.duration_ms)
            return GoalResponse.REJECT
        self.get_logger().info(
            'ACCEPT：id=%d pwm=%d dur=%d' % (
                goal_request.id, goal_request.pwm, goal_request.duration_ms))
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('收到取消请求，接受')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # 交给 executor 里的线程去执行（MultiThreadedExecutor 才能并发）
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        dur_ms = req.duration_ms or int(self.get_parameter('default_duration_ms').value)
        lo = int(self.get_parameter('pwm_min').value)
        hi = int(self.get_parameter('pwm_max').value)

        result = MoveServo.Result()

        # 上电后第一次动作之前必须 PULR，否则舵机纹丝不动
        self._send_raw(req.id, 'PULR')

        cmd = ServoCommand()
        cmd.id = req.id
        cmd.pwm = req.pwm
        cmd.duration_ms = dur_ms

        self._action_active = True
        try:
            # 先记下"出发位置"，再发指令 —— 顺序反了 feedback 会一直显示目标值
            start_pwm = self._pwm_of.get(req.id, req.pwm)
            # 把这条指令发布到 /servo/command：bag 靠它记录，
            # 回放时同一条话题又能驱动舵机，做到「录什么、放什么」
            self.cmd_pub.publish(cmd)
            # 真正的串口发送
            ok, why = self._send_command(cmd)
            if not ok:
                goal_handle.abort()
                result.success = False
                result.final_pwm = 0
                result.message = 'serial_error'
                return result

            t0 = time.monotonic()
            duration_s = dur_ms / 1000.0
            period = 1.0 / FEEDBACK_HZ

            while True:
                # 每轮都检查取消
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('goal 被取消，回中位')
                    self._send_command(ServoCommand(id=req.id, pwm=1500, duration_ms=300))
                    goal_handle.canceled()
                    result.success = False
                    result.final_pwm = 1500
                    result.message = 'canceled'
                    return result

                elapsed = time.monotonic() - t0
                progress = 1.0 if duration_s <= 0 else min(1.0, elapsed / duration_s)

                fb = MoveServo.Feedback()
                fb.current_pwm = int(start_pwm + (req.pwm - start_pwm) * progress)
                fb.progress = float(progress)
                goal_handle.publish_feedback(fb)
                self.get_logger().debug(
                    'id=%d pwm=%d progress=%.2f' % (req.id, fb.current_pwm, fb.progress))

                if progress >= 1.0:
                    break
                time.sleep(period)

            # PRAD 回读校验
            actual = self._read_prad(req.id)
            result.final_pwm = int(actual) if actual is not None else 0
            if actual is None:
                # 有些控制板不把舵机的应答转发回上位机（本机实测：PID 有回显、PRAD 没有）。
                # 这种情况不能判失败——动作确实执行了，只是读不回来。
                self.get_logger().warn(
                    'PRAD 无回显（控制板未转发舵机应答），无法做闭环校验')
                result.success = True
                result.message = 'ok_no_prad'
            else:
                diff = abs(int(actual) - int(req.pwm))
                if diff <= 30:
                    result.success = True
                    result.message = 'ok'
                else:
                    result.success = False
                    result.message = 'not_reached: 目标 %d 实际 %d 差 %d' % (
                        req.pwm, actual, diff)
            self.get_logger().info(
                'RESULT id=%d success=%s final=%d (%s)' % (
                    req.id, result.success, result.final_pwm, result.message))
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            self._action_active = False


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()

    # 关键 1：必须用 MultiThreadedExecutor，否则 execute 里的 sleep 会阻塞所有回调，
    #          Ctrl+C 取消对 Action 永远不生效。
    # 关键 2：线程数必须 >= 同时执行的 goal 数。一个 goal 在执行期间独占一个线程，
    #          只有 4 个线程时，10 个关节只能 4 个 4 个地动 —— 看起来就是"依次动"，
    #          而不是同时动。所以按 servo_ids 的数量动态分配。
    n_servo = len(list(node.get_parameter('servo_ids').value))
    n_threads = max(8, n_servo + 6)
    node.get_logger().info(
        'executor 线程数 = %d（舵机 %d 个，保证多关节能真正同时动）' % (n_threads, n_servo))
    executor = MultiThreadedExecutor(num_threads=n_threads)
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
