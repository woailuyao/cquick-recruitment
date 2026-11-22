#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""servo_driver —— 任务2 完整版（2.2 参数化 + 2.3 Action + 2.6 参数校验 + 2.7 bag）。

四个 Action 回调：
  goal_callback             发串口之前校验（超限 / ID 不在 servo_ids / duration 非法 → REJECT）
  handle_accepted_callback  必须调用 goal_handle.execute()
  cancel_callback           返回 ACCEPT
  execute_callback          发指令 → 10Hz feedback → 10ms 粒度检查取消 → PRAD 读回

参数：7 个必需参数全部由 config/servo_params.yaml 提供，代码里只在 PARAM_DEFAULTS 声明一次。
     所有判断都实时 get_parameter()，不存副本，所以 2.6 的「当帧生效」成立。

bag：每真正发出的一条指令发布到 /servo/command（录下来就是指令流），
     同时订阅它（回放灌回来），自身回显按 (id,pwm,duration)+时间窗吃掉，避免自激。
"""

import re
import sys
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

# ---------------------------------------------------------------------------
# 参数默认值：字面量只出现在这一处（= 声明参数）
# ---------------------------------------------------------------------------
PARAM_DEFAULTS = {
    'port': '/dev/ttyUSB0',
    'baudrate': 115200,
    'pwm_min': 900,
    'pwm_max': 2100,
    'default_duration_ms': 800,
    'servo_ids': [1],
    'startup_reset': True,
}

# 协议常量（协议本身，允许写死）
ID_WIDTH = 3
CMD_PREFIX = '#'
CMD_SUFFIX = '!'
CMD_READ_POS = 'PRAD'
CMD_TORQUE_ON = 'PULR'
CMD_TORQUE_OFF = 'PULK'

# 行为参数
PRAD_TOLERANCE = 30
FEEDBACK_HZ = 10.0
TORQUE_ENABLE_DELAY = 0.1
STATE_HZ = 5.0
ECHO_WINDOW_S = 0.5

NEUTRAL_PWM = 1500
SUPPORTED_BAUDRATES = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')
        self._declare_params()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._serial = None
        self._cb_group = ReentrantCallbackGroup()
        self._motion_lock = threading.Lock()
        self._last_pwm = {}

        # ---- 2.7：bag 用的观察话题 + 回放入口（全文件只在这里创建一次）----
        self._echo_lock = threading.Lock()
        self._echo_pending = []

        self._state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        self._command_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self._command_sub = self.create_subscription(
            ServoCommand, '/servo/command', self._on_command_topic, 10,
            callback_group=self._cb_group,
        )
        self._state_timer = self.create_timer(
            1.0 / STATE_HZ, self._publish_state, callback_group=self._cb_group
        )

        self._open_serial()
        self._startup()

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
        self.get_logger().info('Action Server /move_servo 已就绪')

    # ================================================================ 参数
    def _declare_params(self):
        for name, value in PARAM_DEFAULTS.items():
            self.declare_parameter(name, value)

    def _p(self, name):
        return self.get_parameter(name).value

    def _servo_ids(self):
        return [int(i) for i in self._p('servo_ids')]

    def _neutral(self):
        return NEUTRAL_PWM

    def _on_set_parameters(self, params):
        values = {name: self.get_parameter(name).value for name in PARAM_DEFAULTS}
        for param in params:
            if param.name not in PARAM_DEFAULTS:
                self.get_logger().warn(f'拒绝参数: 不支持的参数 {param.name}')
                return SetParametersResult(successful=False,
                                           reason=f'不支持的参数 {param.name}')
            values[param.name] = param.value

        reason = self._validate_values(values)
        if reason is not None:
            self.get_logger().warn(f'拒绝参数: {reason}')
            return SetParametersResult(successful=False, reason=reason)

        changed = ', '.join(f'{p.name}={p.value}' for p in params)
        self.get_logger().info(f'参数已更新: {changed}（下一帧起按新值校验）')
        return SetParametersResult(successful=True, reason='ok')

    def _validate_values(self, v):
        try:
            pmin = int(v['pwm_min'])
            pmax = int(v['pwm_max'])
        except (TypeError, ValueError):
            return 'pwm_min / pwm_max 必须是整数'

        for name, val in (('pwm_min', pmin), ('pwm_max', pmax)):
            if not 500 <= val <= 2500:
                return f'{name}={val} 超出电气极限 500~2500us'

        if pmin >= pmax:
            return f'pwm_min({pmin}) 必须小于 pwm_max({pmax})'
        if not pmin <= NEUTRAL_PWM <= pmax:
            return f'中位 {NEUTRAL_PWM} 必须落在 [{pmin}, {pmax}] 内'

        baud = int(v['baudrate'])
        if baud not in SUPPORTED_BAUDRATES:
            return f'baudrate={baud} 不是可用的波特率（{list(SUPPORTED_BAUDRATES)}）'

        try:
            ids = [int(i) for i in v['servo_ids']]
        except (TypeError, ValueError):
            return 'servo_ids 必须是整数数组'
        if not ids:
            return 'servo_ids 不能为空'
        if len(set(ids)) != len(ids):
            return f'servo_ids 有重复: {ids}'
        if any(not 1 <= i <= 254 for i in ids):
            return f'servo_ids 必须在 1~254: {ids}'

        dur = int(v['default_duration_ms'])
        if not 1 <= dur <= 65535:
            return f'default_duration_ms={dur} 超出 1~65535ms'
        return None

    # ================================================================ 串口
    def _open_serial(self):
        port, baud = self._p('port'), self._p('baudrate')
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2,
                write_timeout=0.3,
            )
            self.get_logger().info(f'串口已打开 {port} @ {baud} 8-N-1')
        except Exception as exc:
            self.get_logger().error(
                f'打开串口失败 {port}@{baud}: {exc}（查设备名 / dialout 组 / 是否被占用）'
            )
            self._serial = None

    def _write(self, cmd):
        if self._serial is None:
            self.get_logger().error(f'串口未打开，丢弃: {cmd}')
            return False
        try:
            self._serial.write(cmd.encode('ascii'))
            self._serial.flush()
            return True
        except Exception as exc:
            self.get_logger().error(f'写串口失败: {exc}')
            return False

    def _frame(self, sid, body):
        return f'{CMD_PREFIX}{int(sid):0{ID_WIDTH}d}{body}{CMD_SUFFIX}'

    def send_move(self, sid, pwm, duration_ms=None):
        pmin = int(self._p('pwm_min'))
        pmax = int(self._p('pwm_max'))
        if not pmin <= pwm <= pmax:
            self.get_logger().warn(f'pwm={pwm} 超出 [{pmin},{pmax}]，拒绝下发')
            return False
        if int(sid) not in self._servo_ids():
            self.get_logger().warn(f'id={sid} 不在 servo_ids={self._servo_ids()} 里，拒绝下发')
            return False

        dur = int(duration_ms) if duration_ms is not None else int(self._p('default_duration_ms'))
        cmd = self._frame(sid, f'P{pwm}T{dur}')
        self.get_logger().info(f'TX {cmd}')
        if not self._write(cmd):
            return False

        self._last_pwm[int(sid)] = int(pwm)
        self._publish_command(int(sid), int(pwm), int(dur))
        return True

    def send_raw(self, sid, keyword):
        cmd = self._frame(sid, keyword)
        self.get_logger().info(f'TX {cmd}')
        return self._write(cmd)

    def read_position(self, sid):
        """发 #xxxPRAD! 读回实际位置；读不到返回 None。"""
        if self._serial is None:
            return None

        cmd = self._frame(sid, CMD_READ_POS)
        try:
            self._serial.reset_input_buffer()
            self._serial.write(cmd.encode('ascii'))
            self._serial.flush()
            time.sleep(0.1)

            buf = b''
            deadline = time.time() + 0.8
            while time.time() < deadline:
                chunk = self._serial.read(64)
                if chunk:
                    buf += chunk
                    if b'!' in buf:
                        break
                else:
                    time.sleep(0.01)
        except Exception as exc:
            self.get_logger().error(f'读位置失败: {exc}')
            return None

        raw = buf.decode('ascii', 'replace').strip()
        self.get_logger().info(f'RX {raw!r}')
        if not raw:
            return None

        upper = raw.upper()
        if CMD_READ_POS in upper:
            match = re.match(r'\d+', upper.split(CMD_READ_POS, 1)[1])
            return int(match.group()) if match else None

        nums = re.findall(r'\d+', raw)
        return int(nums[-1]) if len(nums) >= 2 else None

    # ================================================= 2.7 bag：观察 + 回放
    def _publish_command(self, sid, pwm, dur):
        now = time.monotonic()
        with self._echo_lock:
            self._echo_pending = [e for e in self._echo_pending
                                  if e[3] > now - ECHO_WINDOW_S]
            self._echo_pending.append((int(sid), int(pwm), int(dur), now))

        msg = ServoCommand()
        msg.id = int(sid)
        msg.pwm = int(pwm)
        msg.duration_ms = int(dur)
        self._command_pub.publish(msg)

    def _consume_echo(self, key):
        now = time.monotonic()
        with self._echo_lock:
            self._echo_pending = [e for e in self._echo_pending
                                  if e[3] > now - ECHO_WINDOW_S]
            for index, entry in enumerate(self._echo_pending):
                if entry[:3] == key:
                    del self._echo_pending[index]
                    return True
        return False

    def _on_command_topic(self, msg):
        key = (int(msg.id), int(msg.pwm), int(msg.duration_ms))
        if self._consume_echo(key):
            self.get_logger().debug(f'/servo/command 忽略自身回显 {key}')
            return

        if key[0] not in self._servo_ids():
            self.get_logger().warn(f'/servo/command 丢弃（ID 不在 servo_ids）: {key}')
            return

        dur = key[2] or int(self._p('default_duration_ms'))
        if self.send_move(key[0], key[1], dur):
            self.get_logger().info(f'回放指令: id={key[0]} pwm={key[1]} duration={dur}ms')

    def _publish_state(self):
        for sid in self._servo_ids():
            msg = ServoCommand()
            msg.id = int(sid)
            msg.pwm = int(self._last_pwm.get(int(sid), NEUTRAL_PWM))
            msg.duration_ms = 0
            self._state_pub.publish(msg)

    # ================================================================ 启动
    def _startup(self):
        ids = self._servo_ids()
        for sid in ids:
            self.send_raw(sid, CMD_TORQUE_ON)
        self.get_logger().info(f'已对 {ids} 发送 PULR 恢复扭矩')
        time.sleep(TORQUE_ENABLE_DELAY)

        if bool(self._p('startup_reset')):
            for sid in ids:
                self.send_move(sid, self._neutral())
            self.get_logger().info(f'startup_reset: {ids} → {self._neutral()}')

    def shutdown(self):
        for sid in self._servo_ids():
            self.send_raw(sid, CMD_TORQUE_OFF)
        if self._serial is not None:
            self._serial.close()

    # ============================================================== 四个回调
    def _validate(self, cmd):
        sid = int(cmd.id)
        if sid not in self._servo_ids():
            return f'id={sid} 不在 servo_ids={self._servo_ids()} 里'
        pmin = int(self._p('pwm_min'))
        pmax = int(self._p('pwm_max'))
        if not pmin <= int(cmd.pwm) <= pmax:
            return f'pwm={cmd.pwm} 超出安全区间 [{pmin}, {pmax}]'
        dur = int(cmd.duration_ms) or int(self._p('default_duration_ms'))
        if not 1 <= dur <= 65535:
            return f'duration_ms={dur} 非法（1~65535）'
        return None

    def goal_callback(self, goal_request):
        reason = self._validate(goal_request.command)
        if reason is not None:
            self.get_logger().warn(f'goal 被拒: {reason}')
            return GoalResponse.REJECT
        cmd = goal_request.command
        self.get_logger().info(
            f'goal 接受: id={cmd.id} pwm={cmd.pwm} duration={cmd.duration_ms}ms'
        )
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('goal 已接受，开始执行')
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('收到取消请求 → ACCEPT')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        cmd = goal_handle.request.command
        sid = int(cmd.id)
        target = int(cmd.pwm)
        duration = int(cmd.duration_ms) or int(self._p('default_duration_ms'))
        start = self._last_pwm.get(sid, self._neutral())

        with self._motion_lock:
            self.get_logger().info(f'开始: id={sid} {start} → {target}, {duration}ms')

            self.send_raw(sid, CMD_TORQUE_ON)
            time.sleep(TORQUE_ENABLE_DELAY)

            if not self.send_move(sid, target, duration):
                goal_handle.abort()
                return MoveServo.Result(
                    success=False, actual_pwm=start, message='串口写入失败'
                )

            period = 1.0 / FEEDBACK_HZ
            total = duration / 1000.0
            t0 = time.monotonic()
            next_tick = t0
            canceled = False

            while True:
                if goal_handle.is_cancel_requested:
                    canceled = True
                    break

                elapsed = time.monotonic() - t0
                progress = 1.0 if elapsed >= total else elapsed / total
                current = int(round(start + (target - start) * progress))

                fb = MoveServo.Feedback()
                fb.servo_id = sid
                fb.current_pwm = current
                fb.progress = float(progress)
                goal_handle.publish_feedback(fb)

                if progress >= 1.0:
                    break

                next_tick += period
                while time.monotonic() < next_tick:
                    if goal_handle.is_cancel_requested:
                        canceled = True
                        break
                    time.sleep(0.01)
                if canceled:
                    break

            if canceled:
                self.get_logger().warn('取消生效：停发 → 回中位')
                self.send_move(sid, self._neutral())
                time.sleep(int(self._p('default_duration_ms')) / 1000.0)
                goal_handle.canceled()
                return MoveServo.Result(
                    success=False,
                    actual_pwm=self._last_pwm.get(sid, start),
                    message='canceled',
                )

            actual = self.read_position(sid)
            if actual is None:
                self.get_logger().error('PRAD 无回显')
                goal_handle.abort()
                return MoveServo.Result(
                    success=False, actual_pwm=0,
                    message='PRAD 无回显（检查共地 / 波特率 / 回显开关）',
                )

            diff = actual - target
            ok = abs(diff) <= PRAD_TOLERANCE
            if ok:
                goal_handle.succeed()
                msg = f'到位 actual={actual} target={target}'
            else:
                goal_handle.abort()
                msg = f'未到位 actual={actual} target={target} 差 {diff}'

            self.get_logger().info(f'完成: success={ok} {msg}')
            return MoveServo.Result(success=ok, actual_pwm=actual, message=msg)


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
        try:
            node.shutdown()
        except Exception as exc:  # noqa: BLE001
            print(f'[servo_driver] 退出清理异常: {exc!r}', file=sys.stderr)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
