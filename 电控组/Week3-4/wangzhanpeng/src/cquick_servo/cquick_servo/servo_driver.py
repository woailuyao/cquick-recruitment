#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from cquick_servo.protocol import (
    move_command,
    parse_prad,
    prad_command,
    pulk_command,
    pulr_command,
)
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand, ServoState


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('port', '/dev/cq_servo')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 1000)
        self.declare_parameter('min_duration_ms', 1000)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)
        self.declare_parameter('center_pwm', 1500)
        self.declare_parameter('tolerance', 30)
        self.declare_parameter('feedback_hz', 10.0)
        self.declare_parameter('max_duration_ms', 10000)

        self.add_on_set_parameters_callback(self._on_param_set)

        self._io_lock = threading.RLock()
        self.ser = None
        self._open_serial()

        self._last_announced = None
        self._last_announced_time = 0.0
        self._cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self.create_subscription(ServoCommand, '/servo/command', self._on_command, 10)
        self._state_pub = self.create_publisher(ServoState, '/servo/state', 10)

        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        if self.get_parameter('startup_reset').value:
            self._startup_reset()

        self.get_logger().info('servo_driver ready: Action=/move_servo')

    # ==================================================================
    # ==================================================================
    def _on_param_set(self, params):
        proposed = {p.name: p.value for p in params}

        def effective(name):
            return proposed[name] if name in proposed else self.get_parameter(name).value

        for name in ('pwm_min', 'pwm_max'):
            if name in proposed:
                value = proposed[name]
                if isinstance(value, bool) or not isinstance(value, int):
                    return SetParametersResult(
                        successful=False, reason=f'{name} 必须是整数')
                if not 500 <= value <= 2500:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{name}={value} 超出允许范围 500~2500')

        if 'pwm_min' in proposed or 'pwm_max' in proposed:
            low, high = effective('pwm_min'), effective('pwm_max')
            if low >= high:
                return SetParametersResult(
                    successful=False,
                    reason=f'pwm_min({low}) 必须小于 pwm_max({high})')

        if 'baudrate' in proposed:
            if proposed['baudrate'] not in (9600, 19200, 38400, 57600, 115200, 230400):
                return SetParametersResult(
                    successful=False,
                    reason=f"不支持的波特率 {proposed['baudrate']}")

        if 'servo_ids' in proposed:
            servo_ids = proposed['servo_ids']
            if not servo_ids or any(i < 1 or i > 254 for i in servo_ids):
                return SetParametersResult(
                    successful=False, reason='servo_ids 必须是 1~254 的非空列表')

        if 'default_duration_ms' in proposed:
            if not 50 <= proposed['default_duration_ms'] <= 10000:
                return SetParametersResult(
                    successful=False, reason='default_duration_ms 必须在 50~10000')

        if 'min_duration_ms' in proposed:
            if not 100 <= proposed['min_duration_ms'] <= 10000:
                return SetParametersResult(
                    successful=False, reason='min_duration_ms 必须在 100~10000')

        if 'max_duration_ms' in proposed:
            if not 50 <= proposed['max_duration_ms'] <= 60000:
                return SetParametersResult(
                    successful=False, reason='max_duration_ms 必须在 50~60000')

        if 'center_pwm' in proposed:
            low, high = effective('pwm_min'), effective('pwm_max')
            if not low <= proposed['center_pwm'] <= high:
                return SetParametersResult(
                    successful=False, reason='center_pwm 必须落在 pwm_min~pwm_max 内')

        if 'feedback_hz' in proposed:
            if not 1.0 <= proposed['feedback_hz'] <= 50.0:
                return SetParametersResult(
                    successful=False, reason='feedback_hz 必须在 1~50')

        return SetParametersResult(successful=True)

    # ==================================================================
    # ==================================================================
    def _open_serial(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.5,
            )
            self.get_logger().info(f'串口已打开: {port} @ {baud} 8-N-1')
        except Exception as exc:
            self.ser = None
            self.get_logger().error(f'串口打开失败: {port} @ {baud} -> {exc}')

    def _ensure_serial(self):
        if self.ser is not None and self.ser.is_open:
            return True
        self.get_logger().warn('串口不可用，尝试重新打开 ...')
        self._open_serial()
        return self.ser is not None

    def _write(self, text):
        with self._io_lock:
            if self.ser is None or not self.ser.is_open:
                raise serial.SerialException('serial port is not open')
            self.ser.write(text.encode('ascii'))
            self.ser.flush()
            self.get_logger().debug(f'TX {text}')

    def _read_reply(self, timeout):
        deadline = time.monotonic() + timeout
        chunks = b''
        with self._io_lock:
            while time.monotonic() < deadline:
                waiting = self.ser.in_waiting
                if waiting:
                    chunks += self.ser.read(waiting)
                else:
                    time.sleep(0.005)
        return chunks

    def _command(self, text, reply_timeout=0.15):
        if not self._ensure_serial():
            return b''
        with self._io_lock:
            try:
                self.ser.reset_input_buffer()
            except serial.SerialException:
                self.ser = None
                return b''
            try:
                self._write(text)
            except serial.SerialException as exc:
                self.get_logger().error(f'串口写失败: {exc}')
                self.ser = None
                return b''
            return self._read_reply(reply_timeout)

    def _query_prad(self, servo_id, timeout=0.15):
        raw = self._command(prad_command(servo_id), reply_timeout=timeout)
        value = parse_prad(raw)
        if value is None:
            self.get_logger().debug(f'PRAD 无回读 (id={servo_id}), 收到: {raw!r}')
        return value

    def _send_pulr(self, servo_id):
        self._command(pulr_command(servo_id), reply_timeout=0.1)
        time.sleep(0.05)

    # ==================================================================
    # ==================================================================
    def _move_raw(self, servo_id, pwm, duration_ms, *, restore_torque=True, announce=True):
        if not self._ensure_serial():
            return False

        if announce:
            self._last_announced = (int(servo_id), int(pwm), int(duration_ms))
            self._last_announced_time = time.monotonic()
            self._publish_command(servo_id, pwm, duration_ms)

        if restore_torque:
            self._send_pulr(servo_id)

        try:
            self._write(move_command(servo_id, pwm, duration_ms))
        except serial.SerialException as exc:
            self.get_logger().error(f'串口写失败: {exc}')
            self.ser = None
            return False
        return True

    def _publish_command(self, servo_id, pwm, duration_ms):
        msg = ServoCommand()
        msg.id = int(servo_id)
        msg.pwm = int(pwm)
        msg.duration_ms = int(duration_ms)
        self._cmd_pub.publish(msg)

    def _publish_state(self, servo_id, current_pwm, target_pwm, progress, moving):
        msg = ServoState()
        msg.id = int(servo_id)
        msg.current_pwm = int(max(0, min(65535, current_pwm)))
        msg.target_pwm = int(max(0, min(65535, target_pwm)))
        msg.progress = float(progress)
        msg.moving = bool(moving)
        self._state_pub.publish(msg)

    # ==================================================================
    # ==================================================================
    def _on_command(self, msg):
        key = (int(msg.id), int(msg.pwm), int(msg.duration_ms))
        if (key == self._last_announced
                and (time.monotonic() - self._last_announced_time) < 1.0):
            self.get_logger().debug(f'/servo/command 自回环，已忽略: {key}')
            return

        ok, reason = self._check_target(msg.id, msg.pwm)
        if not ok:
            self.get_logger().warn(f'/servo/command 丢弃: {reason}')
            return

        duration_ms = int(msg.duration_ms)
        if duration_ms <= 0:
            duration_ms = int(self.get_parameter('default_duration_ms').value)
        duration_ms = max(
            int(self.get_parameter('min_duration_ms').value), duration_ms)

        self.get_logger().info(
            f'/servo/command: id={msg.id} pwm={msg.pwm} duration_ms={duration_ms}')
        self._move_raw(
            msg.id, msg.pwm, duration_ms,
            restore_torque=False,
            announce=False)

    # ==================================================================
    # ==================================================================
    def _startup_reset(self):
        if not self._ensure_serial():
            self.get_logger().error('startup_reset 跳过：串口不可用')
            return
        servo_ids = [int(i) for i in self.get_parameter('servo_ids').value]
        center = int(self.get_parameter('center_pwm').value)
        duration_ms = max(1, int(self.get_parameter('default_duration_ms').value))

        for servo_id in servo_ids:
            self._send_pulr(servo_id)
        for servo_id in servo_ids:
            try:
                self._write(move_command(servo_id, center, duration_ms))
            except serial.SerialException as exc:
                self.get_logger().error(f'startup_reset 失败: {exc}')
                self.ser = None
                return
        self.get_logger().info(
            f'startup_reset: {servo_ids} -> pwm={center} (T{duration_ms}ms)')
        time.sleep(duration_ms / 1000.0)

    # ==================================================================
    # ==================================================================
    def _check_target(self, servo_id, pwm):
        servo_ids = list(self.get_parameter('servo_ids').value)
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        if servo_id not in servo_ids:
            return False, f'id={servo_id} 不在 servo_ids {servo_ids} 里'
        if not pwm_min <= pwm <= pwm_max:
            return False, f'pwm={pwm} 超出安全范围 {pwm_min}~{pwm_max}'
        return True, ''

    def goal_cb(self, goal_request):
        ok, reason = self._check_target(goal_request.id, goal_request.pwm)
        if not ok:
            self.get_logger().error(f'Goal rejected: {reason}')
            return GoalResponse.REJECT

        max_duration = self.get_parameter('max_duration_ms').value
        if goal_request.duration_ms > max_duration:
            self.get_logger().error(
                f'Goal rejected: duration_ms={goal_request.duration_ms} '
                f'超过上限 {max_duration}')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Goal accepted: id={goal_request.id} pwm={goal_request.pwm} '
            f'duration_ms={goal_request.duration_ms}')
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn(f'收到取消请求 (id={goal_handle.request.id})')
        return CancelResponse.ACCEPT

    def _stop_and_center(self, servo_id):
        center = int(self.get_parameter('center_pwm').value)
        tolerance = int(self.get_parameter('tolerance').value)
        center_duration_ms = max(1000, int(self.get_parameter('min_duration_ms').value))

        with self._io_lock:
            try:
                self._write(pulk_command(servo_id))
            except serial.SerialException as exc:
                self.get_logger().error(f'取消时 PULK 失败: {exc}')
                self.ser = None
                return None
        time.sleep(0.05)

        self._send_pulr(servo_id)
        try:
            self._write(move_command(servo_id, center, center_duration_ms))
        except serial.SerialException as exc:
            self.get_logger().error(f'取消时回中位失败: {exc}')
            self.ser = None
            return None

        deadline = time.monotonic() + center_duration_ms / 1000.0 + 1.0
        last = None
        while time.monotonic() < deadline:
            last = self._query_prad(servo_id, timeout=0.1)
            if last is not None and abs(last - center) <= tolerance:
                break
            time.sleep(0.05)
        return last

    def execute_cb(self, goal_handle):
        req = goal_handle.request
        servo_id = int(req.id)
        target = int(req.pwm)

        duration_ms = int(req.duration_ms)
        if duration_ms <= 0:
            duration_ms = int(self.get_parameter('default_duration_ms').value)
        min_duration = int(self.get_parameter('min_duration_ms').value)
        if duration_ms < min_duration:
            self.get_logger().warn(
                f'duration_ms={duration_ms} 低于下限 {min_duration}'
                f'（本板 T<1000ms 不执行），按 {min_duration} 执行')
            duration_ms = min_duration
        duration_s = duration_ms / 1000.0

        tolerance = int(self.get_parameter('tolerance').value)
        rate = max(1.0, float(self.get_parameter('feedback_hz').value))
        period = 1.0 / rate

        if not self._ensure_serial():
            goal_handle.abort()
            return MoveServo.Result(
                success=False, final_pwm=0, message='串口不可用')

        start = self._query_prad(servo_id)
        if start is None:
            self.get_logger().warn(
                f'动作前 PRAD 读回失败 (id={servo_id})，用目标值当起点')
            start = target

        if not self._move_raw(servo_id, target, duration_ms):
            goal_handle.abort()
            return MoveServo.Result(
                success=False, final_pwm=int(start), message='串口写入失败')

        begin = time.monotonic()
        end = begin + duration_s
        last_pwm = int(start)

        while True:
            if goal_handle.is_cancel_requested:
                final = self._stop_and_center(servo_id)
                goal_handle.canceled()
                if final is None:
                    final = last_pwm
                self._publish_state(servo_id, final, target, 0.0, False)
                self.get_logger().warn(f'动作已取消，回中位: final_pwm={final}')
                return MoveServo.Result(
                    success=False, final_pwm=int(final), message='canceled')

            now = time.monotonic()
            if now >= end:
                break

            progress = min(1.0, (now - begin) / duration_s)
            measured = self._query_prad(servo_id, timeout=0.05)
            if measured is not None:
                current = int(measured)
                last_pwm = current
            else:
                current = int(round(start + (target - start) * progress))

            self._publish_state(servo_id, current, target, progress, True)
            goal_handle.publish_feedback(
                MoveServo.Feedback(current_pwm=current, progress=float(progress)))
            time.sleep(max(0.0, min(period, end - time.monotonic())))

        time.sleep(0.15)
        final_pwm = self._query_prad(servo_id, timeout=0.3)
        if final_pwm is None:
            final_pwm = self._query_prad(servo_id, timeout=0.3)
        if final_pwm is None:
            goal_handle.abort()
            self._publish_state(servo_id, last_pwm, target, 1.0, False)
            return MoveServo.Result(
                success=False, final_pwm=int(last_pwm),
                message='指令已发，但 PRAD 读回失败，无法确认是否到位')

        error = abs(int(final_pwm) - target)
        self._publish_state(servo_id, final_pwm, target, 1.0, False)

        if error > tolerance:
            goal_handle.abort()
            self.get_logger().error(
                f'未到位: 目标 {target}, 实际 {final_pwm}, 误差 {error} > {tolerance}')
            return MoveServo.Result(
                success=False, final_pwm=int(final_pwm),
                message=f'未到位: 目标 {target}, 实际 {final_pwm}, 误差 {error}')

        goal_handle.succeed()
        self.get_logger().info(
            f'到位: 目标 {target}, 实际 {final_pwm}, 误差 {error}')
        return MoveServo.Result(
            success=True, final_pwm=int(final_pwm), message=f'到位 (误差 {error})')

    # ==================================================================
    def destroy_node(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
