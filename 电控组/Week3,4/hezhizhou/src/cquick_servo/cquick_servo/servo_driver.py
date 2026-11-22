#!/usr/bin/env python3
"""舵机驱动节点：对外提供 MoveServo Action，并订阅 /servo/command 供 bag 回放。"""

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


def build_move_cmd(servo_id, pwm, duration_ms):
    return f'#{servo_id:03d}P{pwm:04d}T{duration_ms:04d}!'


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('port', '/dev/cq_servo')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 1000)
        self.declare_parameter('pwm_max', 2000)
        self.declare_parameter('default_duration_ms', 1600)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.declare_parameter('startup_reset', False)

        self.add_on_set_parameters_callback(self._on_param_change)

        # 串口是独占资源：多个 execute_callback 并发时必须串行化读写
        self._io_lock = threading.RLock()

        port = self.get_parameter('port').value
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.get_parameter('baudrate').value,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.3,
                write_timeout=0.5,
            )
        except serial.SerialException as exc:
            self.get_logger().error(f'打开串口 {port} 失败：{exc}')
            raise

        self.get_logger().info(f'串口已打开：{port}')

        if self.get_parameter('startup_reset').value:
            self._startup_reset()

        # 可重入回调组：保证 execute 执行期间 cancel_callback 能被调用
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, MoveServo, 'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        # /servo/state 记录每次实际下发给串口的指令，供 bag 与 command 对照
        self._state_pub = self.create_publisher(
            ServoCommand, '/servo/state', 10)

        self.create_subscription(
            ServoCommand, '/servo/command', self._on_command, 10)

        self.get_logger().info('servo_driver 已就绪')

    def _on_param_change(self, params):
        for p in params:
            if p.name in ('pwm_min', 'pwm_max'):
                if not (500 <= p.value <= 2500):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{p.name} 必须在 500~2500，收到 {p.value}')
            elif p.name == 'baudrate':
                if p.value not in (9600, 19200, 38400, 57600, 115200):
                    return SetParametersResult(
                        successful=False,
                        reason=f'不支持的波特率 {p.value}')

        new_min = self._pick(params, 'pwm_min')
        new_max = self._pick(params, 'pwm_max')
        if new_min >= new_max:
            return SetParametersResult(
                successful=False,
                reason=f'pwm_min({new_min}) 必须小于 pwm_max({new_max})')

        return SetParametersResult(successful=True)

    def _pick(self, params, name):
        for p in params:
            if p.name == name:
                return p.value
        return self.get_parameter(name).value

    def _send_raw(self, text):
        with self._io_lock:
            self.ser.write(text.encode('ascii'))
            self._drain()
            # 总线舵机需要指令间隔，否则多关节并发时会有指令被吞掉
            time.sleep(0.005)

    def _drain(self, timeout=0.2):
        # 用 out_waiting 轮询代替 flush()：tcdrain 在部分 CH340 上会永久阻塞
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.ser.out_waiting == 0:
                return
            time.sleep(0.002)

    def _read_reply(self, timeout=0.8):
        buf = b''
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf += chunk
                if b'!' in buf:
                    break
        return buf.decode('ascii', errors='ignore')

    def read_pwm(self, servo_id):
        # 整段事务加锁：清缓冲 → 发 PRAD → 读回包，中间不允许别的线程插入
        with self._io_lock:
            self.ser.reset_input_buffer()
            self._send_raw(f'#{servo_id:03d}PRAD!')
            reply = self._read_reply()
        numbers = re.findall(r'\d+', reply)
        if len(numbers) < 2:
            self.get_logger().warn(f'读 {servo_id} 号位置失败，回包={reply!r}')
            return None
        return int(numbers[-1])

    def move(self, servo_id, pwm, duration_ms):
        self._send_raw(build_move_cmd(servo_id, pwm, duration_ms))
        msg = ServoCommand()
        msg.id = int(servo_id)
        msg.pwm = int(pwm)
        msg.duration_ms = int(duration_ms)
        self._state_pub.publish(msg)

    def _startup_reset(self):
        ids = self.get_parameter('servo_ids').value
        for sid in ids:
            self._send_raw(f'#{sid:03d}PULR!')
            time.sleep(0.05)
            self._send_raw(f'#{sid:03d}P1500T1000!')
        self.get_logger().info(f'已回中位：{list(ids)}')

    def goal_callback(self, goal_request):
        servo_id = goal_request.id
        pwm = goal_request.pwm

        ids = self.get_parameter('servo_ids').value
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value

        if servo_id not in ids:
            self.get_logger().warn(f'拒绝：servo_ids 中没有 {servo_id} 号')
            return GoalResponse.REJECT

        if pwm < pwm_min or pwm > pwm_max:
            self.get_logger().warn(
                f'拒绝：pwm {pwm} 超出安全区间 {pwm_min}~{pwm_max}')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info(f'收到取消请求：{goal_handle.request.id} 号')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # 兜底：串口异常时把原因写进 message，避免 Result 全空无从判断
        try:
            return self._execute_goal(goal_handle)
        except Exception as exc:
            self.get_logger().error(f'执行异常：{exc}')
            goal_handle.abort()
            return MoveServo.Result(
                success=False, final_pwm=0, message=f'执行异常：{exc}')

    def _execute_goal(self, goal_handle):
        req = goal_handle.request
        servo_id = req.id
        target = req.pwm
        duration = int(req.duration_ms)
        if duration <= 0:
            duration = self.get_parameter('default_duration_ms').value

        self.get_logger().info(f'执行：{servo_id} 号 -> {target}，{duration} ms')
        self.move(servo_id, target, duration)
        start = time.time()

        # 150ms 一轮：feedback 保持 5Hz 以上，取消延迟不超过 150ms
        while True:
            if goal_handle.is_cancel_requested:
                self.move(servo_id, 1500, 500)
                goal_handle.canceled()
                self.get_logger().warn('已取消，回中位')
                return MoveServo.Result(
                    success=False, final_pwm=1500, message='canceled')

            elapsed = (time.time() - start) * 1000.0
            if elapsed >= duration:
                break

            fb = MoveServo.Feedback()
            fb.current_pwm = int(target)
            fb.progress = min(elapsed / duration, 1.0)
            goal_handle.publish_feedback(fb)
            time.sleep(0.15)

        actual = self.read_pwm(servo_id)
        if actual is None:
            goal_handle.abort()
            return MoveServo.Result(
                success=False, final_pwm=0, message='PRAD 读回失败')

        if abs(actual - target) > 30:
            goal_handle.abort()
            return MoveServo.Result(
                success=False, final_pwm=actual,
                message=f'未到位：目标 {target} 实际 {actual}')

        goal_handle.succeed()
        self.get_logger().info(f'完成：{servo_id} 号停在 {actual}')
        return MoveServo.Result(
            success=True, final_pwm=actual, message='ok')

    def _on_command(self, msg):
        # 与 Action 共用 move()，保证 bag 回放与录屏一致
        self.move(msg.id, msg.pwm, msg.duration_ms)


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    # 线程数要够：动作组可能同时下发 10 个 goal
    executor = MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
