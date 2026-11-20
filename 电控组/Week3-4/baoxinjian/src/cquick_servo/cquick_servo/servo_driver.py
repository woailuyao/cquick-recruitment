"""servo_driver.py —— 总线舵机串口驱动节点（Action Server）

数据流：
    Action goal ──> 校验（限位 / ID / duration）
                 ──> 拼 ASCII 指令 "#001P1700T1000!"
                 ──> 串口 write + flush
                 ──> 控制板 ──> 舵机
                 ──> 循环发 feedback（10 Hz）
                 ──> 结束后发 "#001PRAD!" 读回实际位置
                 ──> result（成功 / 未到位 / 被取消）

硬编码红线：设备名、波特率、限位、默认时长、舵机列表全部来自参数，
代码里只保留协议常量（# ! P T PID PRAD PULR PULK 和 3 位补零）。
"""

import time
import threading

import rclpy
import serial
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo.protocol import (
    CENTER_PWM,
    PWM_HARD_MAX,
    PWM_HARD_MIN,
    build_keyword_command,
    build_move_command,
    parse_reply,
)

# ---------- 驱动节点内部的常量 ----------
TOLERANCE_PWM = 30         # PRAD 到位判定阈值
FEEDBACK_PERIOD_S = 0.1    # 10 Hz，要求 >= 5 Hz
REPLY_TIMEOUT_S = 0.3
BAUD_ALLOWED = (9600, 19200, 38400, 57600, 115200, 230400)


class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyUSB0'),
                ('baudrate', 115200),
                ('pwm_min', 900),
                ('pwm_max', 2100),
                ('default_duration_ms', 800),
                ('servo_ids', [1, 2, 3, 4, 5, 6]),
                ('startup_reset', True),
                ('dry_run', False),
            ],
        )

        self._ser = None
        self._io_lock = threading.Lock()
        self._last_pwm = {}

        self._open_serial()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            handle_accepted_callback=self._on_accepted,
            callback_group=group,
        )

        # 内部调试总线：bag 回放靠它，必须和 Action 走同一个底层发送函数
        self.create_subscription(
            ServoCommand, '/servo/command', self._on_command, 10,
            callback_group=group)
        self._state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)

        self.get_logger().info(
            f'节点已启动：port={self.get_parameter("port").value} '
            f'baudrate={self.get_parameter("baudrate").value} '
            f'pwm=[{self.get_parameter("pwm_min").value}, '
            f'{self.get_parameter("pwm_max").value}] '
            f'servo_ids={list(self.get_parameter("servo_ids").value)}')

        if self.get_parameter('startup_reset').value:
            self._reset_timer = self.create_timer(
                1.0, self._startup_reset_once, callback_group=group)

    # ---------------- 串口 ----------------

    def _open_serial(self):
        if self.get_parameter('dry_run').value:
            self._ser = None
            self.get_logger().warn('dry_run=true：不打开串口，只打印将要发送的指令')
            return
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)
        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=REPLY_TIMEOUT_S,
                write_timeout=0.2,
            )
            self.get_logger().info(f'串口已打开：{port} @ {baud} 8-N-1')
        except Exception as exc:  # noqa: BLE001
            self._ser = None
            self.get_logger().error(f'打开串口失败：{exc}')

    def _send(self, text: str) -> bool:
        with self._io_lock:
            if self._ser is None:
                self.get_logger().info(f'[dry_run] TX {text}')
                return True
            try:
                self._ser.write(text.encode('ascii'))
                self._ser.flush()
                return True
            except serial.SerialException as exc:
                self.get_logger().error(f'写串口失败：{exc}')
                return False

    def _transact(self, text: str):
        with self._io_lock:
            if self._ser is None:
                self.get_logger().info(f'[dry_run] TX {text}')
                return None
            try:
                self._ser.reset_input_buffer()
                self._ser.write(text.encode('ascii'))
                self._ser.flush()
                buf = b''
                deadline = time.monotonic() + REPLY_TIMEOUT_S
                while time.monotonic() < deadline:
                    chunk = self._ser.read(64)
                    if chunk:
                        buf += chunk
                        if b'!' in buf:
                            break
                return buf
            except serial.SerialException as exc:
                self.get_logger().error(f'读写串口失败：{exc}')
                return None

    def _read_prad(self, servo_id: int):
        """读回实际 PWM；dry_run 时用最后一次下发的值冒充。"""
        if self._ser is None:
            return self._last_pwm.get(servo_id)
        reply = self._transact(build_keyword_command(servo_id, 'PRAD'))
        parsed = parse_reply(reply)
        return parsed[1] if parsed else None

    def _restore_torque(self, servo_id: int) -> bool:
        """上电后第一次动作前必须先 PULR，否则舵机纹丝不动。"""
        if not self._send(build_keyword_command(servo_id, 'PULR')):
            return False
        time.sleep(0.1)
        return True

    # ---------------- 参数 ----------------

    def _on_set_parameters(self, params):
        names = ('port', 'baudrate', 'pwm_min', 'pwm_max',
                 'default_duration_ms', 'servo_ids')
        target = {name: self.get_parameter(name).value for name in names}
        for param in params:
            if param.name in target:
                target[param.name] = param.value

        for param in params:
            if param.name in ('pwm_min', 'pwm_max'):
                value = int(param.value)
                if not PWM_HARD_MIN <= value <= PWM_HARD_MAX:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name}={value} 超出电气范围 '
                               f'{PWM_HARD_MIN}~{PWM_HARD_MAX}')
            elif param.name == 'baudrate':
                if int(param.value) not in BAUD_ALLOWED:
                    return SetParametersResult(
                        successful=False,
                        reason=f'不支持的波特率 {param.value}，可选 {BAUD_ALLOWED}')
            elif param.name == 'default_duration_ms':
                if not 50 <= int(param.value) <= 10000:
                    return SetParametersResult(
                        successful=False,
                        reason='default_duration_ms 必须在 50~10000 ms')
            elif param.name == 'port':
                if not isinstance(param.value, str) or not param.value:
                    return SetParametersResult(
                        successful=False, reason='port 不能为空')
            elif param.name == 'servo_ids':
                ids = [int(i) for i in param.value]
                if not ids or any(not 1 <= i <= 254 for i in ids):
                    return SetParametersResult(
                        successful=False, reason='servo_ids 必须非空且都在 1~254')

        if int(target['pwm_min']) >= int(target['pwm_max']):
            return SetParametersResult(
                successful=False,
                reason=f"pwm_min({target['pwm_min']}) 必须小于 "
                       f"pwm_max({target['pwm_max']})")

        for param in params:
            if param.name in ('port', 'baudrate'):
                self.get_logger().warn(
                    f'{param.name} 已改为 {param.value}，重启节点后才会重新打开串口')
        return SetParametersResult(successful=True)

    # ---------------- Action 回调 ----------------

    def _on_goal(self, goal_request):
        servo_id = int(goal_request.id)
        pwm = int(goal_request.pwm)
        duration = int(goal_request.duration_ms)

        pwm_min = int(self.get_parameter('pwm_min').value)
        pwm_max = int(self.get_parameter('pwm_max').value)
        servo_ids = [int(i) for i in self.get_parameter('servo_ids').value]

        if servo_id not in servo_ids:
            self.get_logger().warn(
                f'拒绝 goal：id={servo_id} 不在 servo_ids={servo_ids}')
            return GoalResponse.REJECT
        if not pwm_min <= pwm <= pwm_max:
            self.get_logger().warn(
                f'拒绝 goal：pwm={pwm} 超出安全区间 [{pwm_min}, {pwm_max}]')
            return GoalResponse.REJECT
        if duration != 0 and not 50 <= duration <= 10000:
            self.get_logger().warn(
                f'拒绝 goal：duration_ms={duration} 非法（0 表示用默认值，'
                f'其余必须在 50~10000）')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'接受 goal：id={servo_id} pwm={pwm} '
            f'duration_ms={duration or "default"}')
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle):
        goal_handle.execute()

    def _on_cancel(self, goal_handle):
        self.get_logger().warn('收到取消请求')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        request = goal_handle.request
        servo_id = int(request.id)
        pwm = int(request.pwm)
        duration_ms = int(request.duration_ms) or int(
            self.get_parameter('default_duration_ms').value)

        result = MoveServo.Result()

        if not self._restore_torque(servo_id):
            goal_handle.abort()
            result.success = False
            result.final_pwm = 0
            result.message = 'PULR 发送失败，串口异常'
            return result

        start_pwm = self._last_pwm.get(servo_id, CENTER_PWM)
        command = build_move_command(servo_id, pwm, duration_ms)
        if not self._send(command):
            goal_handle.abort()
            result.success = False
            result.final_pwm = 0
            result.message = '写串口失败'
            return result
        self._last_pwm[servo_id] = pwm
        self.get_logger().info(f'下发 {command}')

        total = max(duration_ms / 1000.0, 1e-3)
        start_time = time.monotonic()

        while True:
            now = time.monotonic()
            elapsed = now - start_time

            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f'取消生效：id={servo_id} 回中位')
                self._send(build_move_command(servo_id, CENTER_PWM, 300))
                self._last_pwm[servo_id] = CENTER_PWM
                goal_handle.canceled()
                result.success = False
                result.final_pwm = CENTER_PWM
                result.message = 'canceled'
                return result

            if elapsed >= total:
                break

            fraction = min(1.0, elapsed / total)
            feedback = MoveServo.Feedback()
            feedback.current_pwm = int(round(
                start_pwm + (pwm - start_pwm) * fraction))
            feedback.progress = float(fraction)
            goal_handle.publish_feedback(feedback)

            self._state_pub.publish(ServoCommand(
                id=servo_id, pwm=feedback.current_pwm, duration_ms=0))

            time.sleep(FEEDBACK_PERIOD_S)

        actual = self._read_prad(servo_id)
        self._last_pwm[servo_id] = actual if actual is not None else pwm

        if actual is None:
            goal_handle.abort()
            result.success = False
            result.final_pwm = pwm
            result.message = 'PRAD 无回显，无法确认是否到位'
            return result

        diff = actual - pwm
        result.final_pwm = actual
        if abs(diff) <= TOLERANCE_PWM:
            goal_handle.succeed()
            result.success = True
            result.message = f'到位：PRAD={actual}，目标 {pwm}，差 {diff}'
        else:
            goal_handle.abort()
            result.success = False
            result.message = (f'未到位：PRAD={actual}，目标 {pwm}，差 {diff}'
                              f'（阈值 ±{TOLERANCE_PWM}）')
        self.get_logger().info(result.message)
        return result

    # ---------------- 调试总线 / 启动复位 ----------------

    def _on_command(self, msg: ServoCommand):
        duration_ms = int(msg.duration_ms) or int(
            self.get_parameter('default_duration_ms').value)
        self._restore_torque(int(msg.id))
        command = build_move_command(msg.id, msg.pwm, duration_ms)
        if self._send(command):
            self._last_pwm[int(msg.id)] = int(msg.pwm)
            self._state_pub.publish(ServoCommand(
                id=msg.id, pwm=msg.pwm, duration_ms=duration_ms))
            self.get_logger().info(f'[/servo/command] 下发 {command}')

    def _startup_reset_once(self):
        self._reset_timer.cancel()
        for servo_id in [int(i) for i in self.get_parameter('servo_ids').value]:
            self._restore_torque(servo_id)
            command = build_move_command(servo_id, CENTER_PWM, 1000)
            if self._send(command):
                self._last_pwm[servo_id] = CENTER_PWM
                self.get_logger().info(f'启动复位：{command}')
            time.sleep(0.1)

    def close(self):
        """退出前给所有舵机卸力，防止一直顶着力矩发热。"""
        for servo_id in [int(i) for i in self.get_parameter('servo_ids').value]:
            self._send(build_keyword_command(servo_id, 'PULK'))


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
        node.get_logger().info('退出：给所有舵机 PULK 卸力')
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
