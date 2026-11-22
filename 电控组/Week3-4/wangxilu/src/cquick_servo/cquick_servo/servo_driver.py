#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
import serial
import time

from cquick_servo_interfaces.action import MoveServo


def build_move_command(servo_id, pwm, duration_ms):
    return f'#{servo_id:03d}P{pwm}T{duration_ms}!'


def parse_position(response):
    if not response or 'P' not in response:
        return None
    try:
        start = response.index('P') + 1
        end = response.index('!', start)
        return int(response[start:end])
    except (ValueError, IndexError):
        return None


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.serial = serial.Serial(
                port=port, baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0)
            self.get_logger().info(f'串口已打开: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'串口打开失败: {e}')
            self.serial = None

        self.action_server = ActionServer(
            self, MoveServo, 'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup())

        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.get_logger().info('ServoDriver 初始化完成，ActionServer 就绪')

    def send_command(self, cmd):
        if self.serial is None or not self.serial.is_open:
            self.get_logger().error('串口未打开')
            return False
        try:
            self.serial.write(cmd.encode())
            self.serial.flush()
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'串口发送失败: {e}')
            return False

    def read_position(self, servo_id):
        self.send_command(f'#{servo_id:03d}PRAD!')
        time.sleep(0.2)
        try:
            if self.serial.in_waiting:
                resp = self.serial.read(self.serial.in_waiting).decode()
                return parse_position(resp)
        except serial.SerialException:
            pass
        return None

    def goal_callback(self, goal_request):
        pwm = goal_request.command.pwm
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        sid = goal_request.command.id

        if pwm < pwm_min or pwm > pwm_max:
            self.get_logger().warn(
                f'Goal 被拒: pwm={pwm} 超出 [{pwm_min}, {pwm_max}]')
            return GoalResponse.REJECT

        ids = self.get_parameter('servo_ids').value
        if sid not in ids:
            self.get_logger().warn(
                f'Goal 被拒: id={sid} 不在 servo_ids={ids}')
            return GoalResponse.REJECT

        self.get_logger().info(f'Goal 接受: id={sid} pwm={pwm}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        cmd = goal_handle.request.command
        sid = cmd.id
        target_pwm = cmd.pwm
        duration_ms = cmd.duration_ms

        self.get_logger().info(
            f'执行: id={sid} pwm={target_pwm} t={duration_ms}ms')

        if not self.send_command(build_move_command(sid, target_pwm, duration_ms)):
            result = MoveServo.Result()
            result.error_code = MoveServo.Result.ERR_SERIAL
            result.message = '串口发送失败'
            result.success = False
            result.final_pwm = 0
            goal_handle.abort()
            return result

        start_time = self.get_clock().now()
        duration_sec = duration_ms / 1000.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.send_command(f'#{sid:03d}PULK!')
                goal_handle.canceled()
                result = MoveServo.Result()
                result.error_code = MoveServo.Result.ERR_CANCELED
                result.message = '已取消'
                result.success = False
                result.final_pwm = 0
                return result

            self.send_command(f'#{sid:03d}PRAD!')
            time.sleep(0.05)
            current_pwm = None
            try:
                if self.serial.in_waiting:
                    resp = self.serial.read(self.serial.in_waiting).decode()
                    current_pwm = parse_position(resp)
            except serial.SerialException:
                pass
            if current_pwm is None:
                current_pwm = target_pwm

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            progress = min(1.0, elapsed / duration_sec) if duration_sec > 0 else 1.0

            fb = MoveServo.Feedback()
            fb.current_pwm = current_pwm
            fb.progress = float(progress)
            fb.status = (MoveServo.Feedback.STATUS_MOVING
                         if progress < 1.0 else MoveServo.Feedback.STATUS_STOP)
            goal_handle.publish_feedback(fb)

            if progress >= 1.0 and abs(current_pwm - target_pwm) <= 30:
                break

            if elapsed > duration_sec * 1.5 + 0.5:
                self.get_logger().warn('超时未到位')
                result = MoveServo.Result()
                result.error_code = MoveServo.Result.ERR_NOT_REACHED
                result.message = '超时未到位'
                result.success = False
                result.final_pwm = current_pwm
                goal_handle.abort()
                return result

            time.sleep(0.1)

        self.send_command(f'#{sid:03d}PRAD!')
        time.sleep(0.1)
        final_pwm = None
        try:
            if self.serial.in_waiting:
                resp = self.serial.read(self.serial.in_waiting).decode()
                final_pwm = parse_position(resp)
        except serial.SerialException:
            pass
        if final_pwm is None:
            final_pwm = target_pwm

        result = MoveServo.Result()
        result.error_code = MoveServo.Result.ERR_OK
        result.message = '执行成功'
        result.success = True
        result.final_pwm = final_pwm
        goal_handle.succeed()
        return result

    def on_set_parameters(self, params):
        result = SetParametersResult()
        result.successful = True

        pwm_min_new = None
        pwm_max_new = None

        for p in params:
            if p.name == 'pwm_min':
                if not (500 <= p.value <= 2500):
                    result.successful = False
                    result.reason = f'pwm_min={p.value} 超出 [500,2500]'
                    return result
                pwm_min_new = p.value

            elif p.name == 'pwm_max':
                if not (500 <= p.value <= 2500):
                    result.successful = False
                    result.reason = f'pwm_max={p.value} 超出 [500,2500]'
                    return result
                pwm_max_new = p.value

            elif p.name == 'baudrate':
                if p.value not in (9600, 19200, 38400, 57600, 115200):
                    result.successful = False
                    result.reason = f'baudrate={p.value} 不在白名单'
                    return result

            elif p.name == 'port':
                if not isinstance(p.value, str) or not p.value.startswith('/dev/'):
                    result.successful = False
                    result.reason = f'port={p.value} 必须以 /dev/ 开头'
                    return result

        pwm_min = pwm_min_new if pwm_min_new is not None else self.get_parameter('pwm_min').value
        pwm_max = pwm_max_new if pwm_max_new is not None else self.get_parameter('pwm_max').value
        if pwm_min >= pwm_max:
            result.successful = False
            result.reason = f'pwm_min={pwm_min} 必须小于 pwm_max={pwm_max}'
            return result

        return result


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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()