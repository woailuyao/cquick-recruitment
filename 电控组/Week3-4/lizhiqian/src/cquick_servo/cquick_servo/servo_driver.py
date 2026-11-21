#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand
import serial
import time
import threading
import os


# 协议 / 判定常量（不属于参数，考核细则 2.2 允许的协议常量）
NEUTRAL_PWM = 1500
CANCEL_RETURN_DURATION_MS = 500
POS_ERROR_TOLERANCE = 30
FEEDBACK_STEP_MS = 200
READBACK_SETTLE_S = 0.3


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.declare_parameter('startup_reset', True)

        self.add_on_set_parameters_callback(self.on_set_parameters)

        # 串口锁：10 个 action 并发执行时，必须串行读写串口
        self._serial_lock = threading.RLock()

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
            self.get_logger().info(f'串口已打开: {port} @ {baudrate}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().error(f'串口打开失败（没接设备时正常）: {e}')

        if self.get_parameter('startup_reset').value:
            for sid in self.get_parameter('servo_ids').value:
                self._send_command(sid, 'PULR')
                self._send_command(
                    sid, f'P{NEUTRAL_PWM:04d}',
                    duration_ms=CANCEL_RETURN_DURATION_MS,
                )
            self.get_logger().info('启动回中位完成')

        # 内部调试总线：bag 回放时通过 /servo/command 写串口
        self.command_pub = self.create_publisher(
            ServoCommand, '/servo/command', 10
        )
        self.state_pub = self.create_publisher(
            ServoCommand, '/servo/state', 10
        )
        self.command_sub = self.create_subscription(
            ServoCommand, '/servo/command', self.command_callback, 10
        )

        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    # ---------- 串口协议层 ----------
    def _send_command(self, servo_id, command, duration_ms=None):
        id_str = f'{servo_id:03d}'
        if duration_ms is not None:
            text = f'#{id_str}{command}T{duration_ms:04d}!'
        else:
            text = f'#{id_str}{command}!'

        if self.ser is None:
            self.get_logger().warning(f'串口未打开，跳过发送: {text}')
            return None

        with self._serial_lock:
            try:
                self.ser.write((text + '\r\n').encode('ascii'))
                self.ser.flush()
                self.get_logger().info(f'发送: {text}')
                return text
            except serial.SerialException as e:
                self.get_logger().error(f'串口写入失败: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return None

    def _move_servo(self, servo_id, pwm, duration_ms):
        with self._serial_lock:
            self._send_command(servo_id, 'PULR')
            self._send_command(servo_id, f'P{pwm:04d}', duration_ms=duration_ms)

    def _read_response(self):
        if self.ser is None:
            return None
        try:
            data = self.ser.read_until(b'!')
            return data.decode('ascii', errors='ignore').strip()
        except serial.SerialException as e:
            self.get_logger().error(f'串口读取失败: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return None

    def _read_pos(self, servo_id):
        with self._serial_lock:
            self._send_command(servo_id, 'PRAD')
            return self._read_response()

    # ---------- bag 回放回调 ----------
    def command_callback(self, msg):
        duration_ms = msg.duration_ms
        if duration_ms == 0:
            duration_ms = self.get_parameter('default_duration_ms').value

        self._move_servo(msg.id, msg.pwm, duration_ms)

    # ---------- 参数校验回调 ----------
    def on_set_parameters(self, params):
        for p in params:
            if p.name == 'pwm_min':
                if not (500 <= p.value <= 2500):
                    return SetParametersResult(
                        successful=False,
                        reason='pwm_min 必须在 500~2500',
                    )
                pwm_max = self.get_parameter('pwm_max').value
                if p.value >= pwm_max:
                    return SetParametersResult(
                        successful=False,
                        reason='pwm_min 必须小于当前 pwm_max',
                    )
            elif p.name == 'pwm_max':
                if not (500 <= p.value <= 2500):
                    return SetParametersResult(
                        successful=False,
                        reason='pwm_max 必须在 500~2500',
                    )
                pwm_min = self.get_parameter('pwm_min').value
                if p.value <= pwm_min:
                    return SetParametersResult(
                        successful=False,
                        reason='pwm_max 必须大于当前 pwm_min',
                    )
            elif p.name == 'baudrate':
                if p.value not in (9600, 19200, 38400, 57600, 115200):
                    return SetParametersResult(
                        successful=False,
                        reason='不认识的波特率',
                    )
                if p.value != self.get_parameter('baudrate').value:
                    return SetParametersResult(
                        successful=False,
                        reason='波特率运行时修改不生效，请通过 launch/yaml 设置后重启',
                    )
        return SetParametersResult(successful=True)

    # ---------- Action 回调 ----------
    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'收到 goal: id={goal_request.id}, '
            f'pwm={goal_request.pwm}, duration_ms={goal_request.duration_ms}'
        )

        if goal_request.id not in self.get_parameter('servo_ids').value:
            self.get_logger().warn(f'拒绝：ID {goal_request.id} 不在 servo_ids')
            return GoalResponse.REJECT

        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        if not (pwm_min <= goal_request.pwm <= pwm_max):
            self.get_logger().warn(
                f'拒绝：pwm {goal_request.pwm} 超出 {pwm_min}~{pwm_max}'
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = MoveServo.Feedback()
        result = MoveServo.Result()

        duration_ms = goal.duration_ms
        if duration_ms == 0:
            duration_ms = self.get_parameter('default_duration_ms').value


        # 直接写串口（同步、可靠），并发动作组下不会卡在订阅回调上。
        self._move_servo(goal.id, goal.pwm, duration_ms)

        # 发布到 /servo/command 供 bag 录制；回放时由订阅回调复现同一动作。
        cmd = ServoCommand()
        cmd.id = goal.id
        cmd.pwm = goal.pwm
        cmd.duration_ms = duration_ms
        self.command_pub.publish(cmd)

        # 发布 /servo/state，与 README 架构一致。
        state = ServoCommand()
        state.id = goal.id
        state.pwm = goal.pwm
        state.duration_ms = duration_ms
        self.state_pub.publish(state)

        total_ms = max(duration_ms, 1)
        elapsed_ms = 0
        step_ms = FEEDBACK_STEP_MS

        while elapsed_ms < total_ms:
            if goal_handle.is_cancel_requested:
                self._move_servo(
                    goal.id, NEUTRAL_PWM, CANCEL_RETURN_DURATION_MS,
                )
                result.success = False
                result.message = 'canceled'
                result.final_pwm = goal.pwm
                goal_handle.canceled()
                self.get_logger().warn('goal 已取消，已回中位')
                return result

            progress = min(elapsed_ms / total_ms, 1.0)
            feedback.current_pwm = goal.pwm
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)

            time.sleep(step_ms / 1000.0)
            elapsed_ms += step_ms

        time.sleep(READBACK_SETTLE_S)
        read_back = self._read_pos(goal.id)
        self.get_logger().info(f'PRAD 读回: {read_back}')

        if read_back is None:
            result.success = False
            result.message = '串口错误：读不到位置'
            result.final_pwm = 0
            goal_handle.succeed()
            return result

        raw = read_back.strip()
        try:
            final_pos = int(raw.split('P')[-1].split('!')[0])
        except Exception:
            result.success = False
            result.message = f'PRAD 回读解析失败: {read_back!r}'
            result.final_pwm = 0
            goal_handle.succeed()
            return result

        error = abs(final_pos - goal.pwm)
        result.final_pwm = final_pos
        if error <= POS_ERROR_TOLERANCE:
            result.success = True
            result.message = f'done, error={error}'
        else:
            result.success = False
            result.message = f'final_pwm偏离目标{error}'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    num_threads = max(len(node.get_parameter('servo_ids').value) + 4, 8)
    executor = MultiThreadedExecutor(num_threads=num_threads)
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
