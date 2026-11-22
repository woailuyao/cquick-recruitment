#!/usr/bin/env python3
import serial
import time
import glob
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rcl_interfaces.msg import SetParametersResult
from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo_interfaces.action import MoveServo

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)

        self.add_on_set_parameters_callback(self.on_param_set)

        self.ser = None
        self._serial_lock = threading.Lock()
        self.open_serial()

        self.command_sub = self.create_subscription(
            ServoCommand, '/servo/command', self.command_callback, 10)
        self.command_pub = self.create_publisher(ServoCommand, '/servo/command', 10)

        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        if self.get_parameter('startup_reset').value:
            self.reset_all_to_center()

        self.get_logger().info('Servo driver started')

    def open_serial(self):
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        if port == 'auto':
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if not ports:
                self.get_logger().error('auto 模式：没找到串口设备')
                self.ser = None
                return
            port = ports[0]
            self.get_logger().info(f'auto 检测到串口：{port}')
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Serial opened: {port} @ {baudrate}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {port}: {e}')
            self.ser = None

    def send_raw(self, cmd):
        with self._serial_lock:
            if self.ser is None:
                self.open_serial()
                if self.ser is None:
                    return ''
            try:
                self.ser.write((cmd + '\r\n').encode())
                self.ser.flush()
                time.sleep(0.1)
                data = self.ser.read(self.ser.in_waiting)
                return data.decode(errors='ignore') if data else ''
            except Exception as e:
                self.get_logger().warn(f'串口写入失败，尝试自动重连：{e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.open_serial()
                return ''

    @staticmethod
    def build_cmd(sid, body):
        return f'#{sid:03d}{body}!'

    def reset_all_to_center(self):
        ids = self.get_parameter('servo_ids').value
        for sid in ids:
            self.send_raw(self.build_cmd(sid, 'PULR'))
            self.send_raw(self.build_cmd(sid, 'P1500T800'))
        self.get_logger().info('All servos reset to center (PULR + 1500)')

    def on_param_set(self, params):
        for p in params:
            if p.name in ('pwm_min', 'pwm_max'):
                if not (500 <= p.value <= 2500):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{p.name} 必须在 500~2500，收到 {p.value}')
            if p.name == 'baudrate':
                if p.value not in (9600, 115200, 57600, 38400, 19200):
                    return SetParametersResult(
                        successful=False,
                        reason=f'不支持的波特率 {p.value}')
        return SetParametersResult(successful=True)

    def goal_callback(self, goal_request):
        sid = goal_request.id
        pwm = goal_request.pwm
        duration = goal_request.duration_ms

        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        servo_ids = self.get_parameter('servo_ids').value

        if sid not in servo_ids:
            self.get_logger().warn(f'REJECT: id {sid} 不在 servo_ids 里')
            return GoalResponse.REJECT
        if not (pwm_min <= pwm <= pwm_max):
            self.get_logger().warn(f'REJECT: pwm {pwm} 超出 [{pwm_min}, {pwm_max}]')
            return GoalResponse.REJECT
        if duration < 0:
            self.get_logger().warn(f'REJECT: duration {duration} 非法')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        sid = req.id
        pwm = req.pwm
        duration = req.duration_ms if req.duration_ms > 0 else self.get_parameter('default_duration_ms').value

        self.get_logger().info(f'开始执行 goal: id={sid} pwm={pwm} duration={duration}')

        feedback = MoveServo.Feedback()
        result = MoveServo.Result()

        mirror = ServoCommand()
        mirror.id = sid
        mirror.pwm = pwm
        mirror.duration_ms = duration
        self.command_pub.publish(mirror)

        steps = int(duration / 200) + 1
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                self.send_raw(self.build_cmd(sid, 'P1500T500'))
                result.success = False
                result.final_pwm = 1500
                result.message = 'canceled'
                goal_handle.canceled()
                return result

            progress = min(1.0, (i + 1) / steps)
            feedback.current_pwm = int(pwm * progress)
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))

        reply = self.send_raw(self.build_cmd(sid, 'PRAD'))
        final_pwm = self._parse_prad(reply, pwm)
        result.final_pwm = final_pwm
        if abs(final_pwm - pwm) > 30:
            self.get_logger().warn(f'未到位：目标 {pwm}，实际 {final_pwm}，自动重发一次')
            self.send_raw(self.build_cmd(sid, 'PULR'))
            self.send_raw(self.build_cmd(sid, f'P{pwm}T{duration}'))
            reply = self.send_raw(self.build_cmd(sid, 'PRAD'))
            final_pwm = self._parse_prad(reply, pwm)
            result.final_pwm = final_pwm
        if abs(final_pwm - pwm) <= 30:
            result.success = True
            result.message = 'ok'
        else:
            result.success = False
            result.message = f'not reached: target {pwm}, actual {final_pwm}'

        self.get_logger().info(f'执行完成: success={result.success}, final_pwm={result.final_pwm}, message={result.message}')
        goal_handle.succeed()
        return result

    def _parse_prad(self, reply, fallback):
        import re
        m = re.search(r'P(\d{3,4})!', reply)
        if m:
            try:
                return int(m.group(1))
            except ValueError:
                return fallback
        return fallback

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _do_command(self, sid, pwm, duration):
        self.get_logger().info(f'执行底层指令: id={sid} pwm={pwm} duration={duration}')
        self.send_raw(self.build_cmd(sid, 'PULR'))
        self.send_raw(self.build_cmd(sid, f'P{pwm}T{duration}'))

    def command_callback(self, msg):
        self._do_command(msg.id, msg.pwm, msg.duration_ms)

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
