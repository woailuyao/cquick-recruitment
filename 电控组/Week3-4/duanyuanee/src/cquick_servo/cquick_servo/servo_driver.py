import rclpy
from rclpy.node import Node, SetParametersResult
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo_interfaces.action import MoveServo
import serial
import time
import re


class ServoDriver(Node):
    # 允许的波特率集合（用于参数校验：不认识的波特率要拒绝）
    ALLOWED_BAUDS = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}

    def __init__(self):
        super().__init__('servo_driver')

        # 考核要求的七个参数全部来自 yaml / launch，代码里不写死设备名、波特率、限位
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 500)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)
        # 中立位单独参数化（装舵臂前标定到 1500）
        self.declare_parameter('safe_pwm', 1500)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.pwm_min = self.get_parameter('pwm_min').value
        self.pwm_max = self.get_parameter('pwm_max').value
        self.default_duration_ms = self.get_parameter('default_duration_ms').value
        self.ids = list(self.get_parameter('servo_ids').value)
        self.startup_reset = self.get_parameter('startup_reset').value
        self.safe_pwm = self.get_parameter('safe_pwm').value

        self.offline = False
        self.ser = None
        self._open_serial()

        self.cb_group = ReentrantCallbackGroup()

        self.sub_cmd = self.create_subscription(
            ServoCommand, '/servo/command', self.cmd_callback, 10,
            callback_group=self.cb_group
        )
        self.pub_state = self.create_publisher(String, '/servo/state', 10)

        self.action_server = ActionServer(
            self, MoveServo, '/servo/move',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )

        self.add_on_set_parameters_callback(self.param_callback)
        if self.startup_reset:
            self._startup_reset()
        self.get_logger().info('servo_driver ready')

    # ---------- 参数校验回调 ----------
    def param_callback(self, params):
        for p in params:
            if p.name == 'pwm_max':
                if p.value < 500 or p.value > 2500:
                    return SetParametersResult(
                        successful=False,
                        reason=f'pwm_max={p.value} out of [500,2500]'
                    )
                if p.value <= self.get_parameter('pwm_min').value:
                    return SetParametersResult(
                        successful=False,
                        reason=f'pwm_max={p.value} must > pwm_min'
                    )
                self.pwm_max = p.value
            elif p.name == 'pwm_min':
                if p.value < 500 or p.value > 2500:
                    return SetParametersResult(
                        successful=False,
                        reason=f'pwm_min={p.value} out of [500,2500]'
                    )
                if p.value >= self.get_parameter('pwm_max').value:
                    return SetParametersResult(
                        successful=False,
                        reason=f'pwm_min={p.value} must < pwm_max'
                    )
                self.pwm_min = p.value
            elif p.name == 'baudrate':
                if p.value not in self.ALLOWED_BAUDS:
                    return SetParametersResult(
                        successful=False,
                        reason=f'baudrate={p.value} not in {sorted(self.ALLOWED_BAUDS)}'
                    )
                self.baudrate = p.value
                if self.ser is not None:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self._open_serial()
            elif p.name == 'default_duration_ms':
                if p.value <= 0 or p.value > 10000:
                    return SetParametersResult(
                        successful=False,
                        reason=f'default_duration_ms={p.value} out of (0,10000]'
                    )
                self.default_duration_ms = p.value
            elif p.name == 'port':
                self.port = p.value
                if self.ser is not None:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self._open_serial()
            elif p.name == 'servo_ids':
                self.ids = list(p.value)
            elif p.name == 'startup_reset':
                self.startup_reset = p.value
            elif p.name == 'safe_pwm':
                self.safe_pwm = p.value
        return SetParametersResult(successful=True)

    # ---------- 串口 ----------
    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.offline = False
            self.get_logger().info(f'serial open {self.port}@{self.baudrate}')
        except Exception as e:
            self.offline = True
            self.get_logger().warn(f'serial fail, offline mode: {e}')

    def _write(self, s):
        if self.offline or self.ser is None:
            return
        try:
            self.ser.write((s + '\r\n').encode())
            self.ser.flush()
        except Exception as e:
            self.offline = True
            self.get_logger().warn(f'serial fail: {e}')

    def send_torque_on(self, sid):
        self._write(f'#{sid:03d}PULR!')

    def send_torque_off(self, sid):
        self._write(f'#{sid:03d}PULK!')

    def send_pwm(self, sid, pwm, dur):
        self._write(f'#{sid:03d}P{pwm:04d}T{dur:04d}!')

    def read_prad(self, sid):
        """读回舵机当前 PWM 位置，供 Action 完成后做 ±30 到位校验。"""
        if self.offline or self.ser is None:
            return None
        self._write(f'#{sid:03d}PRAD!')
        time.sleep(0.02)
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            m = re.search(r'\d+', line.replace(f'{sid:03d}', ''))
            if m:
                return int(m.group())
        except Exception:
            pass
        return None

    def _startup_reset(self):
        if self.offline:
            return
        for sid in self.ids:
            self.send_torque_on(sid)
            self.send_pwm(sid, self.safe_pwm, self.default_duration_ms)
        self.get_logger().info(f'startup reset done (safe_pwm={self.safe_pwm})')

    # ---------- 话题回调（bag 回放走这里，与 Action 共用 send_pwm） ----------
    def cmd_callback(self, msg):
        if msg.pwm == 0:
            self.send_torque_off(msg.id)
            self.pub_state.publish(String(data=f'cmd {msg.id}->OFF'))
        else:
            self.send_torque_on(msg.id)
            self.send_pwm(msg.id, msg.pwm, msg.duration_ms)
            self.pub_state.publish(String(data=f'cmd {msg.id}->{msg.pwm}'))

    # ---------- Action 回调 ----------
    def goal_callback(self, goal_request):
        req = goal_request.command
        if req.id not in self.ids:
            self.get_logger().warn(f'reject: id {req.id} not in {self.ids}')
            return GoalResponse.REJECT
        if req.pwm < self.pwm_min or req.pwm > self.pwm_max:
            self.get_logger().warn(f'reject: pwm {req.pwm} out of [{self.pwm_min},{self.pwm_max}]')
            return GoalResponse.REJECT
        # duration_ms=0 表示客户端没给，运行时用 default_duration_ms 兜底
        dur = req.duration_ms if req.duration_ms > 0 else self.default_duration_ms
        if dur <= 0 or dur > 10000:
            self.get_logger().warn('reject: bad duration')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _cancel_goal(self, goal_handle, sid):
        """取消：停发 + 回中位 + result(message=canceled)。"""
        self.send_torque_on(sid)                                  # 不发 PULR 舵机不会动
        self.send_pwm(sid, self.safe_pwm, self.default_duration_ms)
        self.pub_state.publish(String(data=f'cmd {sid}->CANCELED->CENTER'))
        goal_handle.canceled()
        result = MoveServo.Result()
        result.success = False
        result.final_pwm = self.safe_pwm
        result.message = 'canceled'
        return result

    def execute_callback(self, goal_handle):
        req = goal_handle.request.command
        sid = req.id
        target_pwm = req.pwm
        dur = req.duration_ms if req.duration_ms > 0 else self.default_duration_ms

        self.get_logger().info(f'exec {sid} -> {target_pwm} / {dur}ms')

        if goal_handle.is_cancel_requested:
            return self._cancel_goal(goal_handle, sid)

        self.send_torque_on(sid)
        self.send_pwm(sid, target_pwm, dur)

        # feedback 循环（≥5Hz，每 200ms 一次）
        steps = max(1, int(dur / 200))
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                return self._cancel_goal(goal_handle, sid)

            progress = min(1.0, (i + 1) / steps)
            feedback = MoveServo.Feedback()
            feedback.current_pwm = int(target_pwm * progress)
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)

        # 动作走完读 PRAD 校验：与目标差超过 ±30 就算没到位
        actual = self.read_prad(sid)
        result = MoveServo.Result()
        if actual is None:
            result.success = False
            result.final_pwm = 0
            result.message = f'PRAD read failed (serial offline), target={target_pwm}'
            goal_handle.abort()
        else:
            diff = abs(actual - target_pwm)
            result.final_pwm = actual
            if diff > 30:
                result.success = False
                result.message = f'not in position: target={target_pwm}, PRAD={actual}, diff={diff}'
                goal_handle.abort()
            else:
                result.success = True
                result.message = f'done, PRAD={actual}, diff={diff}'
                goal_handle.succeed()
        self.get_logger().info(
            f'result id={sid} success={result.success} final_pwm={result.final_pwm} '
            f'msg={result.message}')
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
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
