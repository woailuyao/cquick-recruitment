import time
import threading
import traceback
import serial
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult

from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo_interfaces.action import MoveServo


class ServoDriverNode(Node):
    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.declare_parameter('startup_reset', False)

        self.add_on_set_parameters_callback(self.parameter_callback)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.serial_lock = threading.Lock()
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.2)
            self.get_logger().info(f'成功打开串口: {port} @ {baudrate}')
        except Exception as e:
            self.get_logger().error(f'打开串口失败: {e}')
            self.ser = None

        servo_ids = [int(x) for x in self.get_parameter('servo_ids').value]
        self.current_positions = {sid: 1500 for sid in servo_ids}

        self.state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        if self.ser:
            for sid in servo_ids:
                self._send_raw(f'#{sid:03d}PULR!')
                time.sleep(0.02)
                if self.get_parameter('startup_reset').value:
                    default_dur = int(self.get_parameter('default_duration_ms').value)
                    self._send_move_cmd(sid, 1500, default_dur)

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.cmd_sub = self.create_subscription(
            ServoCommand,
            '/servo/command',
            self.command_topic_callback,
            10,
            callback_group=self.cb_group
        )

    def parameter_callback(self, params):
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value

        for param in params:
            if param.name == 'pwm_min':
                if not (500 <= param.value <= 2500):
                    return SetParametersResult(successful=False, reason='pwm_min 超出 500~2500 范围')
                pwm_min = param.value
            elif param.name == 'pwm_max':
                if not (500 <= param.value <= 2500):
                    return SetParametersResult(successful=False, reason='pwm_max 超出 500~2500 范围')
                pwm_max = param.value
            elif param.name == 'baudrate':
                if param.value not in [9600, 115200]:
                    return SetParametersResult(successful=False, reason='不支持的波特率')

        if pwm_min >= pwm_max:
            return SetParametersResult(successful=False, reason='pwm_min 必须小于 pwm_max')

        return SetParametersResult(successful=True)

    def _send_raw(self, cmd_str):
        if not self.ser or not self.ser.is_open:
            return ''
        with self.serial_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd_str.encode('ascii'))
                self.ser.flush()
                time.sleep(0.06)
                return self.ser.read_all().decode('ascii', errors='ignore').strip()
            except Exception as e:
                self.get_logger().error(f'串口通信异常 ({cmd_str}): {e}')
                return ''

    def _send_move_cmd(self, servo_id, pwm, duration_ms):
        sid = int(servo_id)
        p = int(pwm)
        d = int(duration_ms)
        cmd = f'#{sid:03d}P{p:04d}T{d:04d}!'
        self._send_raw(cmd)
        try:
            msg = ServoCommand()
            msg.id = sid
            msg.pwm = p
            msg.duration_ms = d
            self.state_pub.publish(msg)
        except Exception:
            pass

    def _read_position(self, servo_id):
        sid = int(servo_id)
        for _ in range(3):
            resp = self._send_raw(f'#{sid:03d}PRAD!')
            if 'P' in resp and '!' in resp:
                try:
                    val_str = resp.split('P')[1].split('!')[0]
                    val = int(val_str)
                    if val > 0:
                        return val
                except ValueError:
                    pass
            time.sleep(0.03)
        return 0

    def command_topic_callback(self, msg):
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        if pwm_min <= msg.pwm <= pwm_max:
            self.get_logger().info(f'执行话题指令: id={msg.id}, pwm={msg.pwm}, duration={msg.duration_ms}')
            self._send_move_cmd(msg.id, msg.pwm, msg.duration_ms)
            self.current_positions[int(msg.id)] = int(msg.pwm)

    def goal_callback(self, goal_request):
        servo_ids = [int(x) for x in self.get_parameter('servo_ids').value]
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value

        if int(goal_request.id) not in servo_ids:
            self.get_logger().warn(f'拒绝 Goal: ID {goal_request.id} 不在允许列表 {servo_ids} 中')
            return GoalResponse.REJECT

        if not (pwm_min <= goal_request.pwm <= pwm_max):
            self.get_logger().warn(f'拒绝 Goal: PWM {goal_request.pwm} 超出限位 [{pwm_min}, {pwm_max}]')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求，准备中断动作...')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = MoveServo.Result()
        try:
            goal = goal_handle.request
            sid = int(goal.id)
            target_pwm = int(goal.pwm)
            duration = int(goal.duration_ms) if goal.duration_ms > 0 else int(self.get_parameter('default_duration_ms').value)
            start_pwm = int(self.current_positions.get(sid, 1500))

            self._send_move_cmd(sid, target_pwm, duration)

            step_time = 0.1
            total_steps = max(1, int((duration / 1000.0) / step_time))
            feedback_msg = MoveServo.Feedback()

            for step in range(1, total_steps + 1):
                if goal_handle.is_cancel_requested:
                    default_dur = int(self.get_parameter('default_duration_ms').value)
                    self._send_move_cmd(sid, 1500, default_dur)
                    self.current_positions[sid] = 1500
                    goal_handle.canceled()
                    self.get_logger().info('动作已取消，舵机已回中位')
                    result.success = False
                    result.final_pwm = 1500
                    result.message = 'canceled'
                    return result

                time.sleep(step_time)
                progress = min(1.0, step / total_steps)
                est_pwm = int(start_pwm + (target_pwm - start_pwm) * progress)
                feedback_msg.current_pwm = int(est_pwm)
                feedback_msg.progress = float(progress)
                goal_handle.publish_feedback(feedback_msg)

            self.current_positions[sid] = target_pwm
            actual_pwm = self._read_position(sid)
            result.final_pwm = int(actual_pwm)

            if actual_pwm == 0:
                result.success = False
                result.message = 'PRAD 读回超时或无响应'
            elif abs(actual_pwm - target_pwm) > 30:
                result.success = False
                result.message = f'未到位：目标 {target_pwm}，实际 {actual_pwm}，误差 {abs(actual_pwm - target_pwm)}'
            else:
                result.success = True
                result.message = '执行成功'

            goal_handle.succeed()
            return result

        except Exception as e:
            err_msg = f'{type(e).__name__}: {e}'
            self.get_logger().error(f'execute_callback 异常: {err_msg}\n{traceback.format_exc()}')
            result.success = False
            result.final_pwm = 0
            result.message = err_msg
            try:
                goal_handle.abort()
            except Exception:
                pass
            return result


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    executor = MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
