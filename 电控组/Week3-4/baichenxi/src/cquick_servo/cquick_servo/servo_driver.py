import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
import serial
import time
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1,2,3,4,5,6])
        self.declare_parameter('startup_reset', True)

        self.add_on_set_parameters_callback(self.param_cb)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Serial opened on {port}')
        except Exception as e:
            self.get_logger().error(f'Serial open failed: {e}')
            self.ser = None

        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self.state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)

        self.cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, MoveServo, 'move_servo',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group
        )

    def param_cb(self, params):
        for p in params:
            if p.name == 'pwm_max' and not (500 <= p.value <= 2500):
                return SetParametersResult(successful=False, reason='pwm_max out of range 500-2500')
            if p.name == 'pwm_min' and not (500 <= p.value <= 2500):
                return SetParametersResult(successful=False, reason='pwm_min out of range 500-2500')
        pmin = self.get_parameter('pwm_min').value
        pmax = self.get_parameter('pwm_max').value
        for p in params:
            if p.name == 'pwm_max': pmax = p.value
            if p.name == 'pwm_min': pmin = p.value
        if pmin >= pmax:
            return SetParametersResult(successful=False, reason='pwm_min must be < pwm_max')
        return SetParametersResult(successful=True)

    def goal_cb(self, goal_request):
        cmd = goal_request.command
        pmin = self.get_parameter('pwm_min').value
        pmax = self.get_parameter('pwm_max').value
        ids = self.get_parameter('servo_ids').value
        if cmd.pwm < pmin or cmd.pwm > pmax:
            self.get_logger().warn(f'Rejected: pwm {cmd.pwm} out of {pmin}-{pmax}')
            return GoalResponse.REJECT
        if cmd.id not in ids:
            self.get_logger().warn(f'Rejected: id {cmd.id} not in servo_ids')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        cmd = goal_handle.request.command
        result = MoveServo.Result()
        feedback = MoveServo.Feedback()
        
        mock_mode = False
        if self.ser is None or not self.ser.is_open:
            mock_mode = True
            self.get_logger().warn('串口未开启，进入模拟演示模式...')
        else:
            self.ser.write(f'#{cmd.id:03d}PULR!'.encode('ascii'))
            self.ser.flush()
            cmd_str = f'#{cmd.id:03d}P{cmd.pwm}T{cmd.duration_ms}!'
            self.ser.write(cmd_str.encode('ascii'))
            self.ser.flush()
        
        msg = ServoCommand()
        msg.id = cmd.id
        msg.pwm = cmd.pwm
        msg.duration_ms = cmd.duration_ms
        self.cmd_pub.publish(msg)

        start_time = self.get_clock().now()
        duration_sec = cmd.duration_ms / 1000.0
        rate = self.create_rate(10) 
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                if not mock_mode:
                    self.ser.write(f'#{cmd.id:03d}PULR!'.encode('ascii'))
                    self.ser.write(f'#{cmd.id:03d}P1500T1000!'.encode('ascii'))
                goal_handle.canceled()
                result.success = False
                result.message = 'canceled'
                return result

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= duration_sec:
                break
            
            feedback.current_pwm = cmd.pwm 
            feedback.progress = float(elapsed / duration_sec)
            goal_handle.publish_feedback(feedback)
            
            self.state_pub.publish(msg)
            
            rate.sleep()

        result.final_pwm = cmd.pwm
        result.success = True
        result.message = 'Success'
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
