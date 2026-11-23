#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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
        self.state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.ser = None
        self.init_serial()

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

        self.create_subscription(ServoCommand, '/servo/command', self.command_callback, 10)

        if self.get_parameter('startup_reset').value:
            ids = self.get_parameter('servo_ids').value
            for sid in ids:
                self.send_serial_cmd(f"#{sid:03d}PULR!")
                self.send_serial_cmd(f"#{sid:03d}P1500T1000!")

        self.get_logger().info("Servo Driver initialized.")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'pwm_max':
                if param.value > 2500 or param.value < 500:
                    return SetParametersResult(successful=False, reason="pwm_max 超出 500~2500 限制")
            if param.name == 'pwm_min':
                if param.value < 500 or param.value > 2500:
                    return SetParametersResult(successful=False, reason="pwm_min 超出 500~2500 限制")
        return SetParametersResult(successful=True)

    def init_serial(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Opened serial port {port} at {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

    def send_serial_cmd(self, cmd_str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd_str + '\r\n').encode('ascii'))
            self.ser.flush()

    def goal_callback(self, goal_request):
        pwm = goal_request.pwm
        sid = goal_request.id
        
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        valid_ids = self.get_parameter('servo_ids').value

        if pwm < pwm_min or pwm > pwm_max:
            self.get_logger().warn(f"Goal rejected: PWM {pwm} out of safe range [{pwm_min}, {pwm_max}]")
            return GoalResponse.REJECT
        
        if sid not in valid_ids:
            self.get_logger().warn(f"Goal rejected: ID {sid} not in servo_ids")
            return GoalResponse.REJECT

        self.get_logger().info(f"Goal accepted: ID {sid} -> PWM {pwm}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        sid = goal_handle.request.id
        pwm = goal_handle.request.pwm
        duration = goal_handle.request.duration_ms
        
        if duration <= 0:
            duration = self.get_parameter('default_duration_ms').value

        self.send_serial_cmd(f"#{sid:03d}PULR!")
        self.cmd_pub.publish(ServoCommand(id=sid, pwm=pwm, duration_ms=duration))
        cmd = f"#{sid:03d}P{pwm:04d}T{duration:04d}!"
        self.send_serial_cmd(cmd)

        feedback_msg = MoveServo.Feedback()
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.send_serial_cmd(f"#{sid:03d}PULK!")
                result = MoveServo.Result()
                result.success = False
                result.message = "canceled"
                return result
            
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e6
            progress = min(elapsed / duration, 1.0)
            feedback_msg.progress = float(progress)
            feedback_msg.current_pwm = pwm 
            goal_handle.publish_feedback(feedback_msg)
            self.state_pub.publish(ServoCommand(id=sid, pwm=pwm, duration_ms=0))

            if progress >= 1.0:
                break
            time.sleep(0.05)

        self.send_serial_cmd(f"#{sid:03d}PRAD!")
        time.sleep(0.1)
        actual_pwm = pwm 
        if self.ser and self.ser.in_waiting:
            resp = self.ser.read_all().decode('ascii', errors='ignore')
            self.get_logger().info(f"PRAD response: {resp}")

        goal_handle.succeed()
        result = MoveServo.Result()
        result.success = True
        result.final_pwm = actual_pwm
        result.message = "Goal reached successfully"
        return result

    def command_callback(self, msg):
        self.send_serial_cmd(f"#{msg.id:03d}PULR!")
        self.send_serial_cmd(f"#{msg.id:03d}P{msg.pwm:04d}T{msg.duration_ms:04d}!")

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
