import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import serial
import time

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')
        
        self.declare_parameter('port', '/dev/cq_servo')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('servo_ids', [1, 2, 3, 4])
        self.declare_parameter('startup_reset', True)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'串口 {self.port} 打开成功！')
            if self.get_parameter('startup_reset').value:
                for sid in self.get_parameter('servo_ids').value:
                    self.send_cmd(sid, 'PULR')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
            self.ser = None

        self._action_server = ActionServer(
            self, MoveServo, 'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('Action Server [move_servo] 已启动，等待指令...')

        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self.cmd_sub = self.create_subscription(ServoCommand, '/servo/command', self.command_callback, 10)

    def send_cmd(self, servo_id, cmd_type, pwm=0, duration=0):
        if self.ser is None or not self.ser.is_open:
            return False
        if cmd_type == 'MOVE':
            cmd = f'#{servo_id:03d}P{pwm}T{duration}!'
        else:
            cmd = f'#{servo_id:03d}{cmd_type}!'
            
        self.ser.write(cmd.encode('ascii'))
        self.ser.flush()
        return True

    def goal_callback(self, goal_request):
        curr_max = self.get_parameter('pwm_max').value
        curr_min = self.get_parameter('pwm_min').value
        valid_ids = self.get_parameter('servo_ids').value
        
        if goal_request.id not in valid_ids:
            self.get_logger().warn(f'拒绝: 不存在 ID {goal_request.id}')
            return GoalResponse.REJECT
        if goal_request.pwm < curr_min or goal_request.pwm > curr_max:
            self.get_logger().warn(f'拒绝: PWM {goal_request.pwm} 超出限位')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        feedback_msg = MoveServo.Feedback()
        result = MoveServo.Result()

        # 1. 发送指令
        self.send_cmd(req.id, 'MOVE', req.pwm, req.duration_ms)
        self.get_logger().info(f'发送转动指令: id={req.id}, pwm={req.pwm}')
        
        cmd_msg = ServoCommand()
        cmd_msg.id = req.id
        cmd_msg.pwm = req.pwm
        cmd_msg.duration_ms = req.duration_ms
        self.cmd_pub.publish(cmd_msg)

        # 2. 非阻塞轮询等待（替代 time.sleep）
        start_time = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=(req.duration_ms / 1000.0) + 1.0)
        
        # 只要还没超时，就一直循环
        while (self.get_clock().now() - start_time) < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.send_cmd(req.id, 'PULK')
                result.success = False
                result.message = 'canceled'
                return result
            
            # 读位置算进度
            self.send_cmd(req.id, 'PRAD')
            raw_data = self.ser.read(50) if self.ser else b''
            
            current_pwm = req.pwm
            if b'P' in raw_data:
                try:
                    str_data = raw_data.decode('ascii', errors='ignore')
                    start = str_data.find('P') + 1
                    end = str_data.find('!', start)
                    if end > start:
                        current_pwm = int(str_data[start:end])
                except Exception:
                    pass
            
            feedback_msg.current_pwm = current_pwm
            # 简单计算进度
            feedback_msg.progress = min(1.0, (self.get_clock().now() - start_time).nanoseconds / (req.duration_ms * 1e6))
            goal_handle.publish_feedback(feedback_msg)

            # 用短时间的 sleep，而不是长时间的阻塞 sleep
            time.sleep(0.05)

        # 3. 超时即认为动作完成
        goal_handle.succeed()
        result.success = True
        result.final_pwm = current_pwm
        result.message = 'success'
        self.get_logger().info(f'动作执行完成 (PRAD: {current_pwm})')
        return result

    def command_callback(self, msg):
        self.get_logger().info(f'从话题收到调试指令: id={msg.id}, pwm={msg.pwm}')
        self.send_cmd(msg.id, 'MOVE', msg.pwm, msg.duration_ms)

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
