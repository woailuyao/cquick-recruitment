import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult

import serial
import time
import re

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')
        
        # 1. 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)

        # 2. 注册参数设置回调，拒绝非法修改
        self.add_on_set_parameters_callback(self.param_callback)

        # 3. 初始化串口
        self.ser = None
        self.connect_serial()

        # 4. 创建回调组（允许并发，防阻塞）
        self.cb_group = ReentrantCallbackGroup()

        # 5. 启动 Action Server
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # 6. 发布者与订阅者（解决 bag 录制和回放的核心）
        self.state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        
        # 新增发布者，让 Action 把指令发给 bag 录制
        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        
        # 订阅 /servo/command，用于 bag 回放
        self.cmd_sub = self.create_subscription(
            ServoCommand,
            '/servo/command',
            self.cmd_callback,
            10,
            callback_group=self.cb_group
        )
        
        # 状态心跳定时器
        self.create_timer(0.5, self.publish_state, callback_group=self.cb_group)

        # 7. 启动复位 (恢复扭力)
        if self.get_parameter('startup_reset').value:
            self.get_logger().info("启动复位：正在向所有舵机发送 PULR 恢复扭力...")
            ids = self.get_parameter('servo_ids').value
            for sid in ids:
                self.send_raw_cmd(f"#{sid:03d}PULR!")
            time.sleep(0.1)

        self.get_logger().info("Servo Driver 启动完成！")

    def connect_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"成功打开串口 {port}，波特率 {baud}")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            self.ser = None

    def param_callback(self, params):
        for p in params:
            if p.name == 'pwm_min':
                if p.value < 500 or p.value > 2500:
                    return SetParametersResult(successful=False, reason='pwm_min 超出 500~2500')
                if p.value >= self.get_parameter('pwm_max').value:
                    return SetParametersResult(successful=False, reason='pwm_min 不能大于等于 pwm_max')
            elif p.name == 'pwm_max':
                if p.value < 500 or p.value > 2500:
                    return SetParametersResult(successful=False, reason='pwm_max 超出 500~2500')
                if p.value <= self.get_parameter('pwm_min').value:
                    return SetParametersResult(successful=False, reason='pwm_max 不能小于等于 pwm_min')
            elif p.name == 'port':
                self.connect_serial() 
        return SetParametersResult(successful=True)

    def send_raw_cmd(self, cmd_str):
        """底层发送函数:Action 和 话题 共用"""
        if self.ser and self.ser.is_open:
            self.ser.write(cmd_str.encode('ascii'))
            self.ser.flush()
            return True
        return False

    def read_position(self, sid):
        """底层读取函数：防崩溃核心，提取纯数字"""
        if not self.ser or not self.ser.is_open:
            return 1500
        
        try:
            self.ser.reset_input_buffer()
            self.send_raw_cmd(f"#{sid:03d}PRAD!")
            time.sleep(0.05)
            raw_data = self.ser.read_all()
            
            text = raw_data.decode('utf-8', errors='ignore')
            nums = re.findall(r'\d+', text)
            if nums:
                return int(nums[-1])
        except Exception as e:
            self.get_logger().warn(f"读取位置异常: {e}")
            
        return 1500

    def goal_callback(self, goal_request):
        sid = goal_request.id
        pwm = goal_request.pwm
        
        valid_ids = self.get_parameter('servo_ids').value
        if sid not in valid_ids:
            self.get_logger().warn(f"Goal 被拒绝：ID {sid} 不在 servo_ids {valid_ids} 中")
            return GoalResponse.REJECT

        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        if pwm < pwm_min or pwm > pwm_max:
            self.get_logger().warn(f"Goal 被拒绝：目标 {pwm} 超出安全范围 [{pwm_min}, {pwm_max}]")
            return GoalResponse.REJECT

        self.get_logger().info(f"Goal 接受：ID={sid}, 目标={pwm}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求...')
        return CancelResponse.ACCEPT

    def cmd_callback(self, msg):
        """调试话题回调（bag 回放时触发）"""
        duration = msg.duration_ms if msg.duration_ms > 0 else self.get_parameter('default_duration_ms').value
        cmd = f"#{msg.id:03d}P{msg.pwm}T{duration}!"
        self.get_logger().info(f"[话题驱动/bag回放] 发送: {cmd}")
        self.send_raw_cmd(cmd)

    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        sid = req.id
        target_pwm = req.pwm
        duration = req.duration_ms if req.duration_ms > 0 else self.get_parameter('default_duration_ms').value
        
        self.send_raw_cmd(f"#{sid:03d}PULR!")
        time.sleep(0.02)

        cmd_str = f"#{sid:03d}P{target_pwm}T{duration}!"
        self.get_logger().info(f"Action 执行发送: {cmd_str}")
        
        # 把指令发布到话题
        cmd_msg = ServoCommand()
        cmd_msg.id = sid
        cmd_msg.pwm = target_pwm
        cmd_msg.duration_ms = duration
        self.cmd_pub.publish(cmd_msg)

        # 发送给底层串口
        if not self.send_raw_cmd(cmd_str):
            goal_handle.abort()
            result = MoveServo.Result()
            result.success = False
            result.message = "串口未打开或发送失败"
            return result

        feedback_msg = MoveServo.Feedback()
        start_time = self.get_clock().now()
        duration_sec = duration / 1000.0
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("动作被取消，正在回中位...")
                self.send_raw_cmd(f"#{sid:03d}P1500T500!")
                goal_handle.canceled()
                result = MoveServo.Result()
                result.success = False
                result.message = "canceled"
                result.final_pwm = int(self.read_position(sid))
                return result

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            progress = min(1.0, elapsed / duration_sec)
            
            # 强制转换为 int 和 float，绝对不报类型错误
            feedback_msg.current_pwm = int(target_pwm)
            feedback_msg.progress = float(progress)
            goal_handle.publish_feedback(feedback_msg)

            if progress >= 1.0:
                break
                
            time.sleep(0.1) # 10Hz

        time.sleep(0.1)
        actual_pwm = self.read_position(sid)
        
        result = MoveServo.Result()
        result.final_pwm = int(actual_pwm)
        
        diff = abs(actual_pwm - target_pwm)
        if diff <= 30:
            result.success = True
            result.message = f"动作完成，误差 {diff}"
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f"未到位，目标 {target_pwm} 实际 {actual_pwm}，差 {diff}"
            goal_handle.abort()
            
        self.get_logger().info(result.message)
        return result

    def publish_state(self):
        msg = ServoCommand()
        msg.id = 1
        msg.pwm = 1500 
        msg.duration_ms = 0
        self.state_pub.publish(msg)

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