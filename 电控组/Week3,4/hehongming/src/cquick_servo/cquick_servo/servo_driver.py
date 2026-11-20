#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from cquick_servo_interfaces.action import MoveServo
from std_msgs.msg import String
import time

# 尝试导入串口库
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')
        
        # 1. 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [0, 1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)

        # 2. 注册参数回调
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 3. 初始化串口 (带防崩溃机制)
        self.ser = None
        self.open_serial()

        # 4. 创建话题发布者 (给bag录制用)
        self.cmd_pub = self.create_publisher(String, '/servo/command', 10)
        
        # 5. 创建话题订阅者 (核心：所有发往串口的数据统一从这里走，防止双发)
        self.cmd_sub = self.create_subscription(
            String,
            '/servo/command',
            self.command_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )

        # 6. 创建断线重连定时器 (每2秒检查一次) ⚠️ 加分项
        self.reconnect_timer = self.create_timer(2.0, self.reconnect_serial)

        # 7. 创建 Action Server
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('Action Server 已启动！')

    def open_serial(self):
        """尝试打开串口，提取为独立函数供初始化和重连使用"""
        if HAS_SERIAL:
            port = self.get_parameter('port').value
            try:
                self.ser = serial.Serial(
                    port=port, 
                    baudrate=self.get_parameter('baudrate').value, 
                    timeout=0.1,
                    rtscts=False, 
                    dsrdtr=False
                )
                self.get_logger().info(f'✅ 成功打开串口: {port}')
            except Exception as e:
                self.get_logger().warn(f'⚠️ 无法打开串口 {port}，进入模拟模式。错误: {e}')
                self.ser = None
        else:
            self.get_logger().warn('未安装 pyserial，进入纯软件模拟模式')

    def reconnect_serial(self):
        """断线重连逻辑"""
        if self.ser is None and HAS_SERIAL:
            port = self.get_parameter('port').value
            try:
                self.ser = serial.Serial(
                    port=port, baudrate=self.get_parameter('baudrate').value, 
                    timeout=0.1, rtscts=False, dsrdtr=False
                )
                self.get_logger().info('🔄 串口掉线重连成功！')
            except Exception:
                pass # 连不上就静默等待下一次重试，不刷屏

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'pwm_max':
                if param.value > 2500 or param.value < 500:
                    return SetParametersResult(successful=False, reason="pwm_max 超出 500~2500")
                if param.value <= self.get_parameter('pwm_min').value:
                    return SetParametersResult(successful=False, reason="pwm_max 必须大于 pwm_min")
        return SetParametersResult(successful=True)

    def send_command(self, servo_id, pwm, duration_ms):
        """只负责拼接指令并发布到话题（绝不直接写串口，防止双发）"""
        cmd = f"#{servo_id:03d}P{pwm}T{duration_ms}!\r\n"
        
        # 发布到话题（这会触发 command_callback 去真正写串口，同时也能被 bag 录下来）
        msg = String()
        msg.data = cmd.strip()
        self.cmd_pub.publish(msg)
        
        self.get_logger().info(f'[Action] 发布指令 -> {cmd.strip()}')
        return cmd

    def command_callback(self, msg):
        """所有写入物理串口的操作，统一在这里进行"""
        self.get_logger().info(f'[物理串口] 收到指令: {msg.data}')
        cmd = msg.data + "\r\n"
        
        if self.ser:
            try:
                self.ser.write(cmd.encode('ascii'))
                self.ser.flush()
            except Exception as e:
                self.get_logger().error(f'❌ 串口写入失败（线可能被拔了）: {e}')
                self.ser = None # 标记掉线，触发重连定时器
        else:
            self.get_logger().warn('[物理串口] 未连接串口，无法发送')

    def goal_callback(self, goal_request):
        pwm = goal_request.pwm
        servo_id = goal_request.id
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        servo_ids = self.get_parameter('servo_ids').value

        if pwm < pwm_min or pwm > pwm_max:
            self.get_logger().warn(f'拒绝目标: PWM {pwm} 超出安全范围 [{pwm_min}, {pwm_max}]')
            return GoalResponse.REJECT
        if servo_id not in servo_ids:
            self.get_logger().warn(f'拒绝目标: 舵机 ID {servo_id} 不在配置列表中')
            return GoalResponse.REJECT

        self.get_logger().info(f'接收目标: ID={servo_id}, PWM={pwm}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求！')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('开始执行动作...')
        try:
            feedback_msg = MoveServo.Feedback()
            result = MoveServo.Result()
            
            target_pwm = goal_handle.request.pwm
            duration = goal_handle.request.duration_ms
            if duration <= 0:
                duration = self.get_parameter('default_duration_ms').value
                
            # 发送指令
            self.send_command(goal_handle.request.id, target_pwm, duration)

            steps = 10
            for i in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'canceled'
                    self.get_logger().info('动作已被取消！')
                    
                    # 取消时物理卸力（直接写串口，不走话题，确保立刻停止）
                    cancel_cmd = f"#{goal_handle.request.id:03d}PULK!\r\n"
                    if self.ser:
                        try:
                            self.ser.write(cancel_cmd.encode('ascii'))
                            self.ser.flush()
                            self.get_logger().info(f'已发送物理停止指令 -> {cancel_cmd.strip()}')
                        except Exception:
                            self.ser = None
                    
                    return result
                
                time.sleep(duration / 1000.0 / steps)
                
                feedback_msg.progress = float(i) / steps
                feedback_msg.current_pwm = int(target_pwm * (i / steps))
                goal_handle.publish_feedback(feedback_msg)
                
            goal_handle.succeed()
            result.success = True
            result.final_pwm = target_pwm
            result.message = 'Success'
            self.get_logger().info('动作执行完毕！')
            return result

        except Exception as e:
            self.get_logger().error(f'执行中发生致命错误: {e}')
            goal_handle.abort()
            result = MoveServo.Result()
            result.success = False
            result.message = 'error'
            return result

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # ⚠️ 加分项：捕获 Ctrl+C，退出前发送全部卸力
        node.get_logger().info('正在关闭节点，发送全局卸力指令...')
        if node.ser:
            try:
                # 直接底层发送广播卸力
                node.ser.write(b"#000PULK!\r\n")
                node.ser.flush()
            except Exception:
                pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()