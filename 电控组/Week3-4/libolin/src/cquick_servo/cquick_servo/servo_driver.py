#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import serial
import time
import threading
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

class ServoDriver(Node):
    def cmd_topic_cb(self, msg):
        """bag回放回调：收到话题消息直接发送舵机指令"""
        self.get_logger().info(f"【bag回放触发】收到指令 id={msg.id}, pwm={msg.pwm}, duration={msg.duration_ms}")
        with self.serial_lock:
            self.send_servo_cmd(msg.id, msg.pwm, msg.duration_ms)

    def __init__(self):
        super().__init__('servo_driver')
        self.cb_group = ReentrantCallbackGroup()
        self.serial_lock = threading.Lock() # 串口互斥锁

        # 【参数声明】全部参数化，禁止硬编码
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("pwm_min", 900)
        self.declare_parameter("pwm_max", 2100)
        self.declare_parameter("default_duration_ms", 800)
        self.declare_parameter("servo_ids", [1,2,3,4,5,6])
        self.declare_parameter("startup_reset", True)

        # 注册参数校验回调（实现2.6：参数非法拒绝）
        self.add_on_set_parameters_callback(self.param_validate_callback)

        # 读取初始参数
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.pwm_min = self.get_parameter("pwm_min").value
        self.pwm_max = self.get_parameter("pwm_max").value
        self.default_duration = self.get_parameter("default_duration_ms").value
        self.servo_ids = self.get_parameter("servo_ids").value
        self.startup_reset = self.get_parameter("startup_reset").value

        # 串口
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"串口打开成功 {self.port}")
            # 上电全局恢复扭力 #000PULR!
            with self.serial_lock:
                cmd = f"#000PULR!"
                self.ser.write(cmd.encode())
                self.get_logger().info(f"send global cmd: {cmd}")
                time.sleep(0.2)
            if self.startup_reset:
                self.send_servo_cmd(1, 1500, self.default_duration)
        except Exception as e:
            self.get_logger().error(f"串口打开失败 {e}")

        # 内部话题 /servo/command 用于bag录制（2.7要求）
        self.cmd_pub = self.create_publisher(ServoCommand, "/servo/command", 10)
        self.cmd_sub = self.create_subscription(ServoCommand,"/servo/command",self.cmd_topic_cb,10)
        # Action Server，四个回调
        self._action_server = ActionServer(
            self,
            MoveServo,
            "move_servo",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info("Action服务端启动完成")

    # ========== 参数校验回调（2.6核心） ==========
    def param_validate_callback(self, params: list[Parameter]):
        res = SetParametersResult()
        res.successful = True
        for p in params:
            if p.name == "pwm_max":
                val = p.value
                if val < 500 or val > 2500:
                    self.get_logger().error(f"pwm_max {val} 超出范围500~2500")
                    res.successful = False
                    res.reason = "pwm_max 超出范围500~2500"
                if val <= self.get_parameter("pwm_min").value:
                    self.get_logger().error("pwm_max不能小于等于pwm_min")
                    res.successful = False
                    res.reason = "pwm_max不能小于等于pwm_min"
            if p.name == "pwm_min":
                val = p.value
                if val < 500 or val > 2500:
                    self.get_logger().error(f"pwm_min {val} 超出范围500~2500")
                    res.successful = False
                    res.reason = "pwm_min 超出范围500~2500"
                if val >= self.get_parameter("pwm_max").value:
                    self.get_logger().error("pwm_min不能大于等于pwm_max")
                    res.successful = False
                    res.reason = "pwm_min不能大于等于pwm_max"
            if p.name == "baudrate":
                val = p.value
                if val not in [9600,115200]:
                    self.get_logger().error("波特率非法")
                    res.successful = False
                    res.reason = "波特率只支持9600、115200"
        return res


    # ========== Goal回调：提前校验，拒绝非法指令 ==========
    def goal_callback(self, goal_request):
        pwm_min = self.get_parameter("pwm_min").value
        pwm_max = self.get_parameter("pwm_max").value
        servo_ids = self.get_parameter("servo_ids").value
        if goal_request.id not in servo_ids:
            self.get_logger().error(f"ID {goal_request.id} 不在允许列表")
            return GoalResponse.REJECT
        if goal_request.pwm < pwm_min or goal_request.pwm > pwm_max:
            self.get_logger().error(f"PWM {goal_request.pwm} 超限 [{pwm_min},{pwm_max}]")
            return GoalResponse.REJECT
        if goal_request.duration_ms <=0:
            self.get_logger().error("时长非法")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ========== Cancel回调 ==========
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("收到取消指令")
        return CancelResponse.ACCEPT

    # ========== Execute回调 ==========
    def execute_callback(self, goal_handle: ServerGoalHandle):
        goal = goal_handle.request
        sid = goal.id
        pwm_target = goal.pwm
        duration = goal.duration_ms if goal.duration_ms>0 else self.get_parameter("default_duration_ms").value
        last_valid_pwm = 1500
        with self.serial_lock:
            self.send_servo_cmd(sid, pwm_target, duration)

        # 发布话题给bag录制
        msg = ServoCommand()
        msg.id = sid
        msg.pwm = pwm_target
        msg.duration_ms = duration
        self.cmd_pub.publish(msg)

        time.sleep(0.08)
        
        feedback_msg = MoveServo.Feedback()
        start_time = time.time()
        total_s = duration / 1000.0

        # 循环持续推送feedback，固定200ms间隔
        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("canceled")
                with self.serial_lock:
                    self.send_servo_cmd(sid, 1500, 200)
                result = MoveServo.Result()
                result.success = False
                result.message = "canceled"
                result.final_pwm = 1500
                goal_handle.canceled(result)
                self.get_logger().info("动作取消，回中")
                return result
                
            elapsed = time.time() - start_time
            progress = min(elapsed / total_s, 1.0)
            if progress >= 1.0:
                break
                
            # 每次反馈前读取舵机当前真实位置
            real_pwm = 1500
            with self.serial_lock:
                self.ser.write(f"#{sid:03d}PRAD!".encode())
                time.sleep(0.03)
                resp = self.ser.read(16)
            try:
                resp_str = resp.decode('ascii')
                if resp_str.startswith('#') and '!' in resp_str:
                    inner = resp_str.strip("#!")
                    p_str = inner.split("P")[-1]
                    real_pwm = int(p_str)
                    last_valid_pwm = real_pwm
            except Exception:
                real_pwm = last_valid_pwm

            feedback_msg.current_pwm = real_pwm
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"feedback: 舵机id={sid}, current_pwm={real_pwm}, progress={progress:.2f}")


            if progress >= 1.0:
                break
            # 分段等待200ms，每50ms检查一次取消请求，取消响应无延迟
            for _ in range(4):
                if goal_handle.is_cancel_requested:
                    break
                time.sleep(0.05)


        # ========== PRAD读取，用于最终到位校验 ==========
        real_pwm = 0
        with self.serial_lock:
            self.ser.write(f"#{sid:03d}PRAD!".encode())
            time.sleep(0.15)
            resp = self.ser.read(16)
        self.get_logger().info(f"PRAD原始接收: {resp}")
        try:
            resp_str = resp.decode('ascii')
            if resp_str.startswith('#') and '!' in resp_str:
                inner = resp_str.strip("#!")
                p_str = inner.split("P")[-1]
                real_pwm = int(p_str)
            else:
                raise ValueError("应答格式错误")
        except Exception as e:
            self.get_logger().warn(f"舵机{sid}PRAD读取失败: {e}, raw={resp}")
            real_pwm = 0

        diff = abs(real_pwm - pwm_target)
        result = MoveServo.Result()
        if real_pwm == 0:
            result.success = False
            result.message = "无应答，位置读取失败"
        elif diff > 30:
            result.success = False
            result.message = f"未到位，差值{diff}"
        else:
            result.success = True
            result.message = "ok"
        result.final_pwm = real_pwm

        goal_handle.succeed()
        self.get_logger().info(f"返回result: success={result.success}, msg={result.message}, final_pwm={result.final_pwm}")
        return result


    # 底层发送函数，严格按照灵众协议：3位ID，4位PWM，4位时间
    def send_servo_cmd(self, sid, pwm, dur):
        cmd = f"#{sid:03d}P{pwm:04d}T{dur:04d}!"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"send: {cmd.strip()}")

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
