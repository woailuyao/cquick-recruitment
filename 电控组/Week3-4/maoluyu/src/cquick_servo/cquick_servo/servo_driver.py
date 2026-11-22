import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo_interfaces.action import MoveServo

import serial
import time
import asyncio

# =========协议常量允许硬编码============
FRAME_HEAD = '#'
FRAME_END = '!'
CMD_TORQUE_ON = "PULR"
CMD_TORQUE_OFF = "PULK"
CMD_READ_POS = "PRAD"
CMD_MOVE = "P"
CMD_TIME = "T"

class ServoDriverNode(Node):
    def __init__(self):
        super().__init__("servo_driver")

        # 声明全部参数，禁止硬编码
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("pwm_min", 900)
        self.declare_parameter("pwm_max", 2100)
        self.declare_parameter("default_duration_ms", 800)
        self.declare_parameter("servo_ids", [1,2,3,4,5,6])
        self.declare_parameter("startup_reset", True)

        # 参数修改回调：动态校验参数
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.pwm_min = self.get_parameter("pwm_min").value
        self.pwm_max = self.get_parameter("pwm_max").value
        self.default_duration_ms = self.get_parameter("default_duration_ms").value
        self.servo_ids = self.get_parameter("servo_ids").value
        self.startup_reset = self.get_parameter("startup_reset").value

        self.ser = None
        self.open_serial()

        # Action服务
        self._action_server = ActionServer(
            self,
            MoveServo,
            "move_servo",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        # 发布/servo/command 用于bag录制
        self.pub_command = self.create_publisher(ServoCommand, "/servo/command", 10)

        # bag回放话题订阅，复用发送逻辑
        self._sub_command = self.create_subscription(
            ServoCommand,
            "/servo/command",
            self.bag_command_cb,
            10
        )

        if self.startup_reset:
            for sid in self.servo_ids:
                self.send_raw_cmd(f"{FRAME_HEAD}{sid:03d}{CMD_TORQUE_ON}{FRAME_END}")
                self.send_move_cmd(sid, 1500, self.default_duration_ms)
                time.sleep(0.2)

    def parameters_callback(self, params):
        """动态参数校验回调，非法参数拒绝设置"""
        for param in params:
            if param.name == "pwm_max":
                val = param.value
                if not (500 <= val <= 2500):
                    return SetParametersResult(successful=False, reason="pwm_max must between 500~2500")
            if param.name == "pwm_min":
                val = param.value
                if not (500 <= val <=2500):
                    return SetParametersResult(successful=False, reason="pwm_min must between 500~2500")
        # 校验min<max
        new_min = self.get_parameter("pwm_min").value
        new_max = self.get_parameter("pwm_max").value
        if new_min >= new_max:
            return SetParametersResult(successful=False, reason="pwm_min can not >= pwm_max")
        return SetParametersResult(successful=True)

    def open_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8, stopbits=1, parity='N', timeout=0.2
            )
            self.get_logger().info(f"serial open success port={self.port}")
        except Exception as e:
            self.get_logger().error(f"serial open failed {e}")

    def send_raw_cmd(self, cmd:str):
        if self.ser is None or not self.ser.is_open:
            return
        self.ser.write(cmd.encode())
        self.ser.flush()
        self.get_logger().debug(f"send: {cmd}")

    def send_move_cmd(self, servo_id:int, pwm:int, duration_ms:int):
        """组装运动指令 #001P1700T1000! """
        cmd_str = f"{FRAME_HEAD}{servo_id:03d}{CMD_MOVE}{pwm}{CMD_TIME}{duration_ms}{FRAME_END}"
        self.send_raw_cmd(cmd_str)

    def bag_command_cb(self, msg:ServoCommand):
        """bag回放话题回调，复用底层发送函数"""
        self.send_move_cmd(msg.id, msg.pwm, msg.duration_ms)

    def read_servo_position(self, servo_id: int):
        if self.ser is None:
            return None
        self.send_raw_cmd(f"{FRAME_HEAD}{servo_id:03d}{CMD_READ_POS}{FRAME_END}")
        time.sleep(0.2)
        raw = self.ser.read_all().decode("ascii", errors="ignore").strip()
        self.get_logger().info(f"PRAD 原始回包: '{raw}'")
        try:
            import re
            nums = re.findall(r"\d+", raw)
            return int(nums[-1]) if nums else None
        except Exception as e:
            self.get_logger().warn(f"位置解析失败: {e}")
            return None

    def goal_callback(self, goal_request):
        """goal到达，校验：ID是否合法、pwm是否超限；不允许阻塞"""
        sid = goal_request.id
        target_pwm = goal_request.pwm
        dur = goal_request.duration_ms
        if sid not in self.servo_ids:
            self.get_logger().warn(f"reject goal: unknown id {sid}")
            return GoalResponse.REJECT
        # ⚠️每次实时读取参数，不要读self缓存！！！
        pwm_min_now = self.get_parameter("pwm_min").value
        pwm_max_now = self.get_parameter("pwm_max").value
        if not (pwm_min_now <= target_pwm <= pwm_max_now):
            self.get_logger().warn(f"reject goal pwm {target_pwm} out range [{pwm_min_now}, {pwm_max_now}]")
            return GoalResponse.REJECT
        if dur <= 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("receive cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        res = MoveServo.Result()
        sid = goal.id
        target_pwm = goal.pwm
        dur_ms = goal.duration_ms
        msg = ServoCommand()
        msg.id = goal.id
        msg.pwm = goal.pwm
        msg.duration_ms = goal.duration_ms
        self.pub_command.publish(msg)
        self.get_logger().info("已发布 /servo/command，可被bag录制")

        if dur_ms <= 0:
            dur_ms = self.get_parameter("default_duration_ms").value

        self.send_raw_cmd(f"{FRAME_HEAD}{sid:03d}{CMD_TORQUE_ON}{FRAME_END}")
        self.send_move_cmd(sid, target_pwm, dur_ms)

        start_time = self.get_clock().now()
        total_dur_ns = dur_ms * 1_000_000
        feedback_msg = MoveServo.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                res.success = False
                res.message = "canceled"
                res.final_pwm = 0
                self.send_raw_cmd(f"{FRAME_HEAD}{sid:03d}{CMD_TORQUE_OFF}{FRAME_END}")
                return res

            elapsed = (self.get_clock().now() - start_time).nanoseconds
            progress = min(1.0, float(elapsed) / total_dur_ns)
            feedback_msg.progress = progress
            pos_read = self.read_servo_position(sid)
            if pos_read is not None:
                feedback_msg.current_pwm = pos_read
            goal_handle.publish_feedback(feedback_msg)

            if progress >= 1.0:
                break

            time.sleep(0.2)

        final_pos = self.read_servo_position(sid)
        if final_pos is None:
            res.success = False
            res.message = "读取舵机位置失败（串口无回包）"
            res.final_pwm = 0
        elif abs(final_pos - target_pwm) > 30:
            res.success = False
            res.message = f"位置未到位，目标:{target_pwm} 实际:{final_pos}"
            res.final_pwm = final_pos
        else:
            res.success = True
            res.message = "success"
            res.final_pwm = final_pos

        goal_handle.succeed()
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
