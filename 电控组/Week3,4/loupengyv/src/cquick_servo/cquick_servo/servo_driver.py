#!/usr/bin/env python3
import re
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:
    import serial
except ImportError:
    serial = None

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

FEEDBACK_HZ = 10
FEEDBACK_DT = 1.0 / FEEDBACK_HZ

# 波特率合法性白名单（不是实际使用的值，实际值来自参数 baudrate）
VALID_BAUDRATES = (9600, 19200, 38400, 57600, 115200, 230400, 500000, 1000000)

# 上电回中用的中位值；标定过 1500 的舵臂装上后直接生效
CENTER_PWM = 1500

# #001PRAD1500! 这种标准回复
PRAD_RE = re.compile(r'#\s*\d{3}\s*P?\s*RAD\s*(-?\d+)')
# 兜底：#001P1500! 这种省略了 RAD 前缀的固件
NUM_RE = re.compile(r'-?\d+')


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # ---- 七个参数，全部来自 yaml / 命令行，代码里不写死 ----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)

        self.add_on_set_parameters_callback(self.on_param_change)

        self.ser = None
        self.current_pwm = 1500

        # ---- bag 用的两个话题 ----
        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self.state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)
        self.create_subscription(
            ServoCommand, '/servo/command', self.on_cmd_topic, 10)
        self.create_timer(0.2, self.publish_state)

        self.open_port()
        self.apply_startup_reset()

        self.group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.group,
        )
        self.get_logger().info('servo_driver 就绪，Action: /move_servo')

    # ---------- 参数校验 ----------
    def on_param_change(self, params):
        for p in params:
            if p.name in ('pwm_min', 'pwm_max'):
                if not 500 <= p.value <= 2500:
                    return SetParametersResult(
                        successful=False,
                        reason='%s 必须在 500~2500 之间，收到 %d' % (p.name, p.value))
            if p.name == 'baudrate':
                if p.value not in VALID_BAUDRATES:
                    return SetParametersResult(
                        successful=False,
                        reason='不支持的波特率 %d' % p.value)

        pmin = self.get_parameter('pwm_min').value
        pmax = self.get_parameter('pwm_max').value
        for p in params:
            if p.name == 'pwm_min':
                pmin = p.value
            if p.name == 'pwm_max':
                pmax = p.value
        if pmin >= pmax:
            return SetParametersResult(
                successful=False,
                reason='pwm_min(%d) 必须小于 pwm_max(%d)' % (pmin, pmax))
        return SetParametersResult(successful=True)

    # ---------- 串口 ----------
    def open_port(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        if serial is None:
            self.get_logger().warn('pyserial 未安装，进入模拟模式')
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            self.get_logger().info('串口已打开: %s @ %d' % (port, baud))
        except Exception as e:
            self.get_logger().warn('串口打不开（没硬件时正常）: %s' % e)

    def apply_startup_reset(self):
        """startup_reset=true 时，上电把所有配置过的舵机拉回中位 1500。

        串口没打开（没硬件 / 模拟模式）就静默跳过，不打多余日志。
        """
        if self.ser is None:
            return
        if not self.get_parameter('startup_reset').value:
            self.get_logger().info('startup_reset=false，跳过上电回中')
            return
        ids = list(self.get_parameter('servo_ids').value)
        dur = int(self.get_parameter('default_duration_ms').value)
        self.get_logger().info(
            'startup_reset：%s 全部回中 %d（每个 %d ms）' % (ids, CENTER_PWM, dur))
        for sid in ids:
            self._write('#%03dPULR!' % sid)
            self._write(self.build_command(sid, CENTER_PWM, dur))

    @staticmethod
    def build_command(sid, pwm, duration_ms):
        return '#%03dP%04dT%04d!' % (sid, pwm, duration_ms)

    def _write(self, cmd):
        self.get_logger().info('发送: %s' % cmd)
        if self.ser is not None:
            self.ser.write(cmd.encode())
            self.ser.flush()

    def read_prad(self, sid):
        """发 #001PRAD! 并解析回读角度，收不到就返回 None。"""
        if self.ser is None:
            return None
        try:
            self.ser.reset_input_buffer()
            self.ser.write(('#%03dPRAD!' % sid).encode())
            self.ser.flush()
        except Exception as e:                            # 串口被拔掉之类
            self.get_logger().warn('PRAD 发送失败: %s' % e)
            return None

        # 攒到 '!' 为止，最多等 300 ms —— 比 sleep+read 快，也不会把半截数据当结果
        buf = b''
        deadline = time.time() + 0.3
        try:
            while time.time() < deadline:
                n = self.ser.in_waiting
                if n:
                    buf += self.ser.read(n)
                    if b'!' in buf:
                        break
                else:
                    time.sleep(0.005)
        except Exception as e:                            # noqa: BLE001
            self.get_logger().warn('PRAD 读取失败: %s' % e)
            return None
        return self.parse_prad(buf.decode('ascii', 'ignore'))

    @staticmethod
    def parse_prad(text):
        """兼容 #001PRAD1500! 和 #001P1500! 两种回复格式。

        只有回显（'#001PRAD!'）时看不到第 2 个数字，直接返回 None，
        不会被误当成角度 1。
        """
        if not text:
            return None
        up = text.upper()
        hits = PRAD_RE.findall(up)
        if hits:
            val = int(hits[-1])                           # 粘包时取最新那一帧
        else:
            nums = NUM_RE.findall(up)
            if len(nums) < 2:                             # 只有一个数 = 只剩 ID，没有角度
                return None
            val = int(nums[-1])                           # 第一个数是 3 位 ID，取最后一个
        return val if 0 <= val <= 3000 else None          # 离谱的数值当没收到

    # ---------- bag：状态发布 + 指令订阅（回放） ----------
    def publish_state(self):
        msg = ServoCommand()
        msg.id = 0
        msg.pwm = self.current_pwm
        msg.duration_ms = 0
        self.state_pub.publish(msg)

    def on_cmd_topic(self, msg):
        """话题收到指令 -> 走和 Action 完全相同的底层发送函数。
        bag 回放时走的就是这条路。"""
        self.current_pwm = msg.pwm
        self._write('#%03dPULR!' % msg.id)
        self._write(self.build_command(msg.id, msg.pwm, msg.duration_ms))

    # ---------- Action ----------
    def goal_cb(self, goal_request):
        pmin = self.get_parameter('pwm_min').value
        pmax = self.get_parameter('pwm_max').value
        ids = list(self.get_parameter('servo_ids').value)

        if goal_request.id not in ids:
            self.get_logger().error(
                'goal 被拒：ID %d 不在 servo_ids %s 里' % (goal_request.id, ids))
            return GoalResponse.REJECT
        if not pmin <= goal_request.pwm <= pmax:
            self.get_logger().error(
                'goal 被拒：pwm %d 超出安全范围 %d~%d'
                % (goal_request.pwm, pmin, pmax))
            return GoalResponse.REJECT
        if goal_request.duration_ms == 0:
            self.get_logger().error('goal 被拒：duration_ms 不能为 0')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn('收到取消请求')
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        sid = goal_handle.request.id
        pwm = goal_handle.request.pwm
        dur = goal_handle.request.duration_ms
        result = MoveServo.Result()

        # 发到话题，由 on_cmd_topic 统一写串口 —— Action 和话题共用一条发送路径
        cmd = ServoCommand()
        cmd.id = sid
        cmd.pwm = pwm
        cmd.duration_ms = dur
        self.cmd_pub.publish(cmd)

        steps = max(1, int(dur / 1000.0 * FEEDBACK_HZ))
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._write('#%03dPULK!' % sid)
                result.success = False
                result.final_pwm = 0
                result.message = 'canceled'
                self.get_logger().warn('已取消，舵机卸力')
                return result

            fb = MoveServo.Feedback()
            fb.current_pwm = pwm
            fb.progress = float(i + 1) / steps
            goal_handle.publish_feedback(fb)
            time.sleep(FEEDBACK_DT)

        actual = self.read_prad(sid)
        if actual is None:
            result.success = True
            result.final_pwm = pwm
            result.message = '模拟模式，未做 PRAD 回读'
        else:
            result.final_pwm = actual
            if abs(actual - pwm) <= 30:
                result.success = True
                result.message = '到位，PRAD=%d' % actual
            else:
                result.success = False
                result.message = '未到位：目标 %d，实际 %d，差 %d' % (
                    pwm, actual, abs(actual - pwm))

        goal_handle.succeed()
        self.get_logger().info('完成：%s' % result.message)
        return result


def main():
    rclpy.init()
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
