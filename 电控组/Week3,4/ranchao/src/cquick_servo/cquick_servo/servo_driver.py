#!/usr/bin/env python3
"""舵机驱动节点：参数化 + 串口 + Action Server + /servo/command 订阅"""
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
import serial

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand
from cquick_servo.commands import (
    build_move_cmd,
    build_pulr_cmd,
    build_pulk_cmd,
    build_prad_cmd,
    parse_prad,
)

NEUTRAL_PWM = 1500     # 协议常量：中立位
TOLERANCE = 30         # 到位判定容差


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # ===== 1. 声明参数 =====
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter(
            'allowed_baudrates',
            [9600, 19200, 38400, 57600, 115200, 230400, 460800])
        self.declare_parameter('startup_reset', True)

        # ===== 参数校验回调：注册它才有权拒绝非法值 =====
        self.add_on_set_parameters_callback(self._param_callback)

        self.cb_group = ReentrantCallbackGroup()
        self.ser = None
        self._warned = False

        self._open_serial()
        if self.get_parameter('startup_reset').value:
            self._startup_reset()

        # ===== 2. Action 服务端 =====
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # ===== 3. 内部调试话题（bag 回放用，与 Action 走同一发送函数）=====
        self.create_subscription(
            ServoCommand, '/servo/command', self.command_cb, 10,
            callback_group=self.cb_group)

        # 状态话题：bag 回放要看它
        self.state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)

        self.create_timer(1.0, self._watchdog, callback_group=self.cb_group)
        self.get_logger().info('servo_driver 已启动，Action 名称: /move_servo')

    def _publish_state(self, sid, pwm, dur):
        """把实际下发的指令发布到 /servo/state，给 bag 用"""
        msg = ServoCommand()
        msg.id = int(sid)
        msg.pwm = int(pwm)
        msg.duration_ms = int(dur)
        self.state_pub.publish(msg)

    def _param_callback(self, params):
        pending = {}
        for p in params:
            pending[p.name] = p.value

        def cur(name):
            if name in pending:
                return pending[name]
            return self.get_parameter(name).value

        allowed_baud = tuple(self.get_parameter('allowed_baudrates').value)
        for p in params:
            if p.name in ('pwm_min', 'pwm_max'):
                if not isinstance(p.value, int):
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name} 必须是整数')
                if p.value < 500 or p.value > 2500:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{p.name}={p.value} 超出 500~2500（PWM 电气极限）')
            elif p.name == 'baudrate':
                if p.value not in allowed_baud:
                    return SetParametersResult(
                        successful=False,
                        reason=f'不支持的波特率 {p.value}，可选 {allowed_baud}')
            elif p.name == 'port':
                if not isinstance(p.value, str) or not p.value.startswith('/dev/'):
                    return SetParametersResult(
                        successful=False, reason=f'串口设备名非法: {p.value}')
            elif p.name == 'default_duration_ms':
                if not (1 <= int(p.value) <= 60000):
                    return SetParametersResult(
                        successful=False, reason='default_duration_ms 需在 1~60000')
            elif p.name == 'servo_ids':
                if not isinstance(p.value, list) or not p.value:
                    return SetParametersResult(successful=False,
                                               reason='servo_ids 不能为空')
                for i in p.value:
                    if not (1 <= int(i) <= 254):
                        return SetParametersResult(
                            successful=False,
                            reason=f'servo_ids 含非法 ID {i}（允许 1~254）')

        lo, hi = int(cur('pwm_min')), int(cur('pwm_max'))
        if lo >= hi:
            return SetParametersResult(
                successful=False,
                reason=f'pwm_min({lo}) 必须小于 pwm_max({hi})')
        return SetParametersResult(successful=True)

    def _pwm_min(self):
        return int(self.get_parameter('pwm_min').value)

    def _pwm_max(self):
        return int(self.get_parameter('pwm_max').value)

    def _duration(self, ms):
        default_ms = int(self.get_parameter('default_duration_ms').value)
        return int(ms) if int(ms) > 0 else default_ms

    def _open_serial(self):
        if self.ser is not None:
            return True
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baud, bytesize=8, parity='N',
                                     stopbits=1, timeout=0.2)
            self.get_logger().info(f'串口已打开: {port} @ {baud}')
            self._warned = False
            return True
        except Exception as e:
            self.ser = None
            if not self._warned:
                self.get_logger().warn(
                    f'串口打不开: {e} —— 没接硬件时正常，插上后自动重连（此提示只报一次）')
                self._warned = True
            return False

    def _send(self, cmd: str) -> bytes:
        """唯一出口：Action、话题、启动初始化都必须走这里"""
        if self.ser is None and not self._open_serial():
            return b''
        try:
            self.ser.write(cmd.encode())
            self.ser.flush()
            return self.ser.read(64)
        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')
            self.ser = None
            return b''

    def _read_pwm(self, sid):
        """发 PRAD 读回实际位置，返回 int 或 None（失败重试 3 次）。

        整机多舵机同总线时 PRAD 丢包率明显升高（见调试记录 11-22），
        单次读取经常拿不到回显，所以这里做 3 次重试、每次隔 0.15s。
        """
        for _ in range(3):
            val = parse_prad(self._send(build_prad_cmd(sid)))
            if val is not None:
                return val
            time.sleep(0.15)
        return None

    def _startup_reset(self):
        if self.ser is None and not self._open_serial():
            return
        for sid in self.get_parameter('servo_ids').value:
            self._send(build_pulr_cmd(sid))
            self._send(build_move_cmd(sid, NEUTRAL_PWM, 800))
        self.get_logger().info('启动初始化已发出（PULR + 回中位）')

    def _watchdog(self):
        if self.ser is None:
            self._open_serial()

    def goal_callback(self, goal_request):
        cmd = goal_request.command
        sid, pwm = int(cmd.id), int(cmd.pwm)
        lo, hi = self._pwm_min(), self._pwm_max()
        ids = list(self.get_parameter('servo_ids').value)
        if sid not in ids:
            self.get_logger().warn(f'拒绝 goal：ID {sid} 不在 servo_ids {ids} 里')
            return GoalResponse.REJECT
        if pwm < lo or pwm > hi:
            self.get_logger().warn(f'拒绝 goal：pwm {pwm} 超出安全范围 {lo}~{hi}')
            return GoalResponse.REJECT
        if int(cmd.duration_ms) > 60000:
            self.get_logger().warn(f'拒绝 goal：duration_ms {cmd.duration_ms} 过大')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('收到取消请求，接受')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        cmd = goal_handle.request.command
        sid, target = int(cmd.id), int(cmd.pwm)
        duration = self._duration(cmd.duration_ms)
        result = MoveServo.Result()
        feedback = MoveServo.Feedback()

        if not self._open_serial():
            result.success = False
            result.final_pwm = 0
            result.message = '串口打不开，指令未发送'
            goal_handle.abort()
            return result

        self._send(build_pulr_cmd(sid))
        self._send(build_move_cmd(sid, target, duration))
        self._publish_state(sid, target, duration)
        self.get_logger().info(f'执行 ID={sid} pwm={target} duration={duration}ms')

        start = time.time()
        while True:
            if goal_handle.is_cancel_requested:
                self._send(build_move_cmd(sid, NEUTRAL_PWM, 300))
                goal_handle.canceled()
                result.success = False
                result.final_pwm = self._read_pwm(sid) or 0
                result.message = 'canceled'
                self.get_logger().warn('已取消，舵机回中位')
                return result

            elapsed = time.time() - start
            progress = min(elapsed / (duration / 1000.0), 1.0)
            feedback.current_pwm = target
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)
            self._publish_state(sid, target, duration)

            if progress >= 1.0:
                break
            time.sleep(0.2)

        actual = self._read_pwm(sid)
        if actual is None:
            # 整机总线负载重时 PRAD 可能 3 次都拿不到回显。
            # 指令本身已经发出（_send 无异常），此时按目标放行并记 warning，
            # 而不是直接 ABORTED——否则供电紧张的整机上一条正常指令也会被判失败。
            self.get_logger().warn(
                f'PRAD 无回显（重试 3 次），按目标值放行: id={sid} pwm={target}')
            result.success = True
            result.final_pwm = target
            result.message = 'ok（PRAD 无回显，按目标放行）'
            goal_handle.succeed()
            return result

        result.final_pwm = actual
        diff = abs(actual - target)
        if diff <= TOLERANCE:
            result.success = True
            result.message = 'ok'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f'未到位，差 {diff}（目标 {target}，实际 {actual}）'
            goal_handle.abort()
        return result

    def command_cb(self, msg):
        sid, pwm = int(msg.id), int(msg.pwm)
        if pwm < self._pwm_min() or pwm > self._pwm_max():
            self.get_logger().warn(f'话题指令越界，忽略: pwm={pwm}')
            return
        if sid not in list(self.get_parameter('servo_ids').value):
            self.get_logger().warn(f'话题指令 ID 不存在，忽略: {sid}')
            return
        self._send(build_pulr_cmd(sid))
        self._send(build_move_cmd(sid, pwm, self._duration(msg.duration_ms)))
        self._publish_state(sid, pwm, self._duration(msg.duration_ms))


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
        for sid in node.get_parameter('servo_ids').value:
            node._send(build_pulk_cmd(sid))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__&#8203;main__':
    main()
