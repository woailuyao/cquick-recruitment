#!/usr/bin/env python3
import re
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
import serial

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # ---------- 参数 ----------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4])
        self.declare_parameter('startup_reset', True)
        self.declare_parameter('replay_mode', False)

        # 参数校验回调（改参数时先过这一关）
        self.add_on_set_parameters_callback(self.on_param_change)

        # ---------- 串口 ----------
        self.io_lock = threading.RLock()
        self.ser = None
        self.open_serial()

        # ---------- 通信 ----------
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self, MoveServo, 'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # 供 bag 回放用的内部话题
        self.create_subscription(ServoCommand, '/servo/command',
                                 self.on_command, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)   # ← 新增                        
        # 心跳状态（进 bag、也方便调试）
        self.state_pub = self.create_publisher(String, '/servo/state', 10)
        self.create_timer(0.5, self.publish_state)

        if self.get_parameter('startup_reset').value:
            self.reset_all()

    def open_serial(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'串口已打开: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
            self.ser = None
    # ================= 串口 =================
    def send_cmd(self, cmd: str) -> bool:
        with self.io_lock:
            if self.ser is None:
                self.get_logger().error('串口未打开，无法发送')
                return False
            try:
                self.ser.write(cmd.encode())
                self.ser.flush()
                return True
            except Exception as e:
                self.get_logger().error(f'串口写入失败: {e}')
                self.ser = None
                return False

    def read_prad(self, servo_id: int, timeout: float = 0.1):
        with self.io_lock:
            if self.ser is None:
                return None
            self.ser.reset_input_buffer()
            self.send_cmd(f'#{servo_id:03d}PRAD!\r\n')
            end = time.time() + timeout
            buf = b''
            while time.time() < end:
                buf += self.ser.read(64) or b''
                m = re.search(rb'#\d{3}P(\d{1,4})!', buf)
                if m:
                    return int(m.group(1))
                time.sleep(0.01)
            return None
    # ================= 校验 =================
    def on_param_change(self, params):
        pmin = self.get_parameter('pwm_min').value
        pmax = self.get_parameter('pwm_max').value
        for p in params:
            if p.name == 'pwm_min':
                pmin = p.value
            if p.name == 'pwm_max':
                pmax = p.value
        for p in params:
            if p.name in ('pwm_min', 'pwm_max'):
                if not (500 <= p.value <= 2500):
                    return SetParametersResult(
                        successful=False, reason=f'{p.name}={p.value} 超出 500~2500')
        if pmin >= pmax:
            return SetParametersResult(
                successful=False, reason=f'pwm_min({pmin}) 必须小于 pwm_max({pmax})')
        return SetParametersResult(successful=True)

    def _check(self, sid, pwm):
        if sid not in self.get_parameter('servo_ids').value:
            return False, f'ID {sid} 不在 servo_ids 里'
        pmin = self.get_parameter('pwm_min').value
        pmax = self.get_parameter('pwm_max').value
        if pwm < pmin or pwm > pmax:
            return False, f'pwm {pwm} 超出 [{pmin}, {pmax}]'
        return True, ''

    def move(self, sid, pwm, dur):
        ok, reason = self._check(sid, pwm)
        if not ok:
            self.get_logger().warn(reason)
            return False
        return self.send_cmd(f'#{sid:03d}P{pwm:04d}T{dur:04d}!\r\n')

    def reset_all(self):
        if self.ser is None:
            return
        ids = self.get_parameter('servo_ids').value
        dur = self.get_parameter('default_duration_ms').value
        for i in ids:
            self.send_cmd(f'#{i:03d}PULR!\r\n')
        for i in ids:
            self.move(i, 1500, dur)
        self.get_logger().info('全部舵机已回中位')

    # ================= Action =================
    def goal_callback(self, goal_request):
        ok, reason = self._check(goal_request.id, goal_request.pwm)
        if not ok:
            self.get_logger().warn(f'拒绝 goal: {reason}')
            return GoalResponse.REJECT
        if goal_request.duration_ms <= 0 or goal_request.duration_ms > 10000:
            self.get_logger().warn(f'拒绝 goal: duration_ms={goal_request.duration_ms} 非法')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'接受 goal: id={goal_request.id} pwm={goal_request.pwm} dur={goal_request.duration_ms}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('收到取消请求')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        dur = req.duration_ms or self.get_parameter('default_duration_ms').value

        self.send_cmd(f'#{req.id:03d}PULR!\r\n')
        time.sleep(0.05)

        m = ServoCommand()
        m.id = req.id
        m.pwm = req.pwm
        m.duration_ms = dur
        self.cmd_pub.publish(m)

        if not self.move(req.id, req.pwm, dur):
            goal_handle.abort()
            r = MoveServo.Result()
            r.success = False
            r.final_pwm = 0
            r.message = '发送失败'
            return r

        steps = max(5, int(dur / 200))
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                self.move(req.id, 1500, 500)
                goal_handle.canceled()
                r = MoveServo.Result()
                r.success = False
                r.final_pwm = 1500
                r.message = 'canceled'
                self.get_logger().warn('已取消，回中位')
                return r
            time.sleep(dur / 1000.0 / steps)
            fb = MoveServo.Feedback()
            v = self.read_prad(req.id, timeout=0.03)
            fb.current_pwm = int(v) if v is not None else int(req.pwm)
            fb.progress = float(i + 1) / steps
            goal_handle.publish_feedback(fb)

        final = self.read_prad(req.id, timeout=0.2)
        if final is None:
            goal_handle.abort()
            r = MoveServo.Result()
            r.success = False
            r.final_pwm = 0
            r.message = 'PRAD 读回失败'
            return r

        err = abs(final - req.pwm)
        if err > 50:
            self.get_logger().warn(
                f'id={req.id} 目标 {req.pwm} 实际 {final} 差 {err}，重发一次')
            self.move(req.id, req.pwm, dur)
            time.sleep(dur / 1000.0 + 0.2)
            retry = self.read_prad(req.id, timeout=0.2)
            if retry is not None:
                final = retry
                err = abs(final - req.pwm)
                self.get_logger().info(
                    f'id={req.id} 重发后实际 {final}，差 {err}')

        if err > 50:
            goal_handle.abort()
            r = MoveServo.Result()
            r.success = False
            r.final_pwm = final
            r.message = f'未到位，差 {err}'
            return r

        goal_handle.succeed()
        r = MoveServo.Result()
        r.success = True
        r.final_pwm = final
        r.message = 'ok'
        return r

    # ================= 话题 =================
    def on_command(self, msg: ServoCommand):
        if not self.get_parameter('replay_mode').value:
            return
        self.get_logger().info(f'回放: id={msg.id} pwm={msg.pwm} dur={msg.duration_ms}')    
        self.send_cmd(f'#{msg.id:03d}PULR!\r\n')
        self.move(msg.id, msg.pwm, msg.duration_ms)

    def publish_state(self):
        if self.ser is None:
            return
        m = String()
        m.data = 'alive'
        self.state_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()