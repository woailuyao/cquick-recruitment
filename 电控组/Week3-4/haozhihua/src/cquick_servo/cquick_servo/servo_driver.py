import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# SetParametersResult 在 rcl_interfaces 包里，不在 rclpy.parameter 里
from rcl_interfaces.msg import SetParametersResult

import serial  # 串口库（pyserial）

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')  # 节点名必须和 yaml 顶层 key 一致

        # 串口写锁：execute_callback 和 _on_command 跑在不同线程，
        # 不加锁两条指令的字节会交错写进串口，控制板整条丢弃
        self._lock = threading.Lock()
        # 记住刚发出去的指令，用来过滤「自己订阅自己」产生的回声
        self._last_sent = None

        # 第一步：声明八个参数
        # 所有「配置值」只许出现在这里，逻辑里一律 get_parameter 读出来
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('allowed_baudrates', [9600, 115200, 1000000])
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4])
        self.declare_parameter('startup_reset', True)

        # 第二步：注册参数设置回调（只有它能拒绝非法参数值）
        self.add_on_set_parameters_callback(self.on_set_parameters)

        # 第三步：开串口（值全部来自参数，不写字面量）
        try:
            self.ser = serial.Serial(
                port=self.get_parameter('port').value,
                baudrate=self.get_parameter('baudrate').value,
                timeout=1.0,
            )
            self.get_logger().info(
                '串口已打开: %s' % self.get_parameter('port').value)
        except Exception as e:
            # 没接硬件时走到这里，属于正常，先让节点活着方便调试
            self.ser = None
            self.get_logger().warn('串口打开失败（可能没接硬件）: %s' % e)

        # 第四步：启动时是否全部回中位
        if self.get_parameter('startup_reset').value:
            self._reset_all()

        # 第五步：/servo/command 一发布一订阅（录 bag 和回放都靠它）
        # 发布：Action 每发一条 MOVE 就抄一份到话题，录 bag 才有内容
        # 订阅：bag play 把消息重放回来，走的还是同一个底层发送函数
        self.cmd_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self.sub = self.create_subscription(
            ServoCommand, '/servo/command', self._on_command, 10)
        self._armed = False   # 回放时是否需要先补一次 PULR 恢复扭力

        # 第六步：建 Action 服务端，挂 ReentrantCallbackGroup 允许回调并发
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    # ============ 参数设置回调 ============
    def on_set_parameters(self, params):
        new_values = {}
        for p in params:
            new_values[p.name] = p.value

        for p in params:
            if p.name in ('pwm_min', 'pwm_max'):
                if not (500 <= p.value <= 2500):
                    return SetParametersResult(
                        successful=False,
                        reason='%s 必须在 500~2500 之间，收到 %s' % (p.name, p.value))
            if p.name == 'baudrate':
                # 白名单也来自参数，不写字面量
                allowed = self.get_parameter('allowed_baudrates').value
                if p.value not in allowed:
                    return SetParametersResult(
                        successful=False,
                        reason='不认识的波特率 %s，允许: %s'
                               % (p.value, list(allowed)))

        # 跨参数校验：pwm_min 必须小于 pwm_max（比较的是「将要生效」的新值）
        pmin = new_values.get('pwm_min', self.get_parameter('pwm_min').value)
        pmax = new_values.get('pwm_max', self.get_parameter('pwm_max').value)
        if pmin >= pmax:
            return SetParametersResult(
                successful=False,
                reason='pwm_min(%d) 必须小于 pwm_max(%d)' % (pmin, pmax))

        return SetParametersResult(successful=True)

    # ============ Action 四个回调 ============
    def goal_callback(self, goal_request):
        cmd = goal_request.command
        # 每次实时读参数，不读 __init__ 里的副本，否则 param set 不生效
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value
        servo_ids = self.get_parameter('servo_ids').value

        if cmd.id not in servo_ids:
            self.get_logger().warn('拒绝：ID %d 不在 servo_ids 里' % cmd.id)
            return GoalResponse.REJECT
        if not (pwm_min <= cmd.pwm <= pwm_max):
            self.get_logger().warn(
                '拒绝：pwm %d 超出 [%d, %d]' % (cmd.pwm, pwm_min, pwm_max))
            return GoalResponse.REJECT
        if cmd.duration_ms <= 0:
            self.get_logger().warn('拒绝：duration_ms 非法 %d' % cmd.duration_ms)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_accepted(self, goal_handle):
        # 这一行把 goal 状态从 ACCEPTED 推进到 EXECUTING，
        # 并把 execute_callback 作为任务交给 executor（不阻塞当前线程）。
        # 少了它，后面 succeed()/abort() 会因状态非法而报错。
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        cmd = goal_handle.request.command
        feedback = MoveServo.Feedback()

        # 上电后第一次动作前必须恢复扭力
        self._send_raw(self._build_cmd(cmd.id, 0, 0, 'PULR'))
        # 给控制板 50ms 把 PULR 处理完，否则紧接着到的 MOVE 会被整条丢弃
        time.sleep(0.05)

        # 发主指令（走 _send_command：先上话题留痕，再写串口）
        self._send_command(cmd.id, cmd.pwm, cmd.duration_ms)

        # 循环发 feedback，至少 5Hz
        rate = self.create_rate(5.0)
        steps = max(1, cmd.duration_ms // 200)
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                # 收到取消：回中位、标记 canceled、返回失败
                # T 不能小于 1000：控制板会把 T<1000 的指令整条静默丢弃
                self._send_command(cmd.id, 1500, 1000)
                goal_handle.canceled()
                result = MoveServo.Result()
                result.success = False
                result.message = 'canceled'
                return result
            feedback.current_pwm = cmd.pwm
            feedback.progress = (i + 1) / steps
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        # 读回 PRAD 校验，差 >30 判没到位
        actual = self._read_position(cmd.id)
        result = MoveServo.Result()
        result.actual_pwm = actual
        if abs(actual - cmd.pwm) <= 30:
            result.success = True
            result.message = 'ok'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = '未到位：目标 %d 实际 %d 差 %d' % (
                cmd.pwm, actual, abs(actual - cmd.pwm))
            goal_handle.abort()
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    # ============ 底层发送（Action 和 bag 回放都走这里） ============
    def _build_cmd(self, sid, pwm, duration_ms, kind):
        if kind == 'PULR':
            return ('#%03dPULR!' % sid).encode('ascii')
        if kind == 'MOVE':
            return ('#%03dP%dT%d!' % (sid, pwm, duration_ms)).encode('ascii')
        return b''

    def _send_command(self, sid, pwm, duration_ms):
        """MOVE 指令的唯一出口：先发话题留痕（给 bag），再写串口"""
        msg = ServoCommand()
        msg.id = sid
        msg.pwm = pwm
        msg.duration_ms = duration_ms
        # 必须在 publish 之前记下来：订阅回调可能在另一个线程立刻执行
        self._last_sent = (sid, pwm, duration_ms)
        self.cmd_pub.publish(msg)
        self._send_raw(self._build_cmd(sid, pwm, duration_ms, 'MOVE'))

    def _send_raw(self, data):
        if self.ser is not None:
            with self._lock:
                self.ser.write(data)
                self.ser.flush()
        else:
            self.get_logger().debug('(无硬件) 发送: %s' % data.decode())

    def _read_position(self, sid):
        # 无硬件时返回 1500，让逻辑能跑通
        if self.ser is None:
            return 1500
        with self._lock:
            # 丢掉上一条指令的残留字节，否则可能读到上一次的应答
            self.ser.reset_input_buffer()
            self.ser.write(('#%03dPRAD!' % sid).encode('ascii'))
            self.ser.flush()
            resp = self.ser.readline().decode('ascii', 'replace').strip()
        try:
            # 应答形如 '#002P1304!'，取最后一个 P 之后、! 之前的数字
            return int(resp.split('P')[-1].rstrip('!'))
        except Exception:
            self.get_logger().warn('PRAD 回读无法解析: %r' % resp)
            return -1

    def _reset_all(self):
        for sid in self.get_parameter('servo_ids').value:
            # 回中位用 T1000：T<1000 会被控制板静默丢弃
            self._send_raw(self._build_cmd(sid, 1500, 1000, 'MOVE'))

    def _on_command(self, msg):
        # bag 回放时走到这里：收到一条指令，走同一个底层发送函数
        # 但节点自己也订阅了自己发布的这个话题，所以要先过滤掉自己的回声，
        # 否则每条 MOVE 会写两次串口，和 PULR 挤在一起被控制板丢弃
        if self._last_sent == (msg.id, msg.pwm, msg.duration_ms):
            self._last_sent = None
            return
        try:
            if not self._armed:
                self._send_raw(self._build_cmd(msg.id, 0, 0, 'PULR'))
                self._armed = True
                time.sleep(0.05)
            self._send_raw(self._build_cmd(msg.id, msg.pwm, msg.duration_ms, 'MOVE'))
        except Exception as e:
            self.get_logger().warn('回放指令失败: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    # 关键：MultiThreadedExecutor，否则 execute_callback 跑起来后取消进不来
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Ctrl+C：rclpy 的信号处理会把 KeyboardInterrupt 抛出来，
        # 不接住就会打一整屏 Traceback，进程还以 exit code -2 退出，
        # launch 那边会跟着报 "process has died"。接住就干净退出。
        pass
    finally:
        node.destroy_node()
        # Ctrl+C 时 rclpy 的信号处理已经关过 context，重复 shutdown 会抛 RCLError
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
