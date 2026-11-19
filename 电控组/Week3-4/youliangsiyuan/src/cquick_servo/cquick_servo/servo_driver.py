#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口驱动节点：Action Server + /servo/command 调试总线。

一条 goal 的完整旅程（评审提问就按这个答）：

    action client
        │ goal(id, pwm, duration_ms)
        ▼
    goal_callback            ← ① 校验（实时读参数！）：pwm 在不在 pwm_min~pwm_max、id 在不在 servo_ids
        │ ACCEPT / REJECT
        ▼
    handle_accepted_callback ← ② 显式调用 goal_handle.execute()
        │
        ▼
    execute_callback         ← ③ PULR 上力 → 发 #001P1700T1000! → 每 200ms 发 feedback
        │                       → 每轮检查 is_cancel_requested（取消要在这里生效）
        │                       → 走完发 #001PRAD! 读回，差超过 ±30 就重发一次
        ▼
    result(success, final_pwm, retry_count, message)

为什么必须 MultiThreadedExecutor + ReentrantCallbackGroup（细则最重的 12 分就在这里）：
    execute_callback 一跑就是几百毫秒到几秒。单线程 executor 里这个线程被占着，
    取消请求（cancel_callback）根本进不来 —— 于是 Ctrl+C 没反应。
    换成多线程 + ReentrantCallbackGroup，取消回调才能在另一个线程里跑。
"""

import threading

import rclpy
import serial  # pyserial，apt install python3-serial
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand, ServoState

from cquick_servo.servo_protocol import (
    STANDARD_BAUDRATES,
    ProtocolError,
    build_move_command,
    build_prad_command,
    build_torque_command,
    parse_prad_reply,
)

# 串口帧格式：8 数据位 - 无校验 - 1 停止位。这是协议的一部分，不是配置。
SERIAL_FRAME = dict(bytesize=8, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

ACTION_NAME = 'move_servo'
TOPIC_COMMAND = 'servo/command'
TOPIC_STATE = 'servo/state'

# 串口帧之间的最小间隔：与已验证可用的 center_all.py 保持一致。
# 只用于给控制板留出解析时间，不用于动作组步间同步（步间仍由 result 同步）。
SERIAL_FRAME_GAP_S = 0.02

PHASE_IDLE = 'idle'
PHASE_TORQUE_ON = 'torque_on'
PHASE_MOVING = 'moving'
PHASE_CANCELED = 'canceled'
PHASE_ERROR = 'error'

# 参与参数校验的那些参数名（一次 set 可能要一起看）
_VALIDATED_PARAMS = (
    'port', 'baudrate', 'pwm_min', 'pwm_max', 'default_duration_ms',
    'servo_ids', 'joint_offsets', 'pwm_hard_min', 'pwm_hard_max',
    'feedback_period_ms', 'prad_tolerance', 'prad_retries',
    'watchdog_timeout_s',
)


class ServoDriver(Node):

    # ---------------------------------------------------------------- 初始化
    def __init__(self):
        super().__init__('servo_driver')

        self._declare_parameters()

        # 串口读写锁：action 线程和 /servo/command 回放线程可能同时想用串口
        self._serial_lock = threading.Lock()
        self._ser = None
        self._serial_ok = False
        self._applied_port = None
        self._applied_baudrate = None
        # 记住上一次打开失败的原因，避免每秒刷同一条 warn
        self._last_open_error = None

        self._phase = PHASE_IDLE
        self._last_id = 0
        self._last_pwm = 0

        # 看门狗（加分项 +3）
        self._watchdog_deadline_ns = None
        self._watchdog_tripped = False
        self._torque_enabled = False
        self._want_manual_recovery = False
        self._want_torque_off = False

        # ReentrantCallbackGroup：同一 step 的多个 goal 与取消/参数回调都能并行推进；
        # 各 goal 独立等待自己的 result，串口读写仍由 _serial_lock 互斥保护。
        self._cb_group = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(ServoState, TOPIC_STATE, 10)

        # 内部调试总线：驱动把发出去的指令回显到 /servo/command，bag 才录得到
        self._command_pub = self.create_publisher(ServoCommand, TOPIC_COMMAND, 10)
        # 同时也要订阅它 —— bag play 时就是靠这个订阅把动作重放一遍
        self._command_sub = self.create_subscription(
            ServoCommand, TOPIC_COMMAND, self._on_command, 10,
            callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            MoveServo,
            ACTION_NAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self._cb_group,
        )

        # 参数校验回调：没有它，ros2 param set 什么值都进得去
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.create_timer(1.0, self._publish_state)             # 1 Hz 心跳
        self.create_timer(0.1, self._tick_safety)               # 10 Hz 看门狗
        self.create_timer(float(self.get_parameter('reconnect_interval_s').value),
                          self._tick_serial)                    # 串口重连

        self.get_logger().info(
            f'驱动节点已启动：action=/{ACTION_NAME} '
            f'topic=/{TOPIC_COMMAND} state=/{TOPIC_STATE}')

        self._open_serial()

        if bool(self.get_parameter('startup_reset').value):
            self._startup_reset()

    # -------------------------------------------------------------- 参数声明
    def _declare_parameters(self):
        """细则要求的 7 个参数 —— 名字只允许出现在「声明」和「读取」两处。"""
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)

        # 机械零位偏移（装完结构后逐个测出来）：下标 = ID-1，值 = 标准姿态PWM - 1500
        # 装舵臂时舵盘齿位不可能刚好正中，误差全在这个参数里吃掉；
        # 动作 yaml 永远按 1500 中位写，实际发出去的 PWM = yaml值 + 偏移。
        self.declare_parameter('joint_offsets', [0] * 10)

        # 下面这些是「想加可以加」的附加参数，同样不写死在代码里
        self.declare_parameter('pwm_hard_min', 500)        # 电气下限，参数校验用
        self.declare_parameter('pwm_hard_max', 2500)       # 电气上限
        self.declare_parameter('feedback_period_ms', 200)  # feedback 周期（细则要求 >= 5 Hz）
        self.declare_parameter('prad_tolerance', 30)       # PRAD 闭环允许的误差
        self.declare_parameter('prad_retries', 1)          # 没到位时自动重发几次
        self.declare_parameter('watchdog_timeout_s', 1.0)  # 多久没推进就卸力
        self.declare_parameter('serial_read_timeout_s', 0.05)
        self.declare_parameter('reconnect_interval_s', 1.0)  # 定时器周期在启动时定死
        self.declare_parameter('replay_mode', False)       # true 时只认 /servo/command
        self.declare_parameter('torque_enable', True)      # false = 立刻全部卸力

    # ------------------------------------------------------ 参数读取（实时！）
    # ⚠️ 本版最高频的坑：在 __init__ 里把参数抄成 self.pwm_max，之后再也不读参数，
    #    于是 ros2 param set 返回成功、行为不变。所以下面一律**每次现读**。
    def _pwm_min(self):
        return int(self.get_parameter('pwm_min').value)

    def _pwm_max(self):
        return int(self.get_parameter('pwm_max').value)

    def _mid_pwm(self):
        """安全区间的中点 —— 取消回中位、startup_reset 都用它。"""
        return (self._pwm_min() + self._pwm_max()) // 2

    def _servo_ids(self):
        # rclpy 里整数数组参数取出来不一定是 list，统一转一遍
        return [int(v) for v in self.get_parameter('servo_ids').value]

    def _joint_offsets(self):
        return [int(v) for v in self.get_parameter('joint_offsets').value]

    def _joint_offset(self, servo_id):
        """某个关节的机械零位偏移。ID 从 1 开始，列表下标 = ID-1。

        列表比 ID 短时按 0 处理 —— 这样只标定前几台、后面没测也能跑。
        """
        offsets = self._joint_offsets()
        index = int(servo_id) - 1
        if 0 <= index < len(offsets):
            return offsets[index]
        return 0

    def _applied_pwm(self, servo_id, pwm):
        """动作 yaml 里的目标值 + 机械偏移 = 真正发到总线上的 PWM。"""
        return int(pwm) + self._joint_offset(servo_id)

    def _feedback_period(self):
        return Duration(
            seconds=int(self.get_parameter('feedback_period_ms').value) / 1000.0)

    def _default_duration(self):
        return int(self.get_parameter('default_duration_ms').value)

    # ------------------------------------------------------------ 参数校验
    def _on_set_parameters(self, params):
        """只能在「写进去之前」拒绝非法值：返回 successful=True 才会真的写。

        一次 set 可能带多个参数，所以先把「改完之后的完整样子」拼出来，
        再做**组合**校验 —— 单看 pwm_min=2000、pwm_max=1800 各自都合法，
        合在一起就打架了。
        """
        # 1) 拼出改完之后的视图（get_parameter 拿到的是旧值，用 p.value 覆盖）
        pending = {name: self.get_parameter(name).value for name in _VALIDATED_PARAMS}
        for p in params:
            if p.name in pending:
                pending[p.name] = p.value

        # 2) 电气范围
        hard_min = int(pending['pwm_hard_min'])
        hard_max = int(pending['pwm_hard_max'])
        if hard_min >= hard_max:
            return SetParametersResult(
                successful=False,
                reason=f'pwm_hard_min({hard_min}) 必须小于 pwm_hard_max({hard_max})')

        for name in ('pwm_min', 'pwm_max'):
            value = int(pending[name])
            if not hard_min <= value <= hard_max:
                return SetParametersResult(
                    successful=False,
                    reason=f'{name}={value} 超出电气范围 {hard_min}~{hard_max}')

        # 3) 组合校验：上下限不能打架（细则 2.6 明确要求拒掉这种）
        if int(pending['pwm_min']) >= int(pending['pwm_max']):
            return SetParametersResult(
                successful=False,
                reason=(f'pwm_min({pending["pwm_min"]}) 必须小于 '
                        f'pwm_max({pending["pwm_max"]})'))

        # 4) 其它单值校验
        if int(pending['baudrate']) not in STANDARD_BAUDRATES:
            return SetParametersResult(
                successful=False,
                reason=(f'baudrate={pending["baudrate"]} 不是标准档位，'
                        f'可选 {STANDARD_BAUDRATES}'))

        if not 0 < int(pending['default_duration_ms']) <= 9999:
            return SetParametersResult(
                successful=False,
                reason='default_duration_ms 必须 1~9999（协议里 T 只有 4 位）')

        period = int(pending['feedback_period_ms'])
        if not 20 <= period <= 200:
            return SetParametersResult(
                successful=False,
                reason=(f'feedback_period_ms={period} 不合法，必须 20~200'
                        f'（细则要求 feedback >= 5 Hz）'))

        if int(pending['prad_tolerance']) < 0:
            return SetParametersResult(
                successful=False, reason='prad_tolerance 不能为负')
        if int(pending['prad_retries']) < 0:
            return SetParametersResult(
                successful=False, reason='prad_retries 不能为负')
        if float(pending['watchdog_timeout_s']) <= 0:
            return SetParametersResult(
                successful=False, reason='watchdog_timeout_s 必须大于 0')

        ids = [int(v) for v in pending['servo_ids']]
        if not ids:
            return SetParametersResult(
                successful=False, reason='servo_ids 不能是空列表')
        if any(not 1 <= v <= 254 for v in ids):
            return SetParametersResult(
                successful=False, reason=f'servo_ids 里有非法 ID：{ids}（必须 1~254）')

        # 机械零位偏移：单项过大说明测错了或装歪太多，直接拒掉免得动作全跑偏
        offsets = [int(v) for v in pending['joint_offsets']]
        if any(abs(v) > 500 for v in offsets):
            return SetParametersResult(
                successful=False,
                reason=(f'joint_offsets 单项绝对值不能超过 500，收到 {offsets}'
                        f'（超过说明装配偏差太大，应该拆舵臂重装）'))

        if not str(pending['port']).strip():
            return SetParametersResult(successful=False, reason='port 不能为空')

        # 5) 回调里不做副作用，只挂个"待处理项"，由 10 Hz 定时器去执行
        for p in params:
            if p.name == 'torque_enable':
                if bool(p.value) and self._watchdog_tripped:
                    self._want_manual_recovery = True   # 外部确认后才允许恢复
                elif not bool(p.value):
                    self._want_torque_off = True

        return SetParametersResult(successful=True)

    # ---------------------------------------------------------------- 串口
    def _open_serial(self):
        """开串口。**开不起来不退出进程** —— 参数、launch、限位这些演示还得能跑。

        这也是「拔了 USB 不崩」的底子：打开失败只把 serial_online 置 False，定时器会一直重试。
        """
        port = str(self.get_parameter('port').value)
        baudrate = int(self.get_parameter('baudrate').value)
        timeout = float(self.get_parameter('serial_read_timeout_s').value)

        with self._serial_lock:
            self._close_serial_locked()
            self._applied_port = port
            self._applied_baudrate = baudrate
            try:
                self._ser = serial.Serial(
                    port=port, baudrate=baudrate, timeout=timeout, **SERIAL_FRAME)
                self._serial_ok = True
                self._last_open_error = None
                self.get_logger().info(f'串口已打开：{port} @ {baudrate} 8-N-1')
            except (serial.SerialException, OSError) as exc:
                self._ser = None
                self._serial_ok = False
                # 只在"失败原因变了"的时候打 warn，否则重连定时器会每秒刷一条
                if str(exc) != self._last_open_error:
                    self._last_open_error = str(exc)
                    self.get_logger().warn(
                        f'串口打不开：{port} -> {exc}'
                        f'（先查 ls -l /dev/tty*；dialout 组必须重新登录才生效）'
                        f'  之后会每 {self.get_parameter("reconnect_interval_s").value}s '
                        f'自动重试，不再重复刷屏')

    def _close_serial_locked(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except (serial.SerialException, OSError):
                pass
        self._ser = None
        self._serial_ok = False

    def _tick_serial(self):
        """定时器：端口/波特率参数变了就重开；串口掉了就重连。"""
        port = str(self.get_parameter('port').value)
        baudrate = int(self.get_parameter('baudrate').value)
        if port != self._applied_port or baudrate != self._applied_baudrate:
            self.get_logger().info(f'串口参数变了 -> 重新打开 {port} @ {baudrate}')
            self._open_serial()
        elif not self._serial_ok:
            self._open_serial()

    def _write_locked(self, text):
        """真正写串口。调用前必须已持有 _serial_lock。"""
        try:
            self._ser.write(text.encode('ascii'))
            # 不 flush，指令可能卡在缓冲区里，舵机动都不动（细则 FAQ Q21 第 ⑤ 条）
            self._ser.flush()
            # 锁内留出帧间隔：startup_reset 或并发 goal 连续写入时，
            # 避免控制板还没解析完上一帧就收到下一帧。
            self.get_clock().sleep_for(Duration(seconds=SERIAL_FRAME_GAP_S))
            return True
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'串口写入失败：{exc}（USB 可能掉了）')
            self._serial_ok = False
            return False

    def _send(self, text):
        """发一条指令，不等回包。"""
        with self._serial_lock:
            if self._ser is None or not self._ser.is_open:
                return False
            return self._write_locked(text)

    def _query(self, text):
        """发一条查询指令，把回包原始字节拿回来（读不到就是 b''）。"""
        with self._serial_lock:
            if self._ser is None or not self._ser.is_open:
                return b''
            try:
                # 丢掉上一轮残留，不然读到的可能是上一条的回包
                self._ser.reset_input_buffer()
                if not self._write_locked(text):
                    return b''
                return self._ser.read_until(b'!') or b''
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'串口读取失败：{exc}')
                self._serial_ok = False
                return b''

    def _read_prad(self, servo_id):
        raw = self._query(build_prad_command(servo_id))
        pwm = parse_prad_reply(raw)
        self.get_logger().debug(f'PRAD id={servo_id} raw={raw!r} -> {pwm}')
        return pwm

    # -------------------------------------------- 唯一的底层发送入口（关键）
    def _dispatch(self, servo_id, pwm, duration_ms, publish_echo=True):
        """**Action 和 /servo/command 订阅都只能走这一个函数。**

        细则 2.7 原话：「它必须和 Action 走同一个底层发送函数 —— 两套逻辑的话，
        回放出来的动作和你录屏里的一定对不上。」

        publish_echo=True 时顺带把指令回显到 /servo/command，bag 才有东西可录。
        """
        applied = self._applied_pwm(servo_id, pwm)
        try:
            command = build_move_command(servo_id, applied, duration_ms)
        except ProtocolError as exc:
            self.get_logger().error(
                f'指令拼不出来：{exc}（yaml {int(pwm)} + offset '
                f'{self._joint_offset(servo_id):+d} = {applied}）')
            return False

        self._last_id = int(servo_id)
        self._last_pwm = applied

        if publish_echo:
            echo = ServoCommand()
            echo.id = int(servo_id)
            # 回显写的是 yaml 里的原始值；回放时 _dispatch 会再加一次 offset，
            # 所以录进去什么、放出来就是什么，闭环一致。
            echo.pwm = int(pwm)
            echo.duration_ms = int(duration_ms)
            self._command_pub.publish(echo)

        ok = self._send(command)
        offset = self._joint_offset(servo_id)
        suffix = f'（yaml {int(pwm)} + offset {offset:+d}）' if offset else ''
        self.get_logger().info(f'[{"OK" if ok else "FAIL"}] -> {command}{suffix}')
        self._phase = PHASE_MOVING if ok else PHASE_ERROR
        return ok

    def _torque_on(self, servo_id):
        """恢复扭力。不发这条，后面所有转指令舵机都不动 —— 任务 1 的一号坑。"""
        ok = self._send(build_torque_command(servo_id, True))
        if ok:
            self._torque_enabled = True
            self._phase = PHASE_TORQUE_ON
            self.get_logger().info(f'PULR id={servo_id} 恢复扭力')
        return ok

    def _torque_off_all(self):
        """全部卸力（看门狗触发 / torque_enable=false / 下电前用）。"""
        for servo_id in self._servo_ids():
            self._send(build_torque_command(servo_id, False))
        self._torque_enabled = False
        self.get_logger().warn(f'PULK：servo_ids={self._servo_ids()} 全部卸力')

    def _startup_reset(self):
        """启动时把 servo_ids 里所有舵机都回到安全区间中点。

        顺序：先 PULR 上力，再发 P —— 反过来的话舵机没力，转不动。
        """
        if not self._serial_ok:
            self.get_logger().warn('串口没通，跳过 startup_reset')
            return
        mid = self._mid_pwm()
        duration = self._default_duration()
        for servo_id in self._servo_ids():
            self._torque_on(servo_id)
            self._dispatch(servo_id, mid, duration)
        self.get_logger().info(f'startup_reset 完成：所有舵机 -> {mid}')

    # ------------------------------------------------------------ 状态广播
    def _publish_state(self):
        msg = ServoState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.serial_online = bool(self._serial_ok)
        msg.port = str(self.get_parameter('port').value)
        msg.last_id = self._last_id
        msg.last_pwm = self._last_pwm
        msg.phase = self._phase
        self._state_pub.publish(msg)

    # --------------------------------------- /servo/command 订阅（bag 回放用）
    def _on_command(self, msg):
        """bag 回放时，ros2 bag play 往 /servo/command 发消息，这里把动作重做一遍。

        为什么默认不执行（replay_mode=false）：
        驱动自己也往 /servo/command 回显指令。如果默认就执行，就会「自己发、自己收、
        执行两遍」。所以用 replay_mode 参数把两条路彻底分开：
            录制时 replay_mode=false -> 只回显，不执行
            回放时 replay_mode=true  -> 只执行订阅到的消息，并拒绝 action goal
        """
        if not bool(self.get_parameter('replay_mode').value):
            self.get_logger().debug(
                f'[回显] id={msg.id} pwm={msg.pwm} dur={msg.duration_ms}'
                f'（replay_mode=false，不执行）')
            return

        self.get_logger().info(
            f'[回放] /servo/command id={msg.id} pwm={msg.pwm} dur={msg.duration_ms}')
        try:
            self._dispatch(int(msg.id), int(msg.pwm), int(msg.duration_ms),
                           publish_echo=False)
        except ProtocolError as exc:
            self.get_logger().error(f'回放消息非法：{exc}')

    # ------------------------------------------------------ Action 四个回调
    def goal_callback(self, goal_request):
        """① 发指令**之前**就判断，不合格直接拒。"""
        if bool(self.get_parameter('replay_mode').value):
            self.get_logger().warn('replay_mode=true，不接受 action goal（正在回放 bag）')
            return GoalResponse.REJECT

        reason = self._validate_goal(goal_request)
        if reason:
            self.get_logger().warn(f'goal 被拒绝：{reason}')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'goal 通过校验：id={goal_request.id} pwm={goal_request.pwm} '
            f'duration_ms={goal_request.duration_ms}')
        return GoalResponse.ACCEPT

    def _validate_goal(self, goal):
        """校验用的全是**实时读出来的参数** —— 所以 ros2 param set 当帧生效。"""
        pwm_min = self._pwm_min()
        pwm_max = self._pwm_max()
        servo_ids = self._servo_ids()

        if int(goal.id) not in servo_ids:
            return f'id={goal.id} 不在 servo_ids={servo_ids} 里'
        applied = self._applied_pwm(goal.id, goal.pwm)
        if not pwm_min <= applied <= pwm_max:
            offset = self._joint_offset(goal.id)
            extra = f'（yaml {int(goal.pwm)} + offset {offset:+d}）' if offset else ''
            return (f'pwm={goal.pwm}{extra} -> 实际 {applied} 超出安全区间 '
                    f'[{pwm_min}, {pwm_max}]（现场可 ros2 param set 放宽 pwm_max）')
        duration = int(goal.duration_ms)
        if duration == 0:
            duration = self._default_duration()
        if duration <= 0:
            return 'duration_ms 和 default_duration_ms 都不合法'
        if duration > 9999:
            return (f'duration_ms={goal.duration_ms} 太大：协议里 T 只有 4 位，'
                    f'上限 9999 ms')
        return ''

    def cancel_callback(self, goal_handle):
        """② 取消请求：一律接受，真正的收尾在 execute_callback 的循环里做。"""
        self.get_logger().warn('收到取消请求 -> CancelResponse.ACCEPT')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """③ 显式执行。

        ⚠️ 自己实现 handle_accepted_callback 之后，**必须**调用 goal_handle.execute()，
        否则 execute_callback 永远不会跑 —— 客户端看到 goal accepted 但永远没 result
        （细则 FAQ Q13 就是这个现象）。
        """
        self.get_logger().info('goal 已接受 -> 交给 execute_callback')
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        """④ 真正干活：上力 -> 发指令 -> 循环 feedback -> 检查取消 -> PRAD 校验。"""
        return self._execute(goal_handle)

    def _execute(self, goal_handle):
        goal = goal_handle.request
        servo_id = int(goal.id)

        result = MoveServo.Result()
        result.success = False
        result.final_pwm = 0
        result.retry_count = 0

        target = int(goal.pwm)
        # PRAD 闭环要比对的是**真正发出去的值**，不是 yaml 里的原始值
        applied = self._applied_pwm(servo_id, target)
        duration_ms = int(goal.duration_ms) or self._default_duration()

        # ---- 0) 前置检查 ----
        if not self._serial_ok:
            result.message = ('串口未连接（查 port 参数 / ls -l /dev/tty* / '
                              'dialout 组要重新登录）')
            self.get_logger().error(result.message)
            self._phase = PHASE_ERROR
            goal_handle.abort()
            return result

        if self._watchdog_tripped:
            result.message = ('看门狗已触发，全部舵机已卸力；确认安全后 '
                              'ros2 param set /servo_driver torque_enable true 恢复')
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        # ---- 1) 起点位置（读不到就按中位估进度）----
        start_pwm = self._read_prad(servo_id)
        if start_pwm is None:
            start_pwm = self._mid_pwm()
            self.get_logger().warn(
                f'读不到 id={servo_id} 的 PRAD，feedback 进度按中位 {start_pwm} 估算')

        # ---- 2) 上力 ----
        if not self._torque_on(servo_id):
            result.message = 'PULR 恢复扭力失败（串口写不出去）'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        # ---- 3) 下发动作 ----
        if not self._dispatch(servo_id, target, duration_ms):
            result.message = '串口写入失败（USB 是不是掉了）'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        # ---- 4) feedback 循环（取消就在这个循环里生效）----
        period = self._feedback_period()
        duration_ns = duration_ms * 1_000_000
        start_ns = self.get_clock().now().nanoseconds
        self._watchdog_arm(duration_ns)

        feedback = MoveServo.Feedback()
        while True:
            if goal_handle.is_cancel_requested:
                return self._finish_canceled(goal_handle, servo_id, result)

            if self._watchdog_tripped:
                result.message = '看门狗触发：动作推进停滞，已全部卸力'
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            elapsed_ns = self.get_clock().now().nanoseconds - start_ns
            progress = 1.0 if duration_ns <= 0 else min(1.0, elapsed_ns / duration_ns)

            feedback.current_pwm = int(round(start_pwm + (applied - start_pwm) * progress))
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)
            self._publish_state()

            if progress >= 1.0:
                break

            # 这里的 sleep 会让出 GIL，但 executor 必须还是 MultiThreadedExecutor，
            # 否则没有任何线程去跑 cancel_callback（细则 FAQ Q11）。
            self.get_clock().sleep_for(period)
            self._watchdog_arm(
                duration_ns - (self.get_clock().now().nanoseconds - start_ns))

        # ---- 5) PRAD 闭环校验（加分项 +3）----
        tolerance = int(self.get_parameter('prad_tolerance').value)
        retries = int(self.get_parameter('prad_retries').value)
        final_pwm = self._read_prad(servo_id)

        attempt = 0
        while (final_pwm is not None
               and abs(final_pwm - applied) > tolerance
               and attempt < retries):
            attempt += 1
            self.get_logger().warn(
                f'[PRAD] 未到位：目标 {applied}，实测 {final_pwm}，'
                f'差 {abs(final_pwm - applied)} > {tolerance} -> 第 {attempt} 次重发')
            # 重发之后多等一点余量再读 PRAD，不然读到的还是上一帧的位置
            self._watchdog_arm((duration_ms + 300) * 1_000_000)
            self._dispatch(servo_id, target, duration_ms)
            self.get_clock().sleep_for(Duration(seconds=duration_ms / 1000.0))
            final_pwm = self._read_prad(servo_id)

        result.retry_count = attempt

        if final_pwm is None:
            result.message = '动作已下发，但 PRAD 读不回来（查串口 / 回包格式）'
            self.get_logger().error(result.message)
            self._watchdog_disarm()
            goal_handle.abort()
            return result

        result.final_pwm = int(final_pwm)
        diff = abs(final_pwm - applied)
        self._watchdog_disarm()

        if diff <= tolerance:
            result.success = True
            result.message = (f'到位：目标 {applied}，PRAD 读回 {final_pwm}，'
                              f'误差 {diff} <= {tolerance}')
            self.get_logger().info(f'[PRAD] {result.message}（重发 {attempt} 次）')
            self._phase = PHASE_IDLE
            goal_handle.succeed()
            return result

        result.message = (f'没到位：目标 {applied}，PRAD 读回 {final_pwm}，'
                          f'差 {diff} > {tolerance}（已重发 {attempt} 次）')
        self.get_logger().warn(f'[PRAD] {result.message}')
        self._phase = PHASE_ERROR
        goal_handle.abort()
        return result

    def _finish_canceled(self, goal_handle, servo_id, result):
        """取消收尾：立刻停发 -> 回中位 -> canceled()。

        细则 2.3 原文：「收到取消 -> 立刻停发、回中位、goal_handle.canceled()，
        result 里 success=False, message='canceled'」。
        """
        mid = self._mid_pwm()
        self.get_logger().warn(f'[取消] id={servo_id} 立刻回中位 {mid}')
        self._dispatch(servo_id, mid, self._default_duration())
        self._watchdog_disarm()

        result.success = False
        result.final_pwm = int(mid)
        result.message = 'canceled'
        self._phase = PHASE_CANCELED
        goal_handle.canceled()
        return result

    # -------------------------------------------------------- 看门狗（+3）
    def _watchdog_arm(self, remaining_ns):
        """把「允许的最晚推进时刻」往后推。

        正常循环每 200 ms 推一次；一旦某处卡死（串口阻塞、死锁），
        10 Hz 的定时器就会发现时间过了 deadline，然后全部卸力。
        """
        if self._watchdog_tripped:
            return
        timeout_ns = int(float(self.get_parameter('watchdog_timeout_s').value) * 1e9)
        self._watchdog_deadline_ns = (
            self.get_clock().now().nanoseconds + max(0, remaining_ns) + timeout_ns)

    def _watchdog_disarm(self):
        self._watchdog_deadline_ns = None

    def _tick_safety(self):
        """10 Hz：处理「卸力」和「看门狗恢复」两类外部请求。"""
        # 外部要求卸力（ros2 param set /servo_driver torque_enable false）
        if self._want_torque_off:
            self._want_torque_off = False
            if not self._watchdog_tripped:
                self._torque_off_all()
                self._phase = PHASE_IDLE

        # 看门狗触发后，必须外部确认才允许恢复
        if self._want_manual_recovery:
            self._want_manual_recovery = False
            if self._watchdog_tripped:
                self._watchdog_tripped = False
                self._phase = PHASE_TORQUE_ON
                for servo_id in self._servo_ids():
                    self._torque_on(servo_id)
                self.get_logger().warn(
                    '看门狗已由外部确认恢复（torque_enable=true），舵机重新上力')

        if self._watchdog_tripped or self._watchdog_deadline_ns is None:
            return
        if self.get_clock().now().nanoseconds > self._watchdog_deadline_ns:
            self._watchdog_tripped = True
            self._watchdog_disarm()
            self._phase = PHASE_ERROR
            self.get_logger().error(
                '看门狗触发：动作推进超过 '
                f'{self.get_parameter("watchdog_timeout_s").value}s 没有进展，'
                '全部舵机卸力！恢复需要外部确认：'
                'ros2 param set /servo_driver torque_enable true')
            self._torque_off_all()
            self._publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    # 必须多线程：同一 step 的每个 goal 都要能独立推进，不能因线程数不足排队。
    # 额外留 2 个线程给取消、参数和定时器回调；至少保留原来的 4 线程。
    worker_count = max(4, len(node._servo_ids()) + 2)
    node.get_logger().info(
        f'MultiThreadedExecutor 线程数={worker_count}（servo_ids={len(node._servo_ids())} + 2）')
    executor = MultiThreadedExecutor(num_threads=worker_count)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
