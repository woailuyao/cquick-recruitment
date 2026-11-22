"""
servo_driver.py — 舵机驱动节点（Action Server）

设计要点：
1. 对外只暴露 Action /move_servo；/servo/command 是给 bag 回放用的内部通道。
2. 四个回调全部接上：goal / cancel / execute，以及底层默认的 accepted。
3. ReentrantCallbackGroup + MultiThreadedExecutor：
   execute_callback 里循环发 feedback 会长时间占用一个线程，
   单线程 executor 下 cancel 回调没有机会执行 → is_cancel_requested 永远是 False。
   多线程 + 可重入组让 cancel 在另一个线程被处理，取消才能真正生效。
4. 所有配置（串口/波特率/限位/时长/ID 列表）都来自参数；
   每次判断都重新 get_parameter，不缓存副本，所以 `ros2 param set` 下一帧即生效。
5. 协议常量（# ! P T PID PRAD PULR PULK、ID 补 3 位）属于协议本身，允许写在代码里。
"""

import fcntl
import re
import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

# ---------------------------------------------------------------------------
# 参数合法取值域：约束的是"参数本身"，不是舵机机械限位。
# 用于拒绝 `ros2 param set pwm_max 100000` 这类把参数改坏的操作。
# 注意：VALID_BAUDRATES 是「允许被设置成哪些值」的白名单，
# 运行时真正用的波特率永远来自参数 baudrate，不来自这张表。
# ---------------------------------------------------------------------------
PARAM_ABSOLUTE_MIN = 500
PARAM_ABSOLUTE_MAX = 2500
MAX_DURATION_MS = 60000
VALID_BAUDRATES = (9600, 19200, 38400, 57600, 115200, 230400)

# ---------------------------------------------------------------------------
# 串口时序与判定容差
# ---------------------------------------------------------------------------
PRAD_RESPONSE_DELAY_S = 0.10     # 发完 PRAD 到读回显的等待（多舵机并发时 50ms 不够）
PRAD_READ_MARGIN = 100           # 回显里取数时的容差带，滤掉非位置字段的杂数
POSITION_TOLERANCE_PWM = 30      # 到位判定：|实际 - 目标| ≤ 30 视为到位
PRAD_SETTLE_S = 0.25             # 动作刚结束时舵机还在微调、总线没空，先静一下再读
PULR_SETTLE_S = 0.10             # PULR 与位置帧之间的间隔，背靠背发舵机会吞掉位置帧
PRAD_POLL_TIMEOUT_S = 2.0        # 舵机没走完时的最长等待
PRAD_POLL_INTERVAL_S = 0.15      # 轮询间隔
FEEDBACK_PERIOD_S = 0.2          # feedback 周期，200ms = 5Hz
FRAME_GAP_S = 0.04                # 帧间最小间隔（半双工总线）
STARTUP_RESET_DURATION_MS = 500  # 启动回中位的时长
CANCEL_RETURN_DURATION_MS = 300  # 取消后回中位的时长

# ---------------------------------------------------------------------------
# 协议常量（协议本身，不是可配置项）
# ---------------------------------------------------------------------------
FRAME_HEAD = '#'
FRAME_TAIL = '!'
ID_WIDTH = 3
CMD_POSITION = 'P'
CMD_DURATION = 'T'
CMD_READ_POSITION = 'PRAD'
CMD_TORQUE_ON = 'PULR'
CMD_TORQUE_OFF = 'PULK'


class NullSerial:
    """串口打不开时的降级实现：所有操作都变成空操作。

    这样 Action / feedback / 参数校验 / 取消逻辑在没有硬件时仍可完整演示。
    """

    is_open = False

    def write(self, data):
        return len(data)

    def flush(self):
        pass

    def read_all(self):
        return b''

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # ===== 7 个参数：只声明类型，不给默认值 =====
        # 取值必须来自 config/servo_params.yaml 或 launch 参数覆盖，
        # 代码里不出现设备名 / 波特率 / 限位的具体数字。
        self.declare_parameter('port', Parameter.Type.STRING)
        self.declare_parameter('baudrate', Parameter.Type.INTEGER)
        self.declare_parameter('pwm_min', Parameter.Type.INTEGER)
        self.declare_parameter('pwm_max', Parameter.Type.INTEGER)
        self.declare_parameter('default_duration_ms', Parameter.Type.INTEGER)
        self.declare_parameter('servo_ids', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('startup_reset', Parameter.Type.BOOL)

        # 注册参数回调：只有注册了才有权拒绝非法参数
        self.add_on_set_parameters_callback(self._on_parameters_set)

        # ===== 串口 =====
        # RLock：_send_command 会在持锁状态下调用 _send_frame，
        # 用普通 Lock 会自锁死。
        self._serial_lock = threading.RLock()

        # 自己发布到 /servo/command 的消息，会从自己的订阅回环回来一次。
        # 这里存「我刚发出去、还没收回来的那一条」，用来把回环和真正的 bag 回放区分开；
        # 不区分的话每条 goal 会在串口上发两遍，/servo/state 也会多发一条。
        self._echo_lock = threading.Lock()
        self._echo_pending = None
        self._simulated = False
        self._last_reopen = 0.0
        self._just_reconnected = False

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        if not port or not baudrate:
            raise RuntimeError(
                '参数未加载：必须通过 config/servo_params.yaml 或 launch 参数提供 port / baudrate')

        # 串口名漂移：CH340 拔插或 USB 重新挂载后可能从 ttyUSB0 变成 ttyUSB1。
        # 写死的名字打不开时如果直接进仿真模式，现象是「舵机不动、日志却一切正常」，
        # 极难查（本项目已经踩过两次）。这里先在给定端口上试，
        # 不通就扫其它 ttyUSB*/ttyACM*，挑能应答舵机协议的那个。
        port = self._resolve_port(port, baudrate)
        self._port = port
        self._baudrate = baudrate

        try:
            opened = serial.Serial(port, baudrate, timeout=0.05)
        except Exception as error:  # 硬件没接也能验证软件流程
            self._serial = NullSerial()
            self._simulated = True
            self.get_logger().warn(
                f'串口打开失败({error})，进入仿真模式：软件流程可演示，舵机不会动')
        else:
            # 同一串口只允许一个驱动实例持有。
            # 重复 ros2 launch 时两个进程会交替往串口写，帧被撕裂，现象是
            # 「goal 回来 success=False、PRAD 几乎没变、舵机不动或只抖一下」，
            # 客户端还会刷 'There may be more than one action server'。
            # 这里用非阻塞排他锁，第二个实例直接退出，把原因说清楚，
            # 而不是静默进入一个「看起来在跑、实际什么都没发生」的状态。
            try:
                fcntl.flock(opened.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                opened.close()
                raise RuntimeError(
                    f'串口 {port} 已被另一个 servo_driver 占用（多半是重复启动了 launch）。\n'
                    f'  先执行：pkill -f "[r]os2 launch"; pkill -f "[s]ervo_driver"\n'
                    f'  再查是谁占着：fuser -v {port}')
            self._serial = opened
            self.get_logger().info(f'串口已打开: {port} @ {baudrate}（已加独占锁）')

        # ===== 内部通道 =====
        # /servo/command 记录每条实际下发的指令，供 bag 录制与回放
        # /servo/state   发布每条指令的最终状态，供观测与 bag 录制
        self._command_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self._state_pub = self.create_publisher(ServoCommand, '/servo/state', 10)

        # 可重入回调组：动作执行与取消 / 回放订阅能并发
        callback_group = ReentrantCallbackGroup()

        # ===== Action Server =====
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group)

        # ===== bag 回放入口：与 Action 共用同一套发送函数 =====
        self._command_sub = self.create_subscription(
            ServoCommand, '/servo/command', self._on_command_replay, 10,
            callback_group=callback_group)

        # 启动回中位（中位由 pwm_min / pwm_max 算出，不硬编码）
        if self.get_parameter('startup_reset').value:
            for servo_id in self.get_parameter('servo_ids').value:
                self._send_command(servo_id, self._get_center_pwm(),
                                   STARTUP_RESET_DURATION_MS, record=False)
            self.get_logger().info('启动回中位完成')

    # ------------------------------------------------------------------
    # 参数读取：每次实时读，保证 param set 当帧生效
    # ------------------------------------------------------------------
    def _get_pwm_min(self):
        return int(self.get_parameter('pwm_min').value)

    def _get_pwm_max(self):
        return int(self.get_parameter('pwm_max').value)

    def _get_center_pwm(self):
        return (self._get_pwm_min() + self._get_pwm_max()) // 2

    def _get_servo_ids(self):
        return self.get_parameter('servo_ids').value

    def _on_parameters_set(self, params):
        """参数校验回调：返回 successful=False 则本次 set 整体被拒绝。"""
        for param in params:
            if param.name in ('pwm_min', 'pwm_max'):
                if param.type_ != Parameter.Type.INTEGER:
                    return SetParametersResult(
                        successful=False, reason=f'{param.name} 必须是整数')
                if not (PARAM_ABSOLUTE_MIN <= param.value <= PARAM_ABSOLUTE_MAX):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name}={param.value} '
                               f'超出 {PARAM_ABSOLUTE_MIN}~{PARAM_ABSOLUTE_MAX}')
            elif param.name == 'baudrate':
                if param.value not in VALID_BAUDRATES:
                    return SetParametersResult(
                        successful=False, reason=f'不支持的波特率 {param.value}')
            elif param.name == 'default_duration_ms':
                if not (0 < param.value <= MAX_DURATION_MS):
                    return SetParametersResult(
                        successful=False, reason='default_duration_ms 非法')

        # pwm_min 必须小于 pwm_max（用本次即将生效的值判断）
        pending_min = self.get_parameter('pwm_min').value
        pending_max = self.get_parameter('pwm_max').value
        for param in params:
            if param.name == 'pwm_min':
                pending_min = param.value
            elif param.name == 'pwm_max':
                pending_max = param.value
        if pending_min >= pending_max:
            return SetParametersResult(successful=False, reason='pwm_min 必须小于 pwm_max')

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # 底层收发：Action 与回放都走这里，保证两条路径行为一致
    # ------------------------------------------------------------------
    def _reopen_serial(self, reason):
        """串口掉线（Errno 5 EIO / Broken pipe）时重开一次。

        CH340 在 USB 抖动、舵机大电流干扰、或虚拟机的 USB 代理僵死时
        会把已打开的句柄变成死管。重开能救「瞬时抖动」，
        救不了「代理僵死」——那种情况重开也会报 EIO，那就明确失败，
        绝不能让节点悄悄退回仿真模式假装一切正常。
        """
        if self._simulated:
            return False
        # 冷却：多个 goal 同时收尾时，读 PRAD 失败会很密集；
        # 若每个失败都 close/open 一次，ch341 会在几秒内被折腾到 USB 掉线。
        # 5 秒内只允许真正重连一次，其余直接按失败返回。
        now = time.monotonic()
        if now - getattr(self, '_last_reopen', 0.0) < 5.0:
            return False
        self._last_reopen = now
        with self._serial_lock:
            try:
                self._serial.close()
            except Exception:
                pass
            last_error = None
            # 重试次数要克制：这段是同步阻塞的，跑在一个 goal 的线程里。
            # 上层 sequence_player 的等待窗口只有「时长 + 5 秒」，
            # 这里要是耗掉十几秒，上层只会看到「result 超时」，
            # 真正的原因（桥断了）被掩盖。4 次 × 0.5s ≈ 2s 足够救瞬时抖动。
            for _ in range(4):
                time.sleep(0.5)
                for cand in [self._port] + [
                        p for p in sorted(__import__('glob').glob('/dev/ttyUSB*')
                                          + __import__('glob').glob('/dev/ttyACM*'))
                        if p != self._port]:
                    try:
                        ser = serial.Serial(cand, self._baudrate, timeout=0.05)
                        fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except Exception as error:
                        last_error = error
                        continue
                    self._serial = ser
                    self._port = cand
                    self._just_reconnected = True
                    self.get_logger().warn(
                        f'串口掉线({reason})，已重连到 {cand}')
                    return True
            self.get_logger().error(
                f'串口重连失败({last_error})：如果是走 TCP 桥，先确认宿主 '
                f'bridge.py 还在跑、VM 里 socat 还在；再查 USB 线')
            # 关键：把死句柄置空，别让后面的每帧读都再跑一遍上面这 2 秒。
            # 置空后 _read_position 会立刻返回 None，goal 才能马上失败收尾。
            self._serial = None
            return False

    def _send_frame(self, text):
        """发送一帧协议文本，自动补 CRLF。发送失败时尝试重连一次。"""
        with self._serial_lock:
            payload = (text + '\r\n').encode()
            try:
                self._serial.write(payload)
                self._serial.flush()
                # 关键：帧与帧之间必须留间隔。
                # 无间隔连发时，10 台舵机会同时应答，总线瞬时拥挤，
                # CH340 会被打成 USB 重枚举（现象：动作跑到第 8 秒掉线）。
                time.sleep(FRAME_GAP_S)
            except (serial.SerialException, OSError) as error:
                if not self._reopen_serial(error):
                    raise
                self._serial.write(payload)
                self._serial.flush()

    def _send_command(self, servo_id, pwm, duration_ms, record=True):
        """PULR 恢复扭力 + 位置指令；record=True 时同时发布 /servo/command。"""
        with self._serial_lock:
            # 上电后首次动作前需要 PULR，指令幂等，每次动作前发一次最稳。
            # 两帧之间必须留间隔：背靠背发（间隔 1ms）时实测舵机不动，
            # PRAD 一直停在原位，看起来像「指令被忽略」——其实是控制板
            # 还没从 PULR 里出来，后面那条位置帧被吞了。
            self._send_frame(f'{FRAME_HEAD}{servo_id:0{ID_WIDTH}d}{CMD_TORQUE_ON}{FRAME_TAIL}')
            time.sleep(PULR_SETTLE_S)
            self._send_frame(
                f'{FRAME_HEAD}{servo_id:0{ID_WIDTH}d}'
                f'{CMD_POSITION}{pwm}{CMD_DURATION}{duration_ms}{FRAME_TAIL}')

        command = ServoCommand(id=servo_id, pwm=pwm, duration_ms=duration_ms)
        if record:
            # 记录实际下发的指令供 bag 录制；回放时不记录，避免自己发自己收的死循环
            with self._echo_lock:
                self._echo_pending = (servo_id, pwm, duration_ms)
            self._command_pub.publish(command)
        self._state_pub.publish(command)

    @staticmethod
    def _port_responds(port, baudrate):
        """这个 tty 上有没有舵机在应答。用来辨认漂移后的真实串口。"""
        try:
            ser = serial.Serial(port, baudrate, timeout=0.15)
        except Exception:
            return False
        try:
            time.sleep(0.15)
            ser.reset_input_buffer()
            ser.write(f'{FRAME_HEAD}001PID{FRAME_TAIL}'.encode())
            time.sleep(0.3)
            data = ser.read(64)
            # 回显形如 b'#001P!'，必须带帧头 + 三位 ID，才认作舵机
            return re.search(rb'#\d{3}', data) is not None
        except Exception:
            return False
        finally:
            try:
                ser.close()
            except Exception:
                pass

    def _resolve_port(self, port, baudrate):
        if self._port_responds(port, baudrate):
            return port
        import glob
        candidates = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        for cand in candidates:
            if cand == port:
                continue
            if self._port_responds(cand, baudrate):
                self.get_logger().warn(
                    f'串口名漂移：{port} 无应答，自动改用 {cand}'
                    f'（拔插后会变号，建议用 udev 规则固定成 /dev/servo）')
                return cand
        return port

    def _read_position(self, servo_id, retries=3):
        """发 PRAD 读实际位置；重试若干次仍无回显则返回 None。

        舵机刚走完动作的瞬间常常来不及应答，单次读失败很常见，
        所以这里必须重试，否则会被误判成「仿真模式」。

        多舵机并发时（一个 step 里 5~10 个 goal 同时收尾），
        总线上的回显可能属于别的舵机。所以这里做两件事：
          ① 发查询前清空接收缓冲，避免把上一次的残留当成这次的回显；
          ② 解析时校验回显里的 ID 必须是自己的（#007P1498!）。
        只抓数字不认 ID 的话，会读到别人的位置，日志看起来全绿，实际是错的。
        """
        pwm_lo = self._get_pwm_min() - PRAD_READ_MARGIN
        pwm_hi = self._get_pwm_max() + PRAD_READ_MARGIN
        pattern = re.compile(
            rb'#' + f'{servo_id:0{ID_WIDTH}d}'.encode() + rb'P(\d+)')

        # 句柄已经死了（早先的重连失败把它置空）：不要再对空对象发帧，
        # 那会抛 AttributeError 把整个 execute 回调炸掉，goal 永远不返回。
        if not self._simulated and (
                self._serial is None or not getattr(self._serial, 'is_open', False)):
            self.get_logger().error(f'读 id={servo_id} 前发现串口已断开')
            if not self._reopen_serial('读位置前句柄已坏'):
                return None

        for attempt in range(retries):
            try:
                with self._serial_lock:
                    self._serial.reset_input_buffer()
                    self._send_frame(
                        f'{FRAME_HEAD}{servo_id:0{ID_WIDTH}d}{CMD_READ_POSITION}{FRAME_TAIL}')
                    time.sleep(PRAD_RESPONSE_DELAY_S)
                    frame_bytes = self._serial.read_all()
            except (serial.SerialException, OSError) as error:
                # 读的过程中串口死了：重连一次再继续，别把「串口断了」
                # 当成「舵机没到位」写进 result。
                if not self._reopen_serial(error):
                    return None
                continue

            match = pattern.search(frame_bytes)
            if match:
                value = int(match.group(1))
                if pwm_lo <= value <= pwm_hi:
                    return value

            if attempt < retries - 1:
                time.sleep(PRAD_POLL_INTERVAL_S)
        return None

    def _on_command_replay(self, msg):
        """bag 回放：与 Action 走同一个发送函数，同样受参数限位约束。"""
        # 先判断这是不是自己刚发出去那条的回环：是就吃掉，别在串口上发第二遍。
        key = (msg.id, msg.pwm, msg.duration_ms)
        with self._echo_lock:
            if self._echo_pending == key:
                self._echo_pending = None
                return

        if msg.id not in self._get_servo_ids():
            self.get_logger().warn(f'忽略回放: ID {msg.id} 不在 servo_ids')
            return
        pwm = min(max(msg.pwm, self._get_pwm_min()), self._get_pwm_max())
        self.get_logger().info(
            f'收到 /servo/command 回放: id={msg.id} pwm={pwm} duration={msg.duration_ms}')
        self._send_command(msg.id, pwm, msg.duration_ms, record=False)

    # ------------------------------------------------------------------
    # Action 回调
    # ------------------------------------------------------------------
    def _goal_callback(self, goal_request):
        command = goal_request.command
        servo_id, pwm, duration_ms = command.id, command.pwm, command.duration_ms

        if servo_id not in self._get_servo_ids():
            self.get_logger().warn(f'拒绝 goal: ID {servo_id} 不在 servo_ids')
            return GoalResponse.REJECT

        if not (self._get_pwm_min() <= pwm <= self._get_pwm_max()):
            self.get_logger().warn(
                f'拒绝 goal: pwm {pwm} 超出限位 '
                f'{self._get_pwm_min()}~{self._get_pwm_max()}')
            return GoalResponse.REJECT

        if duration_ms > MAX_DURATION_MS:
            self.get_logger().warn(f'拒绝 goal: duration {duration_ms} 非法')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'接受 goal: id={servo_id} pwm={pwm} duration={duration_ms}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求，接受')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """execute 回调入口：任何异常都必须变成一条 result 返回。

        回调里抛异常时 goal 会停在 executing 状态不动，客户端只能靠自己超时，
        日志上就变成「等 result 超时（动作未结束）」——真实原因（串口断了、
        重连失败、内部报错）全被埋掉，排查方向直接跑偏。所以这里兜底。
        """
        try:
            return self._run_goal(goal_handle)
        except Exception as error:  # noqa: BLE001 - 兜底要抓全部
            result = MoveServo.Result()
            result.success = False
            result.final_pwm = int(getattr(goal_handle.request.command, 'pwm', 0))
            result.message = (f'驱动内部异常: {type(error).__name__}: {error}'
                              f'（先查串口链路：宿主 bridge.py / VM 内 socat）')
            self.get_logger().error(result.message)
            try:
                goal_handle.abort()
            except Exception:
                pass
            return result

    def _run_goal(self, goal_handle):
        command = goal_handle.request.command
        servo_id, target_pwm = command.id, command.pwm
        duration_ms = command.duration_ms or int(
            self.get_parameter('default_duration_ms').value)
        duration_s = max(duration_ms, 1) / 1000.0

        start_pwm = self._read_position(servo_id)
        if start_pwm is None:
            start_pwm = self._get_center_pwm()

        self._send_command(servo_id, target_pwm, duration_ms)

        # feedback 按时间线性插值给出"预计位置"，不是实时读回，
        # 所以字段名叫 current_pwm 但语义是估算值。
        feedback = MoveServo.Feedback()
        start_time = time.monotonic()
        while rclpy.ok() and not goal_handle.is_cancel_requested:
            elapsed_s = time.monotonic() - start_time
            progress = min(elapsed_s / duration_s, 1.0)
            feedback.current_pwm = int(start_pwm + (target_pwm - start_pwm) * progress)
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)
            if progress >= 1.0:
                break
            time.sleep(FEEDBACK_PERIOD_S)

        result = MoveServo.Result()

        # 取消：立刻回中位，结果标记 canceled
        if goal_handle.is_cancel_requested:
            center_pwm = self._get_center_pwm()
            self._send_command(servo_id, center_pwm, CANCEL_RETURN_DURATION_MS)
            result.success = False
            result.final_pwm = center_pwm
            result.message = 'canceled'
            goal_handle.canceled()
            self.get_logger().info('动作已取消，已回中位')
            return result

        # 到位校验：舵机走完行程要时间，轮询到进入容差带或超时。
        # 动作刚结束时舵机还在做最后的微调，串口也刚发完一整条 PWM 帧，
        # 这时候立刻查 PRAD 经常读空，先静一下再开始判定。
        time.sleep(PRAD_SETTLE_S)
        actual_pwm = self._read_position(servo_id, retries=4)
        deadline = time.monotonic() + PRAD_POLL_TIMEOUT_S
        while time.monotonic() < deadline:
            if (actual_pwm is not None
                    and abs(int(actual_pwm) - int(target_pwm)) <= POSITION_TOLERANCE_PWM):
                break
            time.sleep(PRAD_POLL_INTERVAL_S)
            actual_pwm = self._read_position(servo_id)

        note = ''
        if actual_pwm is None:
            if not self._simulated:
                # 串口刚掉线重连过：那条位置帧很可能没送达舵机，
                # 重发一次同样的目标再读，别把「掉线」记成「舵机没到位」。
                if self._just_reconnected:
                    self._just_reconnected = False
                    self.get_logger().warn(
                        f'串口刚重连，重发一次 id={servo_id} pwm={target_pwm}')
                    self._send_command(servo_id, target_pwm, duration_ms, record=False)
                    time.sleep(duration_ms / 1000.0 + PRAD_SETTLE_S + 0.2)
                    actual_pwm = self._read_position(servo_id, retries=4)
            if actual_pwm is None and not self._simulated:
                # 真机上读不到实测位置，绝不能拿目标值冒充：
                # 那样 final_pwm 永远等于 target、success 永远 true，
                # 等于替舵机签字，日志也就失去证据价值。
                result.final_pwm = int(target_pwm)
                result.success = False
                result.message = (
                    f'未确认到位: target={target_pwm} PRAD 多次无回显，'
                    f'无法判断是否到位（先查串口/USB 是否掉线）')
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result
            actual_pwm = target_pwm  # 仿真模式没有硬件，只演示流程
            note = ' (simulated: 串口未打开)'

        diff_pwm = abs(int(actual_pwm) - int(target_pwm))
        result.final_pwm = int(actual_pwm)
        if diff_pwm <= POSITION_TOLERANCE_PWM:
            result.success = True
            result.message = f'ok, PRAD={actual_pwm}{note}'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = (f'not in position: target={target_pwm} '
                              f'PRAD={actual_pwm} diff={diff_pwm}')
            goal_handle.abort()
        self.get_logger().info(f'动作结束: {result.message}')
        return result

    # ------------------------------------------------------------------
    def close_serial(self):
        """节点退出前：先给所有舵机 PULK 卸力，再释放串口。

        这一步对应任务四「安全下电」的第 ① 步。
        Ctrl+C 停节点 = 自动卸力，断电前舵机不会还顶着力矩。
        """
        try:
            ids = list(self.get_parameter('servo_ids').value)
            for servo_id in ids:
                self._send_frame(
                    f'{FRAME_HEAD}{servo_id:0{ID_WIDTH}d}{CMD_TORQUE_OFF}{FRAME_TAIL}')
            self.get_logger().info(
                f'安全下电第 ① 步完成：已对 {len(ids)} 台舵机发送 PULK 卸力'
                f'（断电前必须卸力，否则舵机断电时还在顶力矩，极易堵转发热）')
        except Exception as error:
            self.get_logger().warn(f'卸力时出错：{error}')
        try:
            self._serial.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    # 线程数必须 ≥ 一个 step 里的关节数：Action 的 execute 回调是阻塞的
    # （内部 time.sleep 等动作走完），线程不够时多出来的 goal 只能排队，
    # 表面上"同时下发"，实际是先后动。整机实扫 10 只舵机 → 给 12 个线程。
    executor = MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
