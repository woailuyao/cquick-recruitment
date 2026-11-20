#!/usr/bin/env python3
"""cquick_servo 驱动节点：把 ROS2 Action 翻译成总线舵机 ASCII 指令。

PWM 约定（任务3整机）：
  * goal、/servo/command、home_pwm 和动作 YAML 都是「逻辑目标值」；
  * 实际发送值 = 逻辑目标值 + joint_offsets[ID-1]；
  * 限位校验和 PRAD 到位判断都使用实际发送值；
  * feedback.current_pwm 与 result.final_pwm 是舵机实际读回值。

安全设计：
  * 所有运动都经过 _issue_move()，不会漏加或重复加 offset；
  * 动作中 PRAD 查询在整条总线上统一限频；
  * 任一 goal 可由播放器做到失败即停；
  * torque_enable=false 立即全部卸力并拒绝新动作；
  * 看门狗检测执行停滞，触发后全部卸力。
"""
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

# ---- 协议常量（不是配置，可以留在代码里）----
CMD_READ_ID = 'PID'
CMD_READ_PWM = 'PRAD'
CMD_TORQUE_ON = 'PULR'
CMD_TORQUE_OFF = 'PULK'

# ---- 电气与算法常量 ----
PWM_ABS_MIN = 500          # 协议/电气极限，不等于机械安全行程
PWM_ABS_MAX = 2500
OFFSET_ABS_MAX = 150       # 超过它应调整舵臂齿位，而不是继续用软件硬补
BAUD_MIN = 1200
BAUD_MAX = 3000000
POSITION_TOLERANCE = 30
FEEDBACK_PERIOD = 0.1      # 10 Hz，满足 >=5 Hz 的要求
BUS_POLL_PERIOD = 0.25     # 全总线共用查询间隔，避免多 goal 叠加成高频轮询
DURATION_LIMIT_MS = 10000


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # 基础参数 + 整机标定/保护参数，全部外置
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)
        self.declare_parameter('home_pwm', 1500)       # 逻辑中位，不是直接发送值
        self.declare_parameter('joint_offsets', [0, 0, 0, 0, 0, 0])
        self.declare_parameter('prad_retries', 0)      # 额外重读次数；0 = 只读一次
        self.declare_parameter('torque_enable', True)
        self.declare_parameter('watchdog_timeout_s', 3.0)  # 0 = 关闭看门狗

        self._lock = threading.Lock()          # 串口锁
        self._motion_lock = threading.Lock()   # 加力/卸力原子锁，固定先于串口锁获取
        self._poll_lock = threading.Lock()
        self._activity_lock = threading.Lock()
        self._suppress_lock = threading.Lock()
        self._last_bus_poll = 0.0
        self._goal_activity = {}       # servo_id -> 最近推进时间；同 ID goal 已互斥
        self._motion_inhibit = not bool(self.get_parameter('torque_enable').value)
        self._servo_locks = {servo_id: threading.Lock() for servo_id in range(256)}
        self._serial = None
        self._suppressed = []      # [(逻辑指令签名, 时间戳)]，吃掉自己发的回声
        self._last_pwm = {}

        self._pub_command = self.create_publisher(ServoCommand, '/servo/command', 10)
        self._pub_state = self.create_publisher(ServoCommand, '/servo/state', 10)

        self._open_serial()

        group = ReentrantCallbackGroup()
        self.create_subscription(
            ServoCommand, '/servo/command', self._on_command, 10, callback_group=group)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=group,
        )
        self.create_timer(1.0, self._heartbeat, callback_group=group)
        self.create_timer(0.5, self._watchdog, callback_group=group)

        self.get_logger().info(
            f'servo_driver ready  port={self.get_parameter("port").value} '
            f'baud={self.get_parameter("baudrate").value} '
            f'actual_safe=[{self.get_parameter("pwm_min").value}, '
            f'{self.get_parameter("pwm_max").value}] '
            f'ids={list(self.get_parameter("servo_ids").value)} '
            f'offsets={list(self.get_parameter("joint_offsets").value)} '
            f'startup_reset={self.get_parameter("startup_reset").value}')

        if bool(self.get_parameter('startup_reset').value):
            self.reset_all()

    # ------------------------------------------------------------------
    # 串口层
    # ------------------------------------------------------------------
    def _open_serial(self):
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)
        try:
            self._serial = serial.Serial(port, baud, timeout=0.2, write_timeout=1.0)
            self.get_logger().info(f'串口已打开 {port} @ {baud}')
            return True
        except serial.SerialException as exc:
            self._serial = None
            self.get_logger().error(f'打开串口失败 {port}: {exc}')
            return False

    def _send_raw(self, cmd, wait=0.05):
        """唯一底层串口函数；cmd 必须是已经计算好的实际协议内容。"""
        with self._lock:
            if self._serial is None or not self._serial.is_open:
                if not self._open_serial():
                    return None
            try:
                self._serial.reset_input_buffer()
                self._serial.write(cmd.encode('ascii'))
                self._serial.flush()
                time.sleep(wait)
                size = self._serial.in_waiting
                rx = self._serial.read(size).decode('ascii', 'replace') if size else ''
                return rx.strip()
            except serial.SerialException as exc:
                self.get_logger().error(f'串口异常: {exc}')
                try:
                    self._serial.close()
                except serial.SerialException:
                    pass
                self._serial = None
                return None

    def _read_position(self, servo_id, retries=None, throttle=False):
        """读实际位置。prad_retries 是额外重读次数，0 仍读取一次。"""
        if retries is None:
            retries = int(self.get_parameter('prad_retries').value) + 1

        if throttle:
            with self._poll_lock:
                now = time.monotonic()
                if now - self._last_bus_poll < BUS_POLL_PERIOD:
                    return self._last_pwm.get(servo_id)
                self._last_bus_poll = now
            retries = 1             # 动作中的一次空读留给下一轮，不连续抢总线

        for _ in range(max(1, retries)):
            rx = self._send_raw(f'#{servo_id:03d}{CMD_READ_PWM}!', wait=0.08)
            if not rx:
                continue
            match = re.search(r'P(\d+)', rx)
            if match:
                value = int(match.group(1))
                self._last_pwm[servo_id] = value
                return value
        return None

    def _issue_move(self, servo_id, logical_pwm, duration_ms, publish=True):
        """校验逻辑目标 → 加一次 offset → 原子发送；失败返回 None。

        _motion_lock 覆盖校验、PULR 和 PWM 全程。release_all 也持有同一把锁，
        因此卸力之后不可能被已经开始的另一条运动重新 PULR。
        """
        with self._motion_lock:
            ok, reason, applied_pwm = self._validate(servo_id, logical_pwm, duration_ms)
            if not ok:
                self.get_logger().warning(f'拒绝发送 id={servo_id}: {reason}')
                return None

            # 执行类指令成功时返回 ''，串口异常才返回 None；两者都必须检查。
            if self._send_raw(f'#{servo_id:03d}{CMD_TORQUE_ON}!', wait=0.01) is None:
                self.get_logger().error(f'PULR 发送失败 id={servo_id}，取消本次动作')
                return None
            if self._send_raw(
                    f'#{servo_id:03d}P{applied_pwm}T{duration_ms}!', wait=0.01) is None:
                self.get_logger().error(f'PWM 发送失败 id={servo_id}，本次动作不得判成功')
                # PULR 已成功时才可能走到这里；尽力卸力，仍需人工检查供电。
                self._send_raw(f'#{servo_id:03d}{CMD_TORQUE_OFF}!', wait=0.01)
                return None

            if publish:
                self._publish_command(servo_id, logical_pwm, duration_ms)
            return applied_pwm

    def _publish_command(self, servo_id, logical_pwm, duration_ms):
        """bag 中保存逻辑目标，回放时与 Action 走同一套 offset 和校验。"""
        msg = ServoCommand()
        msg.id = int(servo_id)
        msg.pwm = int(logical_pwm)
        msg.duration_ms = int(duration_ms)
        now = time.monotonic()
        with self._suppress_lock:
            self._suppressed = [(sig, ts) for sig, ts in self._suppressed
                                if now - ts < 3.0]
            self._suppressed.append(((msg.id, msg.pwm, msg.duration_ms), now))
        self._pub_command.publish(msg)

    def _publish_state(self, servo_id, pwm):
        msg = ServoCommand()
        msg.id = int(servo_id)
        msg.pwm = int(pwm)
        msg.duration_ms = 0
        self._pub_state.publish(msg)

    def _heartbeat(self):
        for servo_id, pwm in list(self._last_pwm.items()):
            self._publish_state(servo_id, pwm)

    # ------------------------------------------------------------------
    # 话题回放通道：msg.pwm 同样是逻辑目标
    # ------------------------------------------------------------------
    def _on_command(self, msg):
        # 与 Action 一致：duration_ms=0 表示使用 default_duration_ms。
        signature = (
            int(msg.id), int(msg.pwm),
            self._resolve_duration(int(msg.duration_ms)))
        now = time.monotonic()
        with self._suppress_lock:
            for index, (candidate, stamp) in enumerate(self._suppressed):
                if candidate == signature and now - stamp < 3.0:
                    # 每个本地发布只抵消一条回声；额外的同值外部消息仍会执行。
                    del self._suppressed[index]
                    return
        applied = self._issue_move(*signature, publish=False)
        if applied is not None:
            self.get_logger().info(
                f'[topic] replay id={signature[0]} logical={signature[1]} '
                f'applied={applied} dur={signature[2]}')

    # ------------------------------------------------------------------
    # 参数与目标校验
    # ------------------------------------------------------------------
    def _on_set_parameters(self, params):
        current = {name: self.get_parameter(name).value
                   for name in ('pwm_min', 'pwm_max', 'baudrate', 'home_pwm')}
        release_requested = False

        for param in params:
            if param.name in ('pwm_min', 'pwm_max', 'home_pwm') and not (
                    isinstance(param.value, int)
                    and PWM_ABS_MIN <= param.value <= PWM_ABS_MAX):
                return SetParametersResult(
                    successful=False,
                    reason=f'{param.name} 必须是 {PWM_ABS_MIN}~{PWM_ABS_MAX} 的整数')
            if param.name == 'baudrate' and not (
                    isinstance(param.value, int)
                    and BAUD_MIN <= param.value <= BAUD_MAX):
                return SetParametersResult(
                    successful=False,
                    reason=f'不支持的波特率 {param.value}')
            if param.name == 'joint_offsets':
                values = list(param.value or [])
                if not values or not all(
                        isinstance(value, int) and abs(value) <= OFFSET_ABS_MAX
                        for value in values):
                    return SetParametersResult(
                        successful=False,
                        reason=f'joint_offsets 必须是非空整数数组，每项绝对值 <= {OFFSET_ABS_MAX}')
            if param.name == 'prad_retries' and not (
                    isinstance(param.value, int) and 0 <= param.value <= 3):
                return SetParametersResult(
                    successful=False, reason='prad_retries 必须是 0~3 的整数')
            if param.name == 'watchdog_timeout_s' and not (
                    isinstance(param.value, (int, float))
                    and 0.0 <= float(param.value) <= 30.0):
                return SetParametersResult(
                    successful=False, reason='watchdog_timeout_s 必须在 0~30 秒')
            if param.name == 'torque_enable':
                if not isinstance(param.value, bool):
                    return SetParametersResult(
                        successful=False, reason='torque_enable 必须是 bool')
                release_requested = not param.value
            if param.name in current:
                current[param.name] = param.value

        if current['pwm_min'] >= current['pwm_max']:
            return SetParametersResult(
                successful=False,
                reason=f'pwm_min({current["pwm_min"]}) 必须小于 pwm_max({current["pwm_max"]})')

        if release_requested:
            # 先在 _motion_lock 内置 inhibit，再 PULK；参数生效前也不会重新加力。
            self.release_all()
        elif any(param.name == 'torque_enable' and param.value for param in params):
            with self._motion_lock:
                self._motion_inhibit = False
        return SetParametersResult(successful=True)

    def _validate(self, servo_id, logical_pwm, duration_ms):
        """返回 (是否合法, 原因, 实际PWM)；所有运动发送前必须调用。"""
        servo_ids = list(self.get_parameter('servo_ids').value)
        offsets = list(self.get_parameter('joint_offsets').value)
        pwm_min = int(self.get_parameter('pwm_min').value)
        pwm_max = int(self.get_parameter('pwm_max').value)
        index = int(servo_id) - 1
        applied_pwm = int(logical_pwm)

        if servo_id not in servo_ids:
            return False, f'unknown_id: {servo_id} 不在 servo_ids{servo_ids} 中', applied_pwm
        if self._motion_inhibit or not bool(self.get_parameter('torque_enable').value):
            return False, 'torque_disabled: 已卸力，拒绝新动作', applied_pwm
        if not 0 <= index < len(offsets):
            return False, f'missing_offset: ID {servo_id} 没有关节偏差', applied_pwm

        applied_pwm = int(logical_pwm) + int(offsets[index])
        if not pwm_min <= applied_pwm <= pwm_max:
            return False, (
                f'out_of_range: 逻辑 {logical_pwm} + offset {offsets[index]} '
                f'= 实际 {applied_pwm}，超出 [{pwm_min}, {pwm_max}]'), applied_pwm
        if not 0 < duration_ms <= DURATION_LIMIT_MS:
            return False, f'bad_duration: {duration_ms} 必须在 1~{DURATION_LIMIT_MS} ms', applied_pwm
        return True, 'ok', applied_pwm

    def _resolve_duration(self, duration_ms):
        if int(duration_ms) > 0:
            return int(duration_ms)
        return int(self.get_parameter('default_duration_ms').value)

    # ------------------------------------------------------------------
    # Action 回调
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request):
        duration = self._resolve_duration(goal_request.duration_ms)
        ok, reason, applied = self._validate(
            int(goal_request.id), int(goal_request.pwm), duration)
        if not ok:
            self.get_logger().warning(f'REJECT goal: {reason}')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'ACCEPT goal id={goal_request.id} logical={goal_request.pwm} '
            f'applied={applied} duration={duration}ms')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('goal accepted，开始执行')
        goal_handle.execute()

    @staticmethod
    def _make_result(success, message, final_pwm):
        result = MoveServo.Result()
        result.success = success
        result.message = message
        result.final_pwm = int(final_pwm)
        return result

    def execute_callback(self, goal_handle):
        servo_id = int(goal_handle.request.id)
        logical_target = int(goal_handle.request.pwm)
        duration = self._resolve_duration(goal_handle.request.duration_ms)

        ok, reason, _ = self._validate(servo_id, logical_target, duration)
        if not ok:
            goal_handle.abort()
            return self._make_result(False, reason, 0)

        # 同一舵机绝不允许两个 goal 同时执行；不同 ID 仍可并行协同。
        servo_lock = self._servo_locks[servo_id]
        if not servo_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._make_result(False, f'busy_id: 舵机 {servo_id} 正在执行其他 goal', 0)

        self._goal_started(servo_id)
        current = 0
        try:
            applied_target = self._issue_move(servo_id, logical_target, duration)
            if applied_target is None:
                goal_handle.abort()
                return self._make_result(False, 'send_failed: 指令未可靠发出', 0)

            self.get_logger().info(
                f'执行 id={servo_id} logical={logical_target} '
                f'applied={applied_target} duration={duration}ms')
            start = time.monotonic()
            span = duration / 1000.0
            poll_period = BUS_POLL_PERIOD
            last_poll = -poll_period
            current = self._read_position(servo_id, throttle=True) or 0

            while time.monotonic() - start < span:
                self._touch_activity(servo_id)

                if self._motion_inhibit or not bool(
                        self.get_parameter('torque_enable').value):
                    goal_handle.abort()
                    return self._make_result(False, 'torque_disabled', current)

                if goal_handle.is_cancel_requested:
                    home_logical = int(self.get_parameter('home_pwm').value)
                    self.get_logger().warning('取消：先卸力急停，再回校准中位')
                    if self._send_raw(
                            f'#{servo_id:03d}{CMD_TORQUE_OFF}!', wait=0.01) is None:
                        self.get_logger().error('取消路径 PULK 失败，升级为全部卸力；请检查供电')
                        self.release_all()
                        goal_handle.abort()
                        return self._make_result(
                            False, 'serial_error: 取消时 PULK 发送失败', current)
                    self._activity_sleep(servo_id, 0.5)
                    home_applied = self._issue_move(servo_id, home_logical, 1200)
                    if home_applied is None:
                        self.get_logger().error('取消回中发送失败，升级为全部卸力；不得标记正常取消')
                        self.release_all()
                        goal_handle.abort()
                        return self._make_result(
                            False, 'serial_error: 取消回中发送失败', current)
                    self._activity_sleep(servo_id, 1.8)
                    back = self._read_position(servo_id) or 0
                    goal_handle.canceled()
                    return self._make_result(False, 'canceled', back)

                elapsed = time.monotonic() - start
                if elapsed - last_poll >= poll_period:
                    current = self._read_position(servo_id, throttle=True) or current
                    last_poll = elapsed

                feedback = MoveServo.Feedback()
                feedback.current_pwm = int(current)
                feedback.progress = min(1.0, elapsed / span)
                goal_handle.publish_feedback(feedback)
                self._publish_state(servo_id, feedback.current_pwm)
                time.sleep(FEEDBACK_PERIOD)

            self._activity_sleep(servo_id, 0.4)
            final = self._read_position(servo_id)
            if final is None:
                goal_handle.abort()
                return self._make_result(False, 'serial_error: PRAD 读回失败', 0)

            # 关键：PRAD 与实际发送值比较，不与 YAML 逻辑值比较。
            diff = abs(final - applied_target)
            if diff > POSITION_TOLERANCE:
                self.get_logger().error(
                    f'未到位：id={servo_id} logical={logical_target} '
                    f'applied={applied_target} 读回={final} 差={diff}')
                goal_handle.abort()
                return self._make_result(
                    False,
                    f'not_reached: 实际目标 {applied_target} 读回 {final} 差 {diff}',
                    final)

            self.get_logger().info(
                f'到位：id={servo_id} logical={logical_target} '
                f'applied={applied_target} 读回={final}')
            goal_handle.succeed()
            return self._make_result(True, 'ok', final)
        finally:
            self._goal_finished(servo_id)
            servo_lock.release()

    # ------------------------------------------------------------------
    # 回中、卸力与看门狗
    # ------------------------------------------------------------------
    def reset_all(self):
        home_logical = int(self.get_parameter('home_pwm').value)
        for servo_id in list(self.get_parameter('servo_ids').value):
            applied = self._issue_move(int(servo_id), home_logical, 1200)
            if applied is None:
                self.get_logger().error(f'startup_reset 失败 id={servo_id}')
            else:
                self.get_logger().info(
                    f'startup_reset -> id {servo_id} logical={home_logical} applied={applied}')

    def release_all(self):
        """紧急卸力：先禁止一切新运动，再原子发送 PULK。"""
        with self._motion_lock:
            self._motion_inhibit = True
            self.get_logger().warning('全部舵机卸力 PULK，并禁止后续加力')
            for servo_id in list(self.get_parameter('servo_ids').value):
                if self._send_raw(
                        f'#{int(servo_id):03d}{CMD_TORQUE_OFF}!', wait=0.01) is None:
                    self.get_logger().error(f'PULK 发送失败 id={servo_id}，请直接断动力电')

    def _goal_started(self, servo_id):
        with self._activity_lock:
            self._goal_activity[servo_id] = time.monotonic()

    def _goal_finished(self, servo_id):
        with self._activity_lock:
            self._goal_activity.pop(servo_id, None)

    def _touch_activity(self, servo_id):
        with self._activity_lock:
            if servo_id in self._goal_activity:
                self._goal_activity[servo_id] = time.monotonic()

    def _activity_sleep(self, servo_id, seconds):
        """长等待分段刷新本 goal 时间，避免取消回中被看门狗误判。"""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._touch_activity(servo_id)
            if self._motion_inhibit:
                return
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

    def _watchdog(self):
        timeout = float(self.get_parameter('watchdog_timeout_s').value)
        if timeout <= 0.0 or self._motion_inhibit:
            return
        now = time.monotonic()
        with self._activity_lock:
            stalled = [
                (servo_id, now - stamp)
                for servo_id, stamp in self._goal_activity.items()
                if now - stamp > timeout]
        if stalled:
            self.get_logger().error(f'看门狗触发：停滞 goal={stalled}，全部卸力')
            # 先内部禁止并卸力，再同步参数；中间没有可重新加力的窗口。
            self.release_all()
            self.set_parameters([Parameter('torque_enable', value=False)])


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    # 一个 step 可同时下发 10 个 goal，线程数必须大于 10。
    executor = MultiThreadedExecutor(num_threads=16)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.release_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
