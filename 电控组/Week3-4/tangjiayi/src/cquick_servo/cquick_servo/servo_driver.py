#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""servo_driver —— CQuick 总线舵机驱动节点（Action Server）。

对外接口（只有 Action 是控制入口，话题只用来喂 bag 和看状态）
    Action  /move_servo              cquick_servo_interfaces/action/MoveServo   控制入口
    Topic   /servo/command           ServoCommand   订阅，内部调试总线 / bag 回放
    Topic   /servo/state             String         发布 5 Hz，状态 + 心跳
    Topic   /servo/heartbeat         Empty          订阅，上位机心跳，喂看门狗
    Service /servo/confirm_resume    Trigger        看门狗跳闸后的人工确认

线程模型（讲不清这个就会被问倒，README 里有专门的段落）
    MultiThreadedExecutor + ReentrantCallbackGroup
    - goal 校验（goal_callback）跑在 executor 线程，必须极快，只做算术
    - 每条 goal 在自己的线程里跑 execute()，这样 time.sleep 不会把 executor 占死，
      Ctrl+C 取消才进得来
    - 不同 ID 的 goal 并发执行（一个 step 里多个关节要同时动，靠的就是这个）
    - 同一 ID 的 goal 抢占：旧动作从当前位置退出，新 goal 接手，不回中位、不跳变
"""

from __future__ import annotations

import json
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from cquick_servo import servo_protocol as proto
from cquick_servo.serial_link import SerialLink, SerialLinkError
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

# ---------------------------------------------------------------- 接口名
# 名字属于「接口契约」，不是配置，所以写成常量而不是参数。
ACTION_NAME = 'move_servo'
TOPIC_COMMAND = '/servo/command'
TOPIC_STATE = '/servo/state'
TOPIC_HEARTBEAT = '/servo/heartbeat'
SRV_CONFIRM_RESUME = '/servo/confirm_resume'

# ---------------------------------------------------------------- 校验用常量
# 电气极限：超过这个范围舵机会堵转发热，任何情况下都不许下发。
PWM_ELECTRIC_MIN = 500
PWM_ELECTRIC_MAX = 2500
# 波特率白名单：不在里面的值直接拒，防止把串口设成一个谁也听不懂的速度。
SUPPORTED_BAUDRATES = (9600, 19200, 38400, 57600, 115200, 230400)
MAX_DURATION_MS = 60000

# 参数默认值表：名字只在这里出现一次（声明参数），读参数统一走 _param()
PARAM_DEFAULTS = [
    ('port', '/dev/cq_servo'),          # udev 固定设备名，详见 README 加分项
    ('baudrate', 115200),
    ('pwm_min', 900),
    ('pwm_max', 2100),
    ('default_duration_ms', 800),
    ('servo_ids', [1, 2, 3, 4]),
    ('startup_reset', True),
    # ↓ 以下是本实现为了「能回中位 / 能喂看门狗 / 能重连」补的参数，同样是配置
    ('pwm_center', 1500),
    ('center_duration_ms', 800),
    ('feedback_period_ms', 200),
    ('pos_tolerance', 30),
    ('retry_times', 1),
    ('watchdog_enabled', True),
    ('watchdog_timeout_ms', 1000),
    ('reconnect_period_ms', 1000),
    ('allow_preempt', True),
]


class ServoDriver(Node):
    """舵机驱动节点。"""

    def __init__(self) -> None:
        super().__init__('servo_driver')

        for name, default in PARAM_DEFAULTS:
            self.declare_parameter(name, default)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ---- 并发控制
        self._cb_group = ReentrantCallbackGroup()
        self._goal_lock = threading.Lock()
        self._running: Dict[int, bool] = {}       # 正在跑的 ID
        self._queued: Dict[int, list] = {}        # 被抢占后排队的那条 goal（按 ID 一个 FIFO）
        self._preempt: Dict[int, threading.Event] = {}
        self._moving: set = set()

        # ---- 链路
        self._link = SerialLink(self._param('port'), self._param('baudrate'))
        self._connected = False
        self._reload_link = False

        # ---- 看门狗
        self._watchdog_armed = False
        self._last_beat = 0.0
        self._locked = False

        # ---- 状态
        self._beat = 0
        self._state = 'init'
        self._shutting_down = False

        # ---- ROS 接口
        self._cmd_pub = self.create_publisher(ServoCommand, TOPIC_COMMAND, 10)
        self._state_pub = self.create_publisher(String, TOPIC_STATE, 10)
        self.create_subscription(
            ServoCommand, TOPIC_COMMAND, self._on_command_topic, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Empty, TOPIC_HEARTBEAT, self._on_heartbeat, 10,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, SRV_CONFIRM_RESUME, self._on_confirm_resume,
            callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            MoveServo,
            ACTION_NAME,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            handle_accepted_callback=self._handle_accepted_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.create_timer(0.2, self._on_state_timer, callback_group=self._cb_group)
        self.create_timer(0.1, self._on_watchdog_timer, callback_group=self._cb_group)
        self.create_timer(1.0, self._on_reconnect_timer, callback_group=self._cb_group)
        self._startup_timer = self.create_timer(
            0.5, self._on_startup_timer, callback_group=self._cb_group)

        self.get_logger().info(
            f'servo_driver 启动：action=/{ACTION_NAME} port={self._link.port} '
            f'baud={self._link.baudrate} 安全区间={self._limits()} '
            f'ids={self._servo_ids()}')

        # 先主动开一次串口：不然 launch 起来后头 1 秒内（重连定时器还没跑）
        # 任何 goal 都会被判定成 serial_down 而拒掉，演示时第一印象就废了。
        self._open_link(restore_torque=False)

    # ================================================================ 参数
    def _param(self, name: str):
        """读参数的唯一入口：每次实时读，绝不缓存到 self 上。

        缓存到 self.pwm_max 是本周最高频的坑 —— ros2 param set 返回成功，
        但代码还在用 __init__ 里那份副本，行为不变。
        """
        return self.get_parameter(name).value

    def _limits(self):
        return int(self._param('pwm_min')), int(self._param('pwm_max'))

    def _servo_ids(self) -> List[int]:
        return [int(i) for i in (self._param('servo_ids') or [])]

    def _center(self) -> int:
        return int(self._param('pwm_center'))

    def _default_duration(self) -> int:
        return int(self._param('default_duration_ms'))

    def _on_set_parameters(self, params) -> SetParametersResult:
        """参数校验回调。返回 successful=False 时整个 set 被拒，值保持原样。"""
        proposed = {p.name: p.value for p in params}

        def eff(name):
            """生效值：本次要改就用新值，否则用当前值。"""
            if name in proposed:
                return proposed[name]
            return self.get_parameter(name).value

        def fail(text):
            self.get_logger().warn(f'参数被拒：{text}')
            return SetParametersResult(successful=False, reason=text)

        # 1) 限位：既要落在电气极限内，又必须 min < max，且中位要落在安全区间里
        for name in ('pwm_min', 'pwm_max', 'pwm_center'):
            value = eff(name)
            if not isinstance(value, int):
                return fail(f'{name} 必须是整数，收到 {value!r}')
            if not PWM_ELECTRIC_MIN <= value <= PWM_ELECTRIC_MAX:
                return fail(f'{name}={value} 超出电气极限 '
                            f'{PWM_ELECTRIC_MIN}~{PWM_ELECTRIC_MAX}')
        pwm_min, pwm_max = int(eff('pwm_min')), int(eff('pwm_max'))
        if pwm_min >= pwm_max:
            return fail(f'pwm_min({pwm_min}) 必须小于 pwm_max({pwm_max})')
        center = int(eff('pwm_center'))
        if not pwm_min <= center <= pwm_max:
            return fail(f'pwm_center({center}) 必须落在 {pwm_min}~{pwm_max} 之间，'
                        f'否则回中位就超限了')

        # 2) 波特率白名单
        if eff('baudrate') not in SUPPORTED_BAUDRATES:
            return fail(f'baudrate={eff("baudrate")} 不在白名单 {list(SUPPORTED_BAUDRATES)} 里')

        # 3) 各种时长 / 阈值
        checks = [
            ('default_duration_ms', 1, MAX_DURATION_MS),
            ('center_duration_ms', 1, MAX_DURATION_MS),
            ('feedback_period_ms', 20, 1000),
            ('pos_tolerance', 0, 500),
            ('retry_times', 0, 5),
            ('watchdog_timeout_ms', 100, 60000),
            ('reconnect_period_ms', 100, 60000),
        ]
        for name, low, high in checks:
            value = eff(name)
            if not isinstance(value, int) or not low <= value <= high:
                return fail(f'{name}={value!r} 必须在 {low}~{high} 之间')

        # 4) 串口设备名不能空
        port = eff('port')
        if not isinstance(port, str) or not port.strip():
            return fail(f'port 不能为空：{port!r}')

        # 5) 舵机 ID 表
        ids = eff('servo_ids')
        if not isinstance(ids, list) or not ids:
            return fail(f'servo_ids 不能为空：{ids!r}')
        for value in ids:
            if not isinstance(value, int) or not proto.ID_MIN <= value <= proto.ID_MAX:
                return fail(f'servo_ids 里有非法 ID：{value!r}'
                            f'（合法范围 {proto.ID_MIN}~{proto.ID_MAX}）')

        # 6) 串口相关参数变了 -> 让重连定时器重建链路
        if 'port' in proposed or 'baudrate' in proposed:
            self._reload_link = True

        if proposed:
            self.get_logger().info(f'参数已更新：{proposed}（下一次判断立刻按新值走）')
        return SetParametersResult(successful=True)

    # ================================================================ 串口
    def _write_frame(self, frame: bytes) -> bool:
        """所有串口写出的唯一出口。失败只标记掉线，绝不抛给调用方炸节点。"""
        try:
            self._link.send(frame)
            return True
        except SerialLinkError as exc:
            self._mark_disconnected(str(exc))
            return False

    def _mark_disconnected(self, reason: str) -> None:
        if self._connected:
            self.get_logger().warn(
                f'串口断开：{reason}；节点继续运行，插回去会自动重连',
                throttle_duration_sec=5.0)
        self._connected = False

    def _send_move(self, servo_id: int, pwm: int, duration_ms: int,
                   echo: bool = True) -> bool:
        """下发一条「转角度」指令。

        echo=True 时把这条指令原样发到 /servo/command，让 bag 里留下「我到底发了什么」。
        /servo/command 的订阅回调走 _write_frame 而不是本函数 —— 否则订阅自己发的消息，
        一个字节回流就无限循环。
        """
        ok = self._write_frame(proto.build_move_command(servo_id, pwm, duration_ms))
        if ok and echo:
            msg = ServoCommand()
            msg.id = int(servo_id)
            msg.pwm = int(pwm)
            msg.duration_ms = int(duration_ms)
            self._cmd_pub.publish(msg)
        return ok

    def _torque_on(self, servo_id: int) -> bool:
        return self._write_frame(proto.build_torque_on(servo_id))

    def _torque_off(self, servo_id: int) -> bool:
        return self._write_frame(proto.build_torque_off(servo_id))

    def _read_position(self, servo_id: int, timeout: float = 0.15) -> Optional[int]:
        if not self._connected:
            return None
        try:
            return self._link.read_position(servo_id, timeout)
        except SerialLinkError as exc:
            self._mark_disconnected(str(exc))
            return None

    # ================================================================ Action: 校验
    def _goal_cb(self, goal_request):
        """发指令之前就判断掉。这里只能做算术，不能碰串口，否则会卡住 executor。"""
        servo_id = int(goal_request.id)
        pwm = int(goal_request.pwm)
        duration = int(goal_request.duration_ms)
        pwm_min, pwm_max = self._limits()
        reason = ''

        if self._locked:
            reason = f'watchdog_locked：看门狗已卸力，先调 {SRV_CONFIRM_RESUME} 确认'
        elif not self._connected:
            reason = 'serial_down：串口未连接'
        elif servo_id not in self._servo_ids():
            reason = f'id {servo_id} 不在 servo_ids {self._servo_ids()} 里'
        elif not pwm_min <= pwm <= pwm_max:
            reason = (f'pwm {pwm} 超出安全区间 {pwm_min}~{pwm_max}'
                      f'（pwm_min/pwm_max，可 ros2 param set 在线改）')
        elif duration > MAX_DURATION_MS:
            reason = f'duration_ms {duration} 超过上限 {MAX_DURATION_MS}'

        if reason:
            self.get_logger().warn(
                f'goal REJECTED (id={servo_id}, pwm={pwm}, duration={duration}ms)：{reason}')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'goal accepted (id={servo_id}, pwm={pwm}, duration={duration}ms)')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn(
            f'收到取消请求 (id={goal_handle.request.id})，立刻接受')
        return CancelResponse.ACCEPT

    # ================================================================ Action: 调度
    def _handle_accepted_cb(self, goal_handle) -> None:
        """谁在跑、谁排队、谁被抢占，都在这里决定。

        一条铁律：**每条被 ACCEPT 的 goal 都必须跑一次 execute_callback**，
        否则客户端永远等不到 result（rclpy 只在回调返回后才发布结果）。
        所以抢占不做「丢弃旧 goal」，而是让旧 goal 快速返回 preempted。
        """
        servo_id = int(goal_handle.request.id)
        with self._goal_lock:
            event = self._preempt.setdefault(servo_id, threading.Event())
            if servo_id in self._running:
                self._queued.setdefault(servo_id, []).append(goal_handle)
                if self._param('allow_preempt'):
                    event.set()  # 让正在跑的那条尽快退出
                    self.get_logger().warn(
                        f'id {servo_id} 正在动，新 goal 抢占：旧动作从当前位置退出，'
                        f'新 goal 排队接手（不回中位、不跳变）')
                self.get_logger().info(
                    f'id {servo_id} 的 goal 已排队，等当前动作让位')
                return
            event.clear()
            self._running[servo_id] = True
        threading.Thread(
            target=self._run_goal, args=(goal_handle,), daemon=True,
            name=f'goal-{servo_id}').start()

    def _run_goal(self, goal_handle) -> None:
        """在自己的线程里跑 execute()，不同 ID 天然并发。"""
        servo_id = int(goal_handle.request.id)
        try:
            if goal_handle.is_active:
                goal_handle.execute()
        except Exception as exc:  # noqa: BLE001 - 任何异常都不许把节点带走
            self.get_logger().error(f'goal 执行异常 (id={servo_id})：{exc}')
            if goal_handle.is_active:
                goal_handle.abort()
        finally:
            next_goal = None
            with self._goal_lock:
                self._running.pop(servo_id, None)
                pending = self._queued.get(servo_id) or []
                if pending:
                    # FIFO：接着跑最早排队的那条。如果它后面还有更新的 goal，
                    # 它会在 _execute_cb 开头立刻返回 preempted（一条指令都不发），
                    # 把控制权让给最后那条 —— 这样每条 goal 都能拿到 result。
                    next_goal = pending.pop(0)
                    self._running[servo_id] = True
                    self._preempt.setdefault(servo_id, threading.Event()).clear()
            if next_goal is not None:
                threading.Thread(
                    target=self._run_goal, args=(next_goal,), daemon=True,
                    name=f'goal-{servo_id}').start()

    # ================================================================ Action: 执行
    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        servo_id = int(req.id)
        target = int(req.pwm)
        duration = int(req.duration_ms) or self._default_duration()
        result = MoveServo.Result()

        # 0) 还没开始就被取消了
        if goal_handle.is_cancel_requested:
            self.get_logger().warn(f'goal 在开始前就被取消 (id={servo_id})')
            goal_handle.canceled()
            result.success = False
            result.pwm_readback = 0
            result.message = 'canceled'
            return result

        # 1) 排队期间已经被更新的 goal 顶掉了：一条指令都不发，直接交差
        #    （必须给客户端一个 result，否则它会一直等；所以是「快速失败」不是「丢弃」）
        with self._goal_lock:
            superseded = bool(self._queued.get(servo_id))
        if superseded:
            self.get_logger().warn(
                f'id {servo_id} 的这条 goal 已被更新的 goal 顶掉，不动作直接返回 preempted')
            goal_handle.abort()
            result.success = False
            result.pwm_readback = 0
            result.message = 'preempted: 已有更新的 goal 在排队，本条未执行'
            return result

        # 2) 执行前再校验一次：goal 排队期间参数可能被改小了
        pwm_min, pwm_max = self._limits()
        if self._locked or not self._connected:
            goal_handle.abort()
            result.success = False
            result.pwm_readback = 0
            result.message = 'watchdog_locked' if self._locked else 'serial_down'
            return result
        if not pwm_min <= target <= pwm_max:
            self.get_logger().warn(
                f'goal 排队期间限位被改成 {pwm_min}~{pwm_max}，pwm {target} 已超限，放弃执行')
            goal_handle.abort()
            result.success = False
            result.pwm_readback = 0
            result.message = f'out_of_range: pwm {target} 不在 {pwm_min}~{pwm_max}'
            return result

        self._moving.add(servo_id)
        self._state = 'moving'
        try:
            # 3) 先恢复扭力，再下发目标。上电后第一条动作不发 PULR 舵机纹丝不动
            if not self._torque_on(servo_id):
                goal_handle.abort()
                result.success = False
                result.pwm_readback = 0
                result.message = 'serial_down: PULR 写失败'
                return result

            start_pwm = self._read_position(servo_id, timeout=0.1)
            if start_pwm is None:
                start_pwm = target  # 读不到就按「已经在目标」算进度，不编造位置

            self.get_logger().info(
                f'id {servo_id} 开始动作：{start_pwm} -> {target}，用时 {duration}ms')
            if not self._send_move(servo_id, target, duration, echo=True):
                goal_handle.abort()
                result.success = False
                result.pwm_readback = 0
                result.message = 'serial_down: 写指令失败'
                return result

            # 4) 反馈循环：>= 5 Hz，且每一轮都检查取消 / 抢占 / 看门狗
            outcome = self._feedback_loop(goal_handle, servo_id, start_pwm,
                                          target, duration)
            if outcome == 'canceled':
                return self._finish_canceled(goal_handle, servo_id, result)
            if outcome == 'preempted':
                goal_handle.abort()
                result.success = False
                result.pwm_readback = self._read_position(servo_id) or 0
                result.message = 'preempted: 被更新的 goal 接管，已从当前位置交出控制权'
                self.get_logger().warn(f'id {servo_id} 被抢占，停止发反馈由新 goal 接手')
                return result
            if outcome == 'locked':
                goal_handle.abort()
                result.success = False
                result.pwm_readback = self._read_position(servo_id) or 0
                result.message = 'watchdog_locked: 动作中途看门狗跳闸'
                return result
            if outcome == 'serial_down':
                goal_handle.abort()
                result.success = False
                result.pwm_readback = 0
                result.message = 'serial_down: 动作途中串口断开'
                return result

            # 5) 到位校验：读 PRAD，超阈值就重发一次（PRAD 闭环加分项）
            return self._verify_and_finish(goal_handle, servo_id, target, result)
        finally:
            self._moving.discard(servo_id)
            if not self._moving and not self._locked:
                self._state = 'idle'

    def _feedback_loop(self, goal_handle, servo_id: int, start_pwm: int,
                       target: int, duration_ms: int) -> str:
        """吐 feedback，同时盯取消 / 抢占 / 看门狗。返回结束原因。"""
        period = max(0.02, int(self._param('feedback_period_ms')) / 1000.0)
        span = max(1e-3, duration_ms / 1000.0)
        begun = time.monotonic()
        deadline = begun + span
        last_feedback = 0.0
        event = self._preempt.setdefault(servo_id, threading.Event())

        while True:
            now = time.monotonic()
            ratio = min(1.0, (now - begun) / span)

            if goal_handle.is_cancel_requested:
                return 'canceled'
            if self._locked:
                return 'locked'
            if event.is_set():
                return 'preempted'
            if not self._connected:
                return 'serial_down'

            if now - last_feedback >= period or now >= deadline:
                position = self._read_position(servo_id, timeout=0.05)
                if position is None:
                    position = int(round(start_pwm + (target - start_pwm) * ratio))
                feedback = MoveServo.Feedback()
                feedback.pwm_current = int(position)
                feedback.progress = float(ratio)
                goal_handle.publish_feedback(feedback)
                last_feedback = now

            if now >= deadline:
                return 'done'
            time.sleep(0.01)

    def _verify_and_finish(self, goal_handle, servo_id: int, target: int,
                           result) -> MoveServo.Result:
        """读 PRAD 比对，差超阈值就重发一次，日志里留下「未到位 → 重发 → 到位」。"""
        tolerance = int(self._param('pos_tolerance'))
        retries = int(self._param('retry_times'))
        duration = int(self._param('default_duration_ms'))

        readback = self._read_position(servo_id)
        if readback is None:
            goal_handle.abort()
            result.success = False
            result.pwm_readback = 0
            result.message = 'prad_timeout: 读不到 PRAD，无法校验收否到位'
            self.get_logger().error(f'id {servo_id} 动作结束但读不到 PRAD')
            return result

        delta = readback - target
        if abs(delta) > tolerance:
            self.get_logger().warn(
                f'id {servo_id} 未到位：目标 {target} 读回 {readback} 差 {delta:+d} '
                f'（阈值 ±{tolerance}）')
            for attempt in range(1, retries + 1):
                self.get_logger().warn(f'第 {attempt} 次重发：{readback} -> {target}')
                if not self._send_move(servo_id, target, duration, echo=True):
                    break
                time.sleep(duration / 1000.0 + 0.08)
                readback = self._read_position(servo_id)
                if readback is None:
                    break
                delta = readback - target
                if abs(delta) <= tolerance:
                    self.get_logger().info(
                        f'重发后到位：目标 {target} 读回 {readback} 差 {delta:+d}')
                    break
                self.get_logger().warn(f'重发后仍未到位：读回 {readback} 差 {delta:+d}')

        result.pwm_readback = int(readback or 0)
        if abs(delta) <= tolerance:
            goal_handle.succeed()
            result.success = True
            result.message = 'ok'
            self.get_logger().info(
                f'id {servo_id} 动作完成：目标 {target} 读回 {readback} 差 {delta:+d}')
        else:
            goal_handle.abort()
            result.success = False
            result.message = (f'not_in_position: 目标 {target} 读回 {readback} '
                              f'差 {delta:+d} > {tolerance}')
            self.get_logger().error(result.message)
        return result

    def _finish_canceled(self, goal_handle, servo_id: int, result) -> MoveServo.Result:
        """取消：立刻停发、回中位、result 里写 canceled。"""
        center = self._center()
        duration = int(self._param('center_duration_ms'))
        self.get_logger().warn(
            f'id {servo_id} 取消：停止动作，回中位 {center}（用时 {duration}ms）')
        self._send_move(servo_id, center, duration, echo=True)
        goal_handle.canceled()
        result.success = False
        result.pwm_readback = self._read_position(servo_id) or 0
        result.message = 'canceled'
        return result

    # ================================================================ 话题
    def _on_command_topic(self, msg: ServoCommand) -> None:
        """内部调试总线 / bag 回放的入口。

        注意：这里只写串口，不再把消息发回 /servo/command —— 驱动自己也订阅这个话题，
        回发就是自己喂自己，一个字节就能无限循环。
        """
        if self._locked:
            self.get_logger().warn('/servo/command 被忽略：看门狗锁定中',
                                   throttle_duration_sec=2.0)
            return
        servo_id = int(msg.id)
        pwm = int(msg.pwm)
        if servo_id not in self._servo_ids():
            self.get_logger().warn(
                f'/servo/command 丢弃：id {servo_id} 不在 servo_ids {self._servo_ids()}',
                throttle_duration_sec=2.0)
            return
        pwm_min, pwm_max = self._limits()
        if not pwm_min <= pwm <= pwm_max:
            self.get_logger().warn(
                f'/servo/command 丢弃：pwm {pwm} 超出 {pwm_min}~{pwm_max}',
                throttle_duration_sec=2.0)
            return
        duration = int(msg.duration_ms) or self._default_duration()
        if self._write_frame(proto.build_move_command(servo_id, pwm, duration)):
            self.get_logger().info(
                f'[topic] id {servo_id} -> {pwm}，用时 {duration}ms（bag 回放通道）',
                throttle_duration_sec=1.0)

    def _on_heartbeat(self, _msg: Empty) -> None:
        """上位机心跳。第一次收到才开始计时，避免「没人跑客户端」被误判成卡死。"""
        if not self._param('watchdog_enabled'):
            return
        self._last_beat = time.monotonic()
        if not self._watchdog_armed:
            self._watchdog_armed = True
            self.get_logger().info(
                f'看门狗已武装：{int(self._param("watchdog_timeout_ms"))}ms 内没有心跳就全部卸力')

    # ================================================================ 定时器
    def _on_state_timer(self) -> None:
        self._beat += 1
        state = 'watchdog_locked' if self._locked else (
            'moving' if self._moving else 'idle')
        payload = {
            'stamp': round(self.get_clock().now().nanoseconds / 1e9, 3),
            'connected': bool(self._connected),
            'state': state,
            'locked': bool(self._locked),
            'moving': sorted(int(i) for i in self._moving),
            'beat': self._beat,
        }
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _on_watchdog_timer(self) -> None:
        """1 秒没心跳 -> 全部 PULK 并锁定，必须外部确认才恢复。"""
        if self._locked or not self._watchdog_armed:
            return
        if not self._param('watchdog_enabled'):
            return
        timeout = int(self._param('watchdog_timeout_ms')) / 1000.0
        if time.monotonic() - self._last_beat <= timeout:
            return

        self._locked = True
        self._state = 'watchdog_locked'
        self.get_logger().error(
            f'看门狗跳闸：{timeout:.1f}s 没收到 {TOPIC_HEARTBEAT}，'
            f'全部舵机 PULK 卸力并锁定，需调 {SRV_CONFIRM_RESUME} 才能恢复')
        with self._goal_lock:
            for event in self._preempt.values():
                event.set()  # 让正在跑的动作立刻退出
        self._pulk_all()

    def _on_confirm_resume(self, request, response):
        """人工确认后才允许恢复；恢复后重新武装，等下一拍心跳。"""
        if not self._locked:
            response.success = True
            response.message = '看门狗未跳闸，无需恢复'
            return response
        self._locked = False
        self._watchdog_armed = False
        self._last_beat = time.monotonic()
        self._state = 'idle'
        self.get_logger().warn(
            '看门狗已人工确认恢复：舵机仍处于卸力状态，下一条 goal 会自动重发 PULR')
        response.success = True
        response.message = 'watchdog cleared；下一条 goal 会自动 PULR'
        return response

    def _pulk_all(self) -> None:
        for servo_id in self._servo_ids():
            self._write_frame(proto.build_torque_off(servo_id))

    def _on_reconnect_timer(self) -> None:
        if self._reload_link:
            self._reload_link = False
            self._link.close()
            self._link = SerialLink(self._param('port'), self._param('baudrate'))
            self._connected = False
            self.get_logger().info(
                f'port/baudrate 变化，重建链路：{self._link.port}@{self._link.baudrate}')

        if self._link.is_open:
            return
        self._open_link(restore_torque=True)

    def _open_link(self, restore_torque: bool) -> bool:
        """开串口。失败只打 WARN，由重连定时器继续试 —— 节点不许因此退出。"""
        if self._link.is_open:
            self._connected = True
            return True
        try:
            self._link.open()
        except SerialLinkError as exc:
            self.get_logger().warn(
                f'{exc}；{int(self._param("reconnect_period_ms"))}ms 后自动重试',
                throttle_duration_sec=5.0)
            return False
        self._connected = True
        self.get_logger().info(
            f'串口已连接：{self._link.port}@{self._link.baudrate}（8-N-1）')
        if restore_torque:
            # 掉线期间可能被看门狗卸过力，补一次 PULR，否则下一条指令「发了不动」
            for servo_id in self._servo_ids():
                self._torque_on(servo_id)
        return True

    def _on_startup_timer(self) -> None:
        """一次性：上电全部回中位（startup_reset 可关）。"""
        self._startup_timer.cancel()
        if not self._param('startup_reset'):
            self.get_logger().info('startup_reset=false，跳过上电回中位')
            return
        if not self._connected:
            self.get_logger().warn('startup_reset 跳过：串口还没连上')
            return
        center = self._center()
        duration = int(self._param('center_duration_ms'))
        self.get_logger().info(
            f'startup_reset：{self._servo_ids()} 全部回中位 {center}，用时 {duration}ms')
        for servo_id in self._servo_ids():
            self._torque_on(servo_id)
            self._send_move(servo_id, center, duration, echo=True)
            time.sleep(0.05)

    # ================================================================ 收尾
    def destroy_node(self):
        self._shutting_down = True
        try:
            self._link.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoDriver()
    # 必须是多线程执行器：单线程时 execute_callback 里的 sleep 会把整个 executor 占死，
    # Cancel 请求根本进不来（这是「Ctrl+C 取消没反应」的唯一根因）。
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('servo_driver 退出')
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
