#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""servo_driver.py —— 总线舵机驱动节点（Action Server）。

节点只做四件事，逻辑都在 core.py / arbiter.py 里：
  1. 把 ROS 参数搬进 CoreConfig，并校验参数（非法值直接拒绝）；
  2. 四个 Action 回调：goal / handle_accepted / cancel / execute；
  3. 订阅 /servo/command（bag 回放通道），和 Action 走同一个 core.move；
  4. 0.5s 心跳 + /servo/recover 服务（看门狗安全模式的外部确认口）。
"""

import itertools
import json
import time
from collections import OrderedDict

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cquick_servo.arbiter import GoalArbiter
from cquick_servo.core import SUPPORTED_BAUDRATES, CoreConfig, ServoCore
from cquick_servo.protocol import PWM_MAX_ELECTRIC, PWM_MIN_ELECTRIC
from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

STATE_PERIOD_S = 0.5        # /servo/state 心跳周期（看门狗 1s 超时的一半）
PROGRESS_PERIOD_S = 0.1     # feedback 周期：10 Hz，满足 ≥5 Hz
CANCEL_RETURN_MS = 800      # 取消后回中位用多久：300ms 够快又不至于猛跳
MIN_DURATION_MS = 50
MAX_DURATION_MS = 20000
ECHO_WINDOW_S = 2.0     # 自己发的命令在这么久内收到的回环视为「回声」，不重复执行


class ServoDriver(Node):

    def __init__(self, core=None):
        super().__init__('servo_driver')

        # ---------- ① 参数声明：默认值只允许出现在这里 ----------
        self.declare_parameter('port', '/dev/cq_servo')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pwm_min', 900)
        self.declare_parameter('pwm_max', 2100)
        self.declare_parameter('default_duration_ms', 800)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('startup_reset', True)
        self.declare_parameter('position_tolerance', 30)
        self.declare_parameter('watchdog_timeout_s', 1.0)
        # 读不到 PRAD 时：false=按命令值记成功并在 message 标注 open-loop；
        # true=严格模式，读不到就判失败（控制板支持回读时建议打开）
        self.declare_parameter('require_prad', False)

        # ---------- ② 回调组：允许 execute 和其他回调真正并发 ----------
        self._cb_group = ReentrantCallbackGroup()
        self._arbiter = GoalArbiter()

        # ---------- ③ 核心（可注入，方便离线自测） ----------
        self._core = core or ServoCore(self._config_from_parameters(),
                                       logger=self.get_logger())
        self._core.on_state = self._on_core_state

        # ---------- ④ 参数校验回调（没有它，什么值都能写进来） ----------
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ---------- ⑤ 话题 / 心跳 / 恢复服务 ----------
        self._state_pub = self.create_publisher(String, '/servo/state', 10)
        # /servo/command 既是「bag 录制用的命令日志」，也是「回放入口」：
        #   Action 执行时把同一条指令 publish 出去（bag 才有东西可录）；
        #   回放时本节点订阅到这些消息，走同一个 core.move。
        self._command_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
        self._cmd_seq = itertools.count(1)
        self._pending_seqs = OrderedDict()      # 自己刚发的 seq -> 发出时刻，用于去回环
        self.create_timer(STATE_PERIOD_S, self._on_heartbeat_timer,
                          callback_group=self._cb_group)
        self.create_subscription(ServoCommand, '/servo/command', self._on_command, 10,
                                 callback_group=self._cb_group)
        self.create_service(Trigger, '/servo/recover', self._on_recover,
                            callback_group=self._cb_group)

        # ---------- ⑥ Action Server：四个回调全部接上 ----------
        self._action_server = ActionServer(
            self,
            MoveServo,
            'move_servo',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        # ---------- ⑦ 启动动作 + 看门狗 ----------
        if bool(self.get_parameter('startup_reset').value):
            self._startup_reset()

        self._core.heartbeat()
        self._core.start_watchdog()

        cfg = self._core.config
        self.get_logger().info(
            f'servo_driver 就绪：port={cfg.port} baudrate={cfg.baudrate} '
            f'ids={list(cfg.servo_ids)} 安全区间=[{cfg.pwm_min}, {cfg.pwm_max}] '
            f'看门狗={cfg.watchdog_timeout_s}s')

        if not bool(self.get_parameter('require_prad').value):
            self.get_logger().warning(
                'require_prad=false：读不到 PRAD 回显时按命令值记成功（message 标注 open-loop），'
                'PRAD 闭环加分项不成立，README 里要如实说明')

    # ==================== 参数 ====================

    def _config_from_parameters(self) -> CoreConfig:
        return CoreConfig(
            port=str(self.get_parameter('port').value),
            baudrate=int(self.get_parameter('baudrate').value),
            pwm_min=int(self.get_parameter('pwm_min').value),
            pwm_max=int(self.get_parameter('pwm_max').value),
            default_duration_ms=int(self.get_parameter('default_duration_ms').value),
            servo_ids=tuple(int(x) for x in self.get_parameter('servo_ids').value),
            position_tolerance=int(self.get_parameter('position_tolerance').value),
            watchdog_timeout_s=float(self.get_parameter('watchdog_timeout_s').value),
        )

    @staticmethod
    def _coerce(name: str, value):
        if name == 'port':
            return str(value)
        if name == 'servo_ids':
            return tuple(int(x) for x in value)
        if name == 'watchdog_timeout_s':
            return float(value)
        return int(value)

    def _on_set_parameters(self, params):
        incoming = {p.name: p.value for p in params}

        try:
            errors = self._validate_parameters(incoming)
        except (TypeError, ValueError) as exc:
            reason = f'参数类型错误：{exc}'
            self.get_logger().warning(f'参数被拒绝：{reason}')
            return SetParametersResult(successful=False, reason=reason)

        if errors:
            reason = '；'.join(errors)
            self.get_logger().warning(f'参数被拒绝：{reason}')
            return SetParametersResult(successful=False, reason=reason)

        # 校验通过 -> 同步给 core，下一帧判断立即生效（不需要重启节点）
        updates = {name: self._coerce(name, value) for name, value in incoming.items()
                   if name != 'require_prad'}      # 纯策略开关，不进 CoreConfig
        if updates:
            self._core.update_config(**updates)
        self.get_logger().info(f'参数已更新并生效：{sorted(incoming)}')
        return SetParametersResult(successful=True)

    def _validate_parameters(self, incoming):
        """返回错误列表；空列表表示全部合法。"""
        def effective(name):
            return incoming[name] if name in incoming else self.get_parameter(name).value

        errors = []

        pwm_min = int(effective('pwm_min'))
        pwm_max = int(effective('pwm_max'))
        if not PWM_MIN_ELECTRIC <= pwm_min <= PWM_MAX_ELECTRIC:
            errors.append(f'pwm_min={pwm_min} 超出电气范围 {PWM_MIN_ELECTRIC}~{PWM_MAX_ELECTRIC}')
        if not PWM_MIN_ELECTRIC <= pwm_max <= PWM_MAX_ELECTRIC:
            errors.append(f'pwm_max={pwm_max} 超出电气范围 {PWM_MIN_ELECTRIC}~{PWM_MAX_ELECTRIC}')
        if pwm_min >= pwm_max:
            errors.append(f'pwm_min={pwm_min} 必须小于 pwm_max={pwm_max}')

        baudrate = int(effective('baudrate'))
        if baudrate not in SUPPORTED_BAUDRATES:
            errors.append(f'baudrate={baudrate} 不在支持列表 {list(SUPPORTED_BAUDRATES)}')

        port = str(effective('port'))
        if not port.startswith('/dev/'):
            errors.append(f'port={port} 必须是 /dev/ 下的设备路径')

        ids = [int(x) for x in effective('servo_ids')]
        if not ids:
            errors.append('servo_ids 不能为空')
        bad_ids = [x for x in ids if not 0 <= x <= 254]
        if bad_ids:
            errors.append(f'servo_ids 含非法 ID {bad_ids}（合法 0~254）')
        if len(set(ids)) != len(ids):
            errors.append(f'servo_ids 有重复：{ids}')

        duration = int(effective('default_duration_ms'))
        if not MIN_DURATION_MS <= duration <= MAX_DURATION_MS:
            errors.append(f'default_duration_ms={duration} 超出 {MIN_DURATION_MS}~{MAX_DURATION_MS}')

        tolerance = int(effective('position_tolerance'))
        if not 1 <= tolerance <= 500:
            errors.append(f'position_tolerance={tolerance} 超出 1~500')

        watchdog = float(effective('watchdog_timeout_s'))
        if not 0.0 <= watchdog <= 10.0:
            errors.append(f'watchdog_timeout_s={watchdog} 超出 0~10（0 表示关闭）')

        return errors

    # ==================== Action 回调 ====================

    def goal_callback(self, goal_request):
        """① 发指令之前就拒掉非法 goal。

        注意：rclpy 传给 goal_callback 的是 MoveServo.Goal 本身（没有 .request），
        只有 handle_accepted / execute / cancel 拿到的才是 goal handle。
        """
        sid = int(goal_request.id)
        pwm = int(goal_request.pwm)
        duration = int(goal_request.duration_ms)
        cfg = self._core.config

        if self._core.fault:
            self.get_logger().warning(
                f'[goal REJECT] 安全模式中（{self._core.fault_reason}），'
                f'先调 /servo/recover')
            return GoalResponse.REJECT
        if sid not in cfg.servo_ids:
            self.get_logger().warning(f'[goal REJECT] id={sid} 不在 servo_ids={list(cfg.servo_ids)}')
            return GoalResponse.REJECT
        if not cfg.pwm_min <= pwm <= cfg.pwm_max:
            self.get_logger().warning(
                f'[goal REJECT] pwm={pwm} 超出安全区间 [{cfg.pwm_min}, {cfg.pwm_max}]')
            return GoalResponse.REJECT
        if duration != 0 and not MIN_DURATION_MS <= duration <= MAX_DURATION_MS:
            self.get_logger().warning(
                f'[goal REJECT] duration_ms={duration} 非法'
                f'（{MIN_DURATION_MS}~{MAX_DURATION_MS}，0 表示用默认值）')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'[goal ACCEPT] id={sid} pwm={pwm} duration={duration or cfg.default_duration_ms}')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """② goal 已接受：交给 execute_callback。"""
        self.get_logger().info(f'[goal EXEC] id={int(goal_handle.request.id)} 开始执行')
        execute = getattr(goal_handle, 'execute', None)
        if callable(execute):
            execute()
        # 没有 execute() 的老版本 rclpy 会由 ActionServer 自动调用 execute_callback

    def cancel_callback(self, goal_handle):
        """③ 接受取消请求；真正的停发在 execute 循环里。"""
        self.get_logger().warning(f'[CANCEL] id={int(goal_handle.request.id)} 收到取消请求 -> ACCEPT')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """④ 发指令 -> 循环 feedback -> 读 PRAD -> 返回 result。"""
        request = goal_handle.request
        sid = int(request.id)
        target = int(request.pwm)
        cfg = self._core.config
        duration = int(request.duration_ms) or cfg.default_duration_ms
        tolerance = cfg.position_tolerance

        token = self._arbiter.begin(sid)
        self.get_logger().info(
            f'[goal] id={sid} pwm={target} duration={duration}ms token={token}')

        # 还没开始动就被取消了：直接回中位（别先走到目标再折回来）
        if goal_handle.is_cancel_requested:
            center = self._core.center_pwm()
            self._core.move(sid, center, CANCEL_RETURN_MS)
            goal_handle.canceled()
            self._arbiter.forget(sid, token)
            self.get_logger().warning(f'[goal CANCELED] id={sid} 执行前被取消，已回中位 {center}')
            return self._make_result(False, center, 'canceled')

        # 发到调试总线：bag 记录的就是这条；先登记 seq 再 publish，
        # 这样即使回环消息先到，也能被认出来并丢掉。
        self._publish_command(sid, target, duration)

        if not self._core.move(sid, target, duration):
            reason = 'safe mode: ' + self._core.fault_reason if self._core.fault else 'serial unavailable'
            self.get_logger().error(f'[goal ABORT] id={sid}：{reason}')
            goal_handle.abort()
            self._arbiter.forget(sid, token)
            return self._make_result(False, 0, reason)

        started = time.monotonic()
        duration_s = max(duration / 1000.0, 0.05)

        try:
            while True:
                if not rclpy.ok():
                    goal_handle.abort()
                    return self._make_result(False, 0, 'shutdown')

                if self._core.fault:
                    goal_handle.abort()
                    return self._make_result(False, 0, f'watchdog: {self._core.fault_reason}')

                if not self._arbiter.is_current(sid, token):
                    current = self._core.estimated_pwm(sid)
                    self.get_logger().warning(
                        f'[goal PREEMPTED] id={sid} 被更新的 goal 取代，不回中位（当前位置 {current}）')
                    goal_handle.abort()
                    return self._make_result(False, current, 'preempted')

                if goal_handle.is_cancel_requested:
                    center = self._core.center_pwm()
                    self._core.move(sid, center, CANCEL_RETURN_MS)
                    self.get_logger().warning(f'[goal CANCELED] id={sid} 已回中位 {center}')
                    goal_handle.canceled()
                    return self._make_result(False, center, 'canceled')

                progress = min(1.0, (time.monotonic() - started) / duration_s)
                feedback = MoveServo.Feedback()
                feedback.current_pwm = self._core.estimated_pwm(sid)
                feedback.progress = float(progress)
                try:
                    goal_handle.publish_feedback(feedback)
                except Exception as exc:      # feedback 失败不该影响动作
                    self.get_logger().warning(f'feedback 发布失败：{exc}')

                if progress >= 1.0:
                    break
                time.sleep(PROGRESS_PERIOD_S)
        finally:
            self._arbiter.forget(sid, token)

        # ---- PRAD 闭环校验：读到了就比对；读不到按 require_prad 决定是否降级 ----
        verified = self._core.verify(sid, target, duration, tolerance)
        if verified.final_pwm is None:
            if bool(self.get_parameter('require_prad').value):
                self.get_logger().error(f'[goal ABORT] id={sid} PRAD 读不到回显（严格模式）')
                goal_handle.abort()
                return self._make_result(False, 0, 'PRAD 读不到回显：检查串口 / ID / 供电')
            goal_handle.succeed()
            self.get_logger().warning(
                f'[goal OK-OPEN] id={sid} 无 PRAD 回显，用最后下发的 {target} 作为结果（open-loop）')
            return self._make_result(True, target, 'ok diff=0 (open-loop: no PRAD echo)')

        if verified.ok:
            suffix = ' (retried)' if verified.retried else ''
            goal_handle.succeed()
            self.get_logger().info(
                f'[goal OK] id={sid} pwm={target} final={verified.final_pwm} '
                f'diff={verified.diff}{suffix}')
            return self._make_result(True, verified.final_pwm,
                                     f'ok diff={verified.diff}{suffix}')

        self.get_logger().error(
            f'[goal ABORT] id={sid} 未到位：目标={target} 实际={verified.final_pwm} '
            f'偏差={verified.diff} 阈值=±{tolerance}')
        goal_handle.abort()
        return self._make_result(
            False, verified.final_pwm,
            f'not in position: target={target} actual={verified.final_pwm} '
            f'diff={verified.diff} tol={tolerance}')

    @staticmethod
    def _make_result(success: bool, final_pwm: int, message: str):
        result = MoveServo.Result()
        result.success = bool(success)
        result.final_pwm = max(0, min(65535, int(final_pwm)))
        result.message = str(message)
        return result

    # ==================== 话题 / 服务 ====================

    def _on_heartbeat_timer(self):
        self._core.heartbeat()
        self._publish_state({
            'event': 'heartbeat',
            'alive': True,
            'fault': self._core.fault,
            'ids': list(self._core.config.servo_ids),
            'ts': round(time.time(), 3),
        })

    def _on_core_state(self, payload):
        payload['ts'] = round(time.time(), 3)
        self._publish_state(payload)

    def _publish_state(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._state_pub.publish(msg)

    def _publish_command(self, servo_id: int, pwm: int, duration_ms: int) -> None:
        """把这条指令发到 /servo/command，bag 才有东西可录。"""
        seq = next(self._cmd_seq)
        self._pending_seqs[seq] = time.monotonic()
        while len(self._pending_seqs) > 256:          # 防止只发不收时无限增长
            self._pending_seqs.popitem(last=False)

        command = ServoCommand()
        command.id = int(servo_id)
        command.pwm = int(pwm)
        command.duration_ms = int(duration_ms)
        command.seq = seq
        self._command_pub.publish(command)

    def _on_command(self, msg: ServoCommand):
        """bag 回放通道：和 Action 走同一个 core.move，所以回放动作和录屏一致。"""
        # 自己发出去的命令会被自己的订阅收到（ROS 2 回环）。
        # 用 seq 认出来并丢掉，否则同一条命令会被执行两次。
        seq = int(getattr(msg, 'seq', 0) or 0)
        if seq:
            published_at = self._pending_seqs.pop(seq, None)
            if published_at is not None and time.monotonic() - published_at <= ECHO_WINDOW_S:
                self.get_logger().debug(f'/servo/command seq={seq} 是自己的回环，忽略')
                return

        sid = int(msg.id)
        pwm = int(msg.pwm)
        duration = int(msg.duration_ms)
        cfg = self._core.config

        if self._core.fault:
            self.get_logger().warning('/servo/command 忽略：安全模式中')
            return
        if sid not in cfg.servo_ids:
            self.get_logger().warning(f'/servo/command 忽略：id={sid} 不在 servo_ids')
            return
        if not cfg.pwm_min <= pwm <= cfg.pwm_max:
            self.get_logger().warning(
                f'/servo/command 忽略：pwm={pwm} 超出 [{cfg.pwm_min}, {cfg.pwm_max}]')
            return

        self.get_logger().info(
            f'[topic] /servo/command id={sid} pwm={pwm} duration={duration or cfg.default_duration_ms}')
        if not self._core.move(sid, pwm, duration or cfg.default_duration_ms):
            self.get_logger().warning('[topic] 下发失败（串口不可用？）')

    def _on_recover(self, request, response):
        success, message = self._core.recover()
        response.success = bool(success)
        response.message = str(message)
        return response

    # ==================== 启动 / 退出 ====================

    def _startup_reset(self):
        cfg = self._core.config
        center = self._core.center_pwm()
        self.get_logger().info(
            f'启动回中位：ids={list(cfg.servo_ids)} pwm={center} duration={cfg.default_duration_ms}ms')
        for sid in cfg.servo_ids:
            self._core.move(int(sid), center, cfg.default_duration_ms)

    def destroy_node(self):
        try:
            self._core.stop_watchdog()
            self._core.pulk_all()       # 退出前一定卸力
            self._core.close()
        except Exception as exc:        # 退出路径不许再抛
            self.get_logger().warning(f'退出清理异常：{exc}')
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    # 8 个线程：一个 step 里 6 个关节并行 + 心跳 + 服务/取消回调，4 个不够
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
