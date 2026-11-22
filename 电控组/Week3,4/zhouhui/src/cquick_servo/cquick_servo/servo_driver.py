"""舵机驱动节点：Action 服务端 + 串口 ASCII 指令。

对外只有三个口子：
  Action  /move_servo      —— 唯一控制口（goal / feedback / result / cancel）
  话题    /servo/command   —— 给 bag 回放用的内部调试总线，和 Action 走同一个发送函数
  话题    /servo/state     —— 心跳，给看门狗和 bag 用

参数全部外置：端口、波特率、安全限位、默认时长、舵机清单、上电姿态、限速、
看门狗阈值、重发次数。用之前一律 self.get_parameter(...).value 现场读，
不在 __init__ 里抄副本 —— 抄了副本 ros2 param set 就不会当帧生效。
"""
import threading
import time
import traceback

import serial

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand, ServoState

from .servo_ascii import (
    enable_cmd,
    move_cmd,
    parse_position,
    plan_duration,
    read_pos_cmd,
    release_cmd,
)

# ---- 协议/物理常量（不是配置，允许写在代码里）----
MID_PWM = 1500            # 中立位 = 1.5 ms 脉宽 = 1500
ON_TARGET = 30            # PRAD 读回与目标差多少算「到位」
FB_PERIOD_S = 0.2         # 5 Hz，考核要求 feedback 不低于 5 Hz
POS_READ_TIMEOUT_S = 0.08 # 单次 PRAD 读回上限，太长会把 feedback 节拍拖慢
STARTUP_RESET_MS = 4000   # 上电回中位给足时间，带载实测大约 30~50 PWM/s
CANCEL_MS = 300           # 取消时回中位要「快」
CANCEL_SETTLE_S = 0.35    # 回中位后等停稳再卸力
RETRY_EXTRA_MS = 1200     # 未到位重发时至少给这么久，否则重发也追不上

# ---- result.code 约定 ----
CODE_OK = 0
CODE_NOT_IN_PLACE = 1
CODE_CANCELED = 2
CODE_PREEMPTED = 3
CODE_SERIAL = 4
CODE_LOCKED = 5


class ServoDriver(Node):
    def __init__(self):
        super().__init__("servo_driver")

        self._declare_params()
        self.add_on_set_parameters_callback(self._on_param_set)

        self.group = ReentrantCallbackGroup()
        self._io_lock = threading.RLock()   # 一条半双工总线只能有一个线程在用
        self._ser = None
        self._link_ok = False
        self._opened_before = False
        self._reopen_requested = False

        self._locked = False                # 安全锁：看门狗触发后置位
        self._goals = {}                    # sid -> 正在跑的 goal_handle
        self._preempted = set()             # 被新 goal 顶掉的 goal_handle（按对象 id 记，避免串号）
        self._active = 0
        self._last_tick = time.monotonic()
        self._last_link_warn = 0.0

        self._open_port()

        self._state_pub = self.create_publisher(ServoState, "/servo/state", 10)
        self.create_timer(0.2, self._publish_state)
        self.create_timer(0.2, self._watchdog_tick)
        self.create_timer(2.0, self._link_tick)

        self.create_subscription(
            ServoCommand, "/servo/command", self._on_command_topic, 10,
            callback_group=self.group)

        self.create_service(Trigger, "~/torque_off", self._srv_torque_off)
        self.create_service(Trigger, "~/watchdog_reset", self._srv_watchdog_reset)
        self.create_service(Trigger, "~/watchdog_trip", self._srv_watchdog_trip)

        self._server = ActionServer(
            self, MoveServo, "move_servo",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self.group,
        )

        self._startup()

    # ------------------------------------------------------------------ 参数
    def _declare_params(self):
        d = ParameterDescriptor
        t = ParameterType
        self.declare_parameter("port", "/dev/cq_servo", d(type=t.PARAMETER_STRING))
        self.declare_parameter("baudrate", 115200, d(type=t.PARAMETER_INTEGER))
        self.declare_parameter("pwm_min", 900, d(type=t.PARAMETER_INTEGER))
        self.declare_parameter("pwm_max", 2100, d(type=t.PARAMETER_INTEGER))
        self.declare_parameter("default_duration_ms", 800, d(type=t.PARAMETER_INTEGER))
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 6],
                               d(type=t.PARAMETER_INTEGER_ARRAY))
        self.declare_parameter("startup_reset", True, d(type=t.PARAMETER_BOOL))
        self.declare_parameter("speed_pwm_per_s", 600, d(type=t.PARAMETER_INTEGER))
        self.declare_parameter("watchdog_timeout_s", 1.0, d(type=t.PARAMETER_DOUBLE))
        self.declare_parameter("prad_retry", 1, d(type=t.PARAMETER_INTEGER))

    def _p(self, name):
        """每次都现场读参数，这是「当帧生效」的关键。"""
        return self.get_parameter(name).value

    def _on_param_set(self, params):
        """ros2 param set 的守门人：返回 False 才真的拒掉，参数保持原样。"""
        pmin, pmax = int(self._p("pwm_min")), int(self._p("pwm_max"))
        baud = int(self._p("baudrate"))
        dur = int(self._p("default_duration_ms"))
        speed = int(self._p("speed_pwm_per_s"))
        wd = float(self._p("watchdog_timeout_s"))
        ids = list(self._p("servo_ids"))

        for item in params:
            if item.name == "pwm_min":
                pmin = int(item.value)
            elif item.name == "pwm_max":
                pmax = int(item.value)
            elif item.name == "baudrate":
                baud = int(item.value)
            elif item.name == "default_duration_ms":
                dur = int(item.value)
                if dur <= 0:
                    return SetParametersResult(
                        successful=False, reason="default_duration_ms 必须 > 0")
            elif item.name == "speed_pwm_per_s":
                speed = int(item.value)
                if speed < 1:
                    return SetParametersResult(
                        successful=False, reason="speed_pwm_per_s 必须 >= 1")
            elif item.name == "watchdog_timeout_s":
                wd = float(item.value)
                if wd < 0.1:
                    return SetParametersResult(
                        successful=False, reason="watchdog_timeout_s 不能小于 0.1 s")
            elif item.name == "servo_ids":
                ids = [int(x) for x in item.value]
                if not ids:
                    return SetParametersResult(
                        successful=False, reason="servo_ids 不能为空")
                if any(x < 1 or x > 254 for x in ids):
                    return SetParametersResult(
                        successful=False, reason="servo_ids 每个元素必须在 1~254")
                if len(set(ids)) != len(ids):
                    return SetParametersResult(
                        successful=False, reason="servo_ids 不能有重复")
            elif item.name == "port":
                self._reopen_requested = True

        if baud not in (9600, 115200):
            return SetParametersResult(
                successful=False, reason=f"不认识的波特率 {baud}，只接受 9600 / 115200")
        if not (500 <= pmin <= 2500) or not (500 <= pmax <= 2500):
            return SetParametersResult(
                successful=False, reason="pwm 限位必须落在电气范围 500~2500")
        if pmin >= pmax:
            return SetParametersResult(
                successful=False, reason=f"pwm_min({pmin}) 不能 >= pwm_max({pmax})")
        if self._reopen_requested or any(
                item.name == "baudrate" for item in params):
            self._reopen_requested = True
            self.get_logger().warn("port/baudrate 已改，2 秒内按新值重开串口")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------- 串口通路
    def _open_port(self):
        port = self._p("port")
        baud = int(self._p("baudrate"))
        with self._io_lock:
            self._close_port_silently()
            try:
                # timeout 必须小：read(1) 等太久会把 PRAD 轮询和 feedback 节拍一起拖垮
                self._ser = serial.Serial(port, baud, timeout=0.02)
                self._link_ok = True
                if self._opened_before:
                    self.get_logger().warn(f"串口恢复可用：{port} @ {baud}")
                else:
                    self.get_logger().info(f"串口已打开：{port} @ {baud}")
                self._opened_before = True
                return True
            except Exception as exc:
                self._ser = None
                self._link_ok = False
                now = time.monotonic()
                if now - self._last_link_warn > 5.0:
                    self.get_logger().warn(f"串口打不开（{port} @ {baud}）：{exc}，2 s 后重试")
                    self._last_link_warn = now
                return False

    def _close_port_silently(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._link_ok = False

    def _drop_link(self, why):
        """掉线只关句柄、不退出节点 —— 节点崩了才是真的整机失控。"""
        with self._io_lock:
            was_ok = self._link_ok
            self._close_port_silently()
        if was_ok:
            self.get_logger().warn(f"串口掉线（{why}），节点继续运行，等自动重连")

    def _link_tick(self):
        with self._io_lock:
            need_open = self._reopen_requested
            self._reopen_requested = False
            alive = self._ser is not None and getattr(self._ser, "is_open", False)
        if need_open or not alive:
            self._open_port()

    def _send(self, line, force=False):
        """所有指令的唯一出口：Action、话题、服务都走这里。"""
        with self._io_lock:
            if self._locked and not force:
                self.get_logger().warn(f"安全锁生效，指令被丢弃：{line}",
                                       throttle_duration_sec=1.0)
                return False
            if self._ser is None or not getattr(self._ser, "is_open", False):
                self._open_port()
            if self._ser is None or not getattr(self._ser, "is_open", False):
                return False
            try:
                self._ser.reset_input_buffer()   # 清掉上一条留下的回显，免得串位
                self._ser.write(line.encode("ascii"))
                self._ser.flush()                # 不 flush 指令会压在缓冲里
            except Exception as exc:
                # 拔 USB 时 pyserial 可能抛 termios.error，它既不是 OSError 也不是
                # SerialException，这里用宽 except，宁可丢一条也不能让 goal 线程炸掉
                self._drop_link(f"{line} 写入异常 {exc!r}")
                return False
        self.get_logger().info(f"TX {line}")
        return True

    def _read_position(self, sid, timeout=POS_READ_TIMEOUT_S):
        with self._io_lock:
            if not self._send(read_pos_cmd(sid)):
                return None
            buf = b""
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                ser = self._ser
                if ser is None:
                    return None
                try:
                    bite = ser.read(1)
                except Exception as exc:
                    self._drop_link(f"PRAD 读取异常 {exc!r}")
                    return None
                if not bite:
                    continue
                buf += bite
                if bite == b"!":
                    break
            return parse_position(buf)

    def _read_position_retry(self, sid, tries=3, gap=0.05):
        """总线忙时单次 PRAD 可能读空，判成败前多试两次再下结论。"""
        for _ in range(max(int(tries), 1)):
            value = self._read_position(sid)
            if value is not None:
                return value
            time.sleep(gap)
        return None

    # --------------------------------------------------------------- 上电自检
    def _startup(self):
        try:
            ids = list(self._p("servo_ids"))
            for sid in ids:
                # PULR 只恢复输出、不产生位移，先发它是任务 1 踩过的坑
                self._send(enable_cmd(sid))
            if self._p("startup_reset"):
                self.get_logger().info(
                    f"上电回中位：{len(ids)} 个舵机 → {MID_PWM}，{STARTUP_RESET_MS} ms 走完")
                time.sleep(0.1)
                for sid in ids:
                    self._send(move_cmd(sid, MID_PWM, STARTUP_RESET_MS))
            else:
                self.get_logger().info("startup_reset=false，只上扭力，由动作 yaml 决定姿态")
        except Exception:
            traceback.print_exc()
            self.get_logger().error("上电自检出错，节点继续运行，串口巡检会重连")
        self.get_logger().info("servo_driver 就绪，等 /move_servo goal")

    # ---------------------------------------------------------------- 心跳/看门狗
    def _publish_state(self):
        msg = ServoState()
        msg.alive = bool(self._link_ok)
        msg.locked = bool(self._locked)
        msg.online_count = len(self._p("servo_ids")) if self._link_ok else 0
        msg.detail = f"link={'ok' if self._link_ok else 'down'} goals={self._active}"
        self._state_pub.publish(msg)

    def _watchdog_tick(self):
        """看门狗：动作在跑却 1 秒没有进度心跳 → 全部 PULK + 上锁，等人来解。

        监视的是「execute 线程还活着吗」。上位机卡死、executor 被堵、动作挂半路，
        表现都是这条超时 —— 舵机却还在满力矩顶结构，这 3 分就是救这个场景。
        """
        if self._locked:
            return
        if self._active <= 0:
            self._last_tick = time.monotonic()
            return
        limit = float(self._p("watchdog_timeout_s"))
        if time.monotonic() - self._last_tick > limit:
            self._trip(f"动作停摆超过 {limit}s")

    def _trip(self, why):
        self.get_logger().error(f"看门狗触发：{why} → 全部舵机 PULK 并上锁，需人工复位")
        for sid in list(self._p("servo_ids")):
            self._send(release_cmd(sid), force=True)
        self._locked = True
        self.get_logger().error("安全锁已置位：拒绝一切 goal / 话题指令")

    def _srv_torque_off(self, request, response):
        """下电第一步：ros2 service call /servo_driver/torque_off std_srvs/srv/Trigger"""
        self.get_logger().info("收到卸力请求：全部舵机 PULK")
        for sid in list(self._p("servo_ids")):
            self._send(release_cmd(sid), force=True)
        response.success = True
        response.message = "all servos torque off"
        return response

    def _srv_watchdog_reset(self, request, response):
        """只允许人来解 —— 节点不会自己恢复。"""
        if not self._locked:
            response.success = False
            response.message = "not locked"
            return response
        self._locked = False
        self._last_tick = time.monotonic()
        self.get_logger().warn("外部确认收到：安全锁解除，可以重新下发指令")
        response.success = True
        response.message = "unlocked"
        return response

    def _srv_watchdog_trip(self, request, response):
        """演示用：手动触发一次「卸力 + 上锁」。"""
        self._trip("手动触发")
        response.success = True
        response.message = "watchdog tripped"
        return response

    # ------------------------------------------------------- 内部调试话题（bag）
    def _on_command_topic(self, cmd):
        if self._locked:
            self.get_logger().warn("安全锁生效，话题指令被拒")
            return
        sid, pwm = int(cmd.id), int(cmd.pwm)
        dur = int(cmd.duration_ms)
        if dur <= 0:
            dur = int(self._p("default_duration_ms"))
        if sid not in list(self._p("servo_ids")):
            self.get_logger().warn(f"话题指令 id={sid} 不在 servo_ids，丢弃")
            return
        pmin, pmax = int(self._p("pwm_min")), int(self._p("pwm_max"))
        if not pmin <= pwm <= pmax:
            self.get_logger().warn(f"话题指令 pwm={pwm} 超限 {pmin}~{pmax}，丢弃")
            return
        # 与 Action 同一个发送函数，回放出来的动作才和录屏对得上
        self._send(enable_cmd(sid))
        time.sleep(0.05)
        self._send(move_cmd(sid, pwm, dur))

    # ------------------------------------------------------------------ Action
    def _on_goal(self, goal_request):
        if self._locked:
            self.get_logger().error("Goal 被拒：安全锁生效（看门狗）")
            return GoalResponse.REJECT

        sid, pwm = int(goal_request.id), int(goal_request.pwm)
        ids = list(self._p("servo_ids"))
        pmin, pmax = int(self._p("pwm_min")), int(self._p("pwm_max"))

        if sid not in ids:
            self.get_logger().error(f"Goal 被拒：id={sid} 不在 servo_ids {ids}")
            return GoalResponse.REJECT
        if not pmin <= pwm <= pmax:
            self.get_logger().error(f"Goal 被拒：pwm={pwm} 超出安全区间 {pmin}~{pmax}")
            return GoalResponse.REJECT

        # 抢占：同一舵机已有 goal 在跑 → 让旧的交棒，新的从当前位置平滑接上
        old = self._goals.get(sid)
        if old is not None and getattr(old, "is_active", False):
            self.get_logger().warn(f"id={sid} 新 goal 抢占旧 goal（不跳变、不回中位）")
            self._preempted.add(id(old))
            old.cancel()
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info("收到取消请求")
        return CancelResponse.ACCEPT

    def _detach(self, sid, goal_handle):
        """只注销自己这一条：抢占时新 goal 可能已经占住了 sid 的位置。"""
        if self._goals.get(sid) is goal_handle:
            self._goals.pop(sid, None)
        self._preempted.discard(id(goal_handle))
        self._active = max(self._active - 1, 0)
        self._last_tick = time.monotonic()

    def _feedback_loop(self, goal_handle, sid, total_ms):
        """按 5 Hz 发反馈，返回 'done' / 'cancel' / 'locked'。"""
        total_s = max(int(total_ms), 1) / 1000.0
        t0 = time.monotonic()
        end = t0 + total_s
        fb = MoveServo.Feedback()
        last_pos = 0

        while True:
            if self._locked:
                return "locked"
            if goal_handle.is_cancel_requested:
                return "cancel"

            now = time.monotonic()
            pos = self._read_position(sid)
            if pos is not None:
                last_pos = pos          # 读不到就沿用上一次真值，别把 0 当位置发出去
            fb.current_pwm = last_pos
            fb.progress = float(min((now - t0) / total_s, 1.0))
            goal_handle.publish_feedback(fb)
            self._last_tick = time.monotonic()

            if now >= end:
                return "done"
            # 读 PRAD 花掉的时间从 0.2 s 里扣掉，保证真的是 5 Hz
            left = FB_PERIOD_S - (time.monotonic() - now)
            if left > 0:
                time.sleep(left)

    def _execute(self, goal_handle):
        sid = int(goal_handle.request.id)
        target = int(goal_handle.request.pwm)
        asked = int(goal_handle.request.duration_ms)
        if asked <= 0:
            asked = int(self._p("default_duration_ms"))

        self._goals[sid] = goal_handle
        self._active += 1
        self._last_tick = time.monotonic()

        result = MoveServo.Result()
        cur = self._read_position_retry(sid, tries=2)

        # 已经在目标位：不发 move，直接成功（避免原地抖一下）
        if cur is not None and abs(cur - target) <= ON_TARGET:
            self.get_logger().info(f"id={sid} 已在目标位 {cur}，无需动作")
            result.success = True
            result.final_pwm = cur
            result.message = "already in place"
            result.code = CODE_OK
            goal_handle.succeed()
            self._detach(sid, goal_handle)
            return result

        dur = plan_duration(cur, target, asked, int(self._p("speed_pwm_per_s")))

        if not self._send(enable_cmd(sid)):
            result.success = False
            result.message = "serial lost (PULR)"
            result.code = CODE_SERIAL
            goal_handle.abort()
            self._detach(sid, goal_handle)
            return result
        if not self._send(move_cmd(sid, target, dur)):
            result.success = False
            result.message = "serial lost (move)"
            result.code = CODE_SERIAL
            goal_handle.abort()
            self._detach(sid, goal_handle)
            return result

        self.get_logger().info(f"id={sid} {cur} → {target}，计划 {dur} ms")

        status = self._feedback_loop(goal_handle, sid, dur)

        if status == "locked":
            result.success = False
            result.message = "locked by watchdog"
            result.code = CODE_LOCKED
            result.final_pwm = 0
            goal_handle.abort()
            self.get_logger().error(f"id={sid} 动作被看门狗打断")
            self._detach(sid, goal_handle)
            return result

        if status == "cancel":
            if id(goal_handle) in self._preempted:
                self._preempted.discard(id(goal_handle))
                # 抢占不是取消：扭力保持，新 goal 从当前位置接着走
                result.success = False
                result.message = "preempted"
                result.code = CODE_PREEMPTED
                self.get_logger().warn(f"id={sid} 被新 goal 抢占，交棒")
            else:
                self.get_logger().warn(f"id={sid} 收到取消：停动作 → 回中位 → 卸力")
                self._send(move_cmd(sid, MID_PWM, CANCEL_MS), force=True)
                time.sleep(CANCEL_SETTLE_S)
                self._send(release_cmd(sid), force=True)
                result.success = False
                result.message = "canceled"
                result.code = CODE_CANCELED
            goal_handle.canceled()
            final = self._read_position_retry(sid)
            result.final_pwm = final if final is not None else 0
            self._detach(sid, goal_handle)
            return result

        # 闭环校验：读回 → 没到位就重发（+3 加分项）
        final = self._read_position_retry(sid)
        retries = int(self._p("prad_retry"))
        while retries > 0 and final is not None and abs(final - target) > ON_TARGET:
            self.get_logger().warn(
                f"id={sid} 未到位 target={target} actual={final} → 重发")
            again = max(dur, RETRY_EXTRA_MS)
            self._send(move_cmd(sid, target, again))
            self._feedback_loop(goal_handle, sid, again)
            final = self._read_position_retry(sid)
            retries -= 1
            if final is not None and abs(final - target) <= ON_TARGET:
                self.get_logger().info(f"id={sid} 重发后到位 actual={final}")

        if final is None:
            result.success = False
            result.message = "PRAD read fail"
            result.code = CODE_SERIAL
            result.final_pwm = 0
        else:
            diff = abs(final - target)
            result.final_pwm = final
            if diff > ON_TARGET:
                result.success = False
                result.code = CODE_NOT_IN_PLACE
                result.message = f"Not in place: target {target}, actual {final}, diff {diff}"
                self.get_logger().warn(result.message)
            else:
                result.success = True
                result.code = CODE_OK
                result.message = "success"
                self.get_logger().info(f"id={sid} 到位 {final}")

        goal_handle.succeed()
        self._detach(sid, goal_handle)
        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ServoDriver()
    except Exception:
        # launch 只会报 process has died，看不到真因，这里自己打 traceback
        traceback.print_exc()
        rclpy.shutdown()
        raise

    # 必须多线程：execute 里有 sleep 和串口等待，单线程会挡住 cancel 回调，
    # 现象就是「Ctrl+C 了，舵机还在转」
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
