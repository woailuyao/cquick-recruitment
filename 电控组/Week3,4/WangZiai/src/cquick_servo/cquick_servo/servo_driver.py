#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""servo_driver.py —— 总线舵机驱动节点

驱动共三层，若硬件更换只改前两层，ROS2 层可原样复用：

    ┌─────────────────────────────────────────────┐
    │ 3. ServoDriverNode   ROS2 层                 │  ← 参数、Action、话题、参数校验
    │      依赖「协议层接口」，不关心串口细节         │
    ├─────────────────────────────────────────────┤
    │ 2. BusServo          协议层                  │  ← #001P1500T1000! 的拼装与解析
    │      只依赖「传输层接口」                     │
    ├─────────────────────────────────────────────┤
    │ 1. Transport         传输层（可替换）         │  ← SerialTransport / MockTransport / CanTransport…
    └─────────────────────────────────────────────┘

其他驱动可复用（CAN 电机、GPIO、PWM 舵机板）：
  · 换 Transport：只要实现 write/read/close 三个方法；
  · 换 Protocol：把「字节 ↔ 物理量」的编解码写进去；
  · ROS2 层几乎不用动（参数不缓存、Action 四回调、限速校验都是通用的）。
"""

from __future__ import annotations

import os
import re
import sys
import atexit
import traceback

try:
    import termios  # 串口被拔/pty 消失时 pyserial 会抛 termios.error（不是 OSError）
except ImportError:  # 非 Linux 平台
    termios = None
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Callable, Optional, TypeVar, cast

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from cquick_servo_interfaces.action import MoveServo
from cquick_servo_interfaces.msg import ServoCommand

# 为SerialTransport的_do(fn)提供范型
T = TypeVar("T")


# 协议常量
PWM_ABS_MIN = 500
PWM_ABS_MAX = 2500
PWM_NEUTRAL = 1500
PWM_TOLERANCE = 30  # 到点判定容差
CMD_TERM = "!"
BUSY_RESERVE_TIMEOUT = 30.0  # 预占后多久还没开始执行就自动释放


# ============================================================================
# 1. 传输层
# ============================================================================
class Transport(ABC):
    """字节流的收发接口。CAN / 网口 / 别的板子，实现这个类即可。"""

    @abstractmethod
    def write(self, data: bytes) -> None: ...

    @abstractmethod
    def read(self, size: int = 64) -> bytes: ...

    @abstractmethod
    def close(self) -> None: ...


class SerialTransport(Transport):
    """真实硬件：USB 转 TTL（CH340 / CP2102）。断线后自动重连。"""

    def __init__(
        self,
        port: str,
        baudrate: int,
        r_timeout: float = 0.05,
        w_timeout: float = 0.5,
        log=None,
    ) -> None:
        # 延迟导入，没有 pyserial 时只有用真串口才报错
        import serial

        self._serial = serial  # 存储serial模块
        self._port = port  # 端口
        self._baud = int(baudrate)  # 波特率
        self._r_timeout = r_timeout  # 读超时等待
        self._w_timeout = w_timeout  # 写超时等待
        self._log = log or (lambda msg: None)  # 加载日志
        self._lock = threading.Lock()  # I/O锁
        self._reconnecting = threading.Event()  # 重连
        self.reconnects = 0  # 重连计数
        self._io_errors: tuple = (OSError,)  # 拔 USB 时不同层的异常抛出
        if termios is not None:
            self._io_errors += (termios.error,)
        self._ser = self._open_with_retry()  # 加载基础配置

    def _open_with_retry(self, tries: int = 3, delay: float = 1.0):
        """打开串口，失败重试；全部失败时抛 RuntimeError。"""
        last_exc = None

        for i in range(1, tries + 1):
            try:
                return self._open()
            except Exception as exc:  # noqa: BLE001 - 打不开立即重试，最后统一报错
                last_exc = exc
                if self._log:
                    self._log(f"打开串口失败（{i}/{tries}）：{exc}")
                if i < tries:
                    time.sleep(delay)
        raise RuntimeError(f"打不开串口 {self._port}：{last_exc}\n") from last_exc

    def _open(self):
        return self._serial.Serial(
            self._port,
            baudrate=self._baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self._r_timeout,
            write_timeout=self._w_timeout,
        )

    def _reconnect(self, exc: Exception) -> None:
        """串口掉了：关掉旧句柄，等设备回来后再重开（最多等 6 秒）。"""
        self.reconnects += 1  # 重连计数
        self._log(f"串口断开（{exc}），第 {self.reconnects} 次重连：{self._port}")

        try:
            self._ser.close()  # 尝试关闭串口
        except Exception:  # noqa: BLE001
            pass
        for attempt in range(3):
            time.sleep(2.0)  # 等待重新连接
            try:
                self._ser = self._open()  # 尝试重连
                self._log(f"串口重连成功：{self._port}")
                return
            except Exception as open_exc:  # noqa: BLE001
                if attempt == 2:
                    raise
                self._log(f"重连失败（{open_exc}），继续等设备…")

    def _do(self, fn: Callable[[], T]) -> T:
        for attempt in range(3):  # 尝试连接三次

            while self._reconnecting.is_set():  # 等待其他线程重连
                self._reconnecting.wait(timeout=0.05)
            try:
                with self._lock:  # 原子化
                    return fn()  # 连接成功后自动跳走
            except self._io_errors as exc:
                if attempt == 2:  # 两次尝试后直接原样抛出
                    raise
                self._reconnecting.set()
                try:
                    self._reconnect(exc)  # 尝试重连
                finally:
                    self._reconnecting.clear()

        raise RuntimeError("串口操作失败：已重试3次")  # 防报错

    def write(self, data: bytes) -> None:
        def _write():

            self._ser.reset_input_buffer()  # 清掉上一轮的残留应答
            self._ser.write(data)  # 写入数据
            self._ser.flush()  # 等待数据发出

        self._do(_write)

    def read(self, size: int = 64) -> bytes:  # 最多读64字节
        def _read() -> bytes:

            data = self._ser.read_until(CMD_TERM, size)  # 读到指令结束标志即返回

            return data if data else b""  # 防御其他实现可能返回 None / 空

        return self._do(_read)

    def close(self) -> None:
        try:
            with self._lock:  # 保持线程原子化
                self._ser.close()
        except Exception:  # noqa: BLE001 - 防止关闭失败节点崩溃
            pass


class MockTransport(Transport):
    """虚拟实现 1~6 号舵机。

    用法：ros2 launch ... port:=mock
    这样 Action / 参数 / bag 全链路都能在没舵机的情况下验证。
    """

    _RE_PID = re.compile(r"^#(\d{3})PID!$")
    _RE_PRAD = re.compile(r"^#(\d{3})PRAD!$")
    _RE_MOVE = re.compile(r"^#(\d{3})P(\d{4})T(\d{4})!$")

    def __init__(self) -> None:

        self._pos: dict[int, int] = {}
        self._reply = b""
        # CQ_MOCK_DEV=60 → 每个舵机第一次动作只走到 目标+60，用于测试 PRAD 闭环重发
        self._dev_left = int(os.environ.get("CQ_MOCK_DEV", "0") or 0)
        self._dev_always = int(os.environ.get("CQ_MOCK_DEV_ALWAYS", "0") or 0)

    def write(self, data: bytes) -> None:

        cmd = data.decode("ascii", errors="ignore").strip()
        self._reply = b""

        if m := self._RE_PID.match(cmd):
            sid = int(m.group(1))
            self._pos.setdefault(sid, PWM_NEUTRAL)
            self._reply = f"#{sid:03d}PID!".encode()

        elif m := self._RE_PRAD.match(cmd):
            sid = int(m.group(1))
            self._reply = f"#{sid:03d}P{self._pos.get(sid, PWM_NEUTRAL):04d}!".encode()

        elif m := self._RE_MOVE.match(cmd):
            sid, pwm = int(m.group(1)), int(m.group(2))
            delta = self._dev_left + self._dev_always
            self._pos[sid] = max(PWM_ABS_MIN, min(PWM_ABS_MAX, pwm + delta))
            self._dev_left = 0  # 一次性偏差用完即止；_dev_always 常驻
            self._reply = f"#{sid:03d}P{self._pos[sid]:04d}!".encode()

        # PULR!/PULK! 只回一个空应答（真机也是这样）

    def read(self, size: int = 64) -> bytes:
        reply, self._reply = self._reply, b""
        return reply

    def close(self) -> None:
        pass


# ============================================================================
# 2. 协议层
# ============================================================================
class BusServo:
    """把「舵机指令」翻译成「字节」。换协议（CAN 帧、PWM 值）只改这一层。"""

    _RE_POS = re.compile(r"#(\d{3})P(\d{4})" + re.escape(CMD_TERM))  # 宽松读取串口关键帧

    def __init__(self, transport: Transport, settle: float = 0.02) -> None:
        self._t = transport  # 登记传输层
        self._settle = settle  # 发完等舵机处理的时间
        self._bus_lock = threading.Lock()  # 总线是半双工，这里使用I/O锁实现读写串行化

    def _cmd(self, text: str) -> str:  # 向总线传输指令（需要应答）
        with self._bus_lock:
            self._t.write(text.encode("ascii"))  # 按ASCII码传输指令
            time.sleep(self._settle)  # 等舵机把应答放进缓冲区

            return (
                self._t.read(64).decode("ascii", errors="ignore").strip()
            )  # 收应答并直接丢弃噪声字节

    def _cmd_fast(self, text: str) -> None:  # 向总线传输指令（不需要应答）
        with self._bus_lock:
            self._t.write(text.encode("ascii"))  # 按ASCII码传输指令

    # -------------------------------- 动 作 封 装 ---------------------------------------
    def move(self, sid: int, pwm: int, duration_ms: int) -> str:  # 旋转pwm
        duration_ms = max(1, min(9999, int(duration_ms)))  # 进行最大值夹取
        # 固件对 P...T... 指令不回显，用 _cmd_fast 只写不读，省掉 50 ms 读超时
        self._cmd_fast(f"#{sid:03d}P{pwm:04d}T{duration_ms:04d}{CMD_TERM}")
        return ""

    def torque_on(self, sid: int) -> str:  # 舵机上力
        self._cmd_fast(f"#{sid:03d}PULR{CMD_TERM}")  # PULR 不回显
        return ""

    def torque_off(self, sid: int) -> str:  # 舵机卸力
        self._cmd_fast(f"#{sid:03d}PULK{CMD_TERM}")  # PULK 不回显
        return ""

    def move_fast(self, sid: int, pwm: int, duration_ms: int) -> None:  # 快速旋转pwm
        duration_ms = max(1, min(9999, int(duration_ms)))  # 进行最大值夹取
        self._cmd_fast(f"#{sid:03d}P{pwm:04d}T{duration_ms:04d}{CMD_TERM}")

    def torque_on_fast(self, sid: int) -> None:  # 舵机快速上力
        self._cmd_fast(f"#{sid:03d}PULR{CMD_TERM}")

    def torque_off_fast(self, sid: int) -> None:  # 舵机快速卸力
        self._cmd_fast(f"#{sid:03d}PULK{CMD_TERM}")

    def ping(self, sid: int) -> str:  # 读取舵机ID
        return self._cmd(f"#{sid:03d}PID{CMD_TERM}")

    def read_pos(self, sid: int) -> Optional[int]:  # 读回PWM
        """读回当前 PWM；读不到（舵机不回话/总线忙）返回 None。"""
        raw = self._cmd(f"#{sid:03d}PRAD{CMD_TERM}")

        # group(2) 是 PWM 位置， group(1) 是 ID， group(0) 是整串
        for m in self._RE_POS.finditer(raw):
            if int(m.group(1)) == int(sid):  # 确定 ID 匹配
                return int(m.group(2))

        return None

    def close(self) -> None:
        with self._bus_lock:
            self._t.close()


# ============================================================================
# 3. ROS2 节点层
# ============================================================================
class ServoDriverNode(Node):
    """参数、Action Server、/servo/command 订阅、/servo/state 发布。"""

    # 全部参数名
    P_PORT = "port"
    P_BAUD = "baudrate"
    P_PWM_MIN = "pwm_min"
    P_PWM_MAX = "pwm_max"
    P_DEFAULT_DUR = "default_duration_ms"
    P_SERVO_IDS = "servo_ids"
    P_STARTUP_RESET = "startup_reset"
    P_WATCHDOG = "watchdog_timeout"
    P_RESEND = "resend_threshold"
    P_SUCCESS = "success_tolerance"
    P_ACCEPT_WAIT = "accept_wait_timeout"
    P_SEND_WAIT = "send_wait_timeout"
    P_STOP_ON_FINISH = "stop_on_finish"
    P_RELEASE_ON_FINISH = "release_on_finish"
    P_ECHO_TTL = "echo_ttl"

    def __init__(self) -> None:
        super().__init__("servo_driver")

        # ----------------------- 参 数 层 --------------------------
        # 声明参数（并设置默认值）
        self.declare_parameter(self.P_PORT, "/dev/cq_servo")
        self.declare_parameter(self.P_BAUD, 115200)
        self.declare_parameter(self.P_PWM_MIN, 900)
        self.declare_parameter(self.P_PWM_MAX, 2100)
        self.declare_parameter(self.P_DEFAULT_DUR, 800)
        self.declare_parameter(self.P_SERVO_IDS, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.declare_parameter(self.P_STARTUP_RESET, True)
        self.declare_parameter(self.P_SUCCESS, PWM_TOLERANCE)
        self.declare_parameter(self.P_WATCHDOG, 1.0)  # 0 = 关闭看门狗
        self.declare_parameter(self.P_RESEND, PWM_TOLERANCE)  # 0 = 关闭闭环重发
        self.declare_parameter(self.P_ACCEPT_WAIT, 3.0)  # 舵机忙时最多等多久再拒（0 = 立即拒）
        self.declare_parameter(self.P_SEND_WAIT, 1.0)  # 单条指令抢舵机锁的等待上限
        self.declare_parameter(self.P_STOP_ON_FINISH, True)  # 动作结束后钉住，停止残余插值
        self.declare_parameter(self.P_RELEASE_ON_FINISH, False)  # 动作结束后全部卸力（PULK）
        self.declare_parameter(self.P_ECHO_TTL, 1.0)  # 回声去重窗口（秒），太小会自激抽搐
        self.add_on_set_parameters_callback(self._on_set_parameters)  # 参数越界时拒绝写入
        self._locks: dict[int, threading.Lock] = {}  # 每个舵机一把I/O锁
        self._last_cmd_time = time.monotonic()  # 看门狗状态：最后一次指令时间
        self._torqued_off_ids: set[int] = set()  # 看门狗状态：是否已卸力

        # ----------------------- 传 输 + 协 议 层  -------------------------
        port = self._p_str(self.P_PORT)
        baud = self._p_int(self.P_BAUD)

        if port.startswith("mock"):  # 提供虚拟舵机mock的选择
            self.get_logger().warn("▸ 传输层：MockTransport（模拟，不连真硬件）")
            self._transport: Transport = MockTransport()
        else:  # 使用真实舵机
            self._transport = SerialTransport(port, baud, log=lambda m: self.get_logger().warn(m))

        self._servo = BusServo(self._transport)  # 启动协议层

        # --------------------- Action Server ------------------------
        self._action_group = ReentrantCallbackGroup()
        self._busy_ids: dict[int, float] = {}  # 同一舵机同时只允许一个 goal（不同舵机可并行）
        self._executing_ids: set[int] = (
            set()
        )  # sid -> 预占时间戳；超过 BUSY_RESERVE_TIMEOUT 且未开始执行会自动释放
        self._goal_lock = threading.Lock()  #
        self._closed = False  # 防重复销毁
        self._action_server = ActionServer(
            self,
            MoveServo,
            "move_servo",
            execute_callback=self._execute_move,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=self._on_cancel,
            callback_group=self._action_group,
        )

        # ------------------------ 话 题 ----------------------------
        self._cmd_sub = self.create_subscription(
            ServoCommand, "/servo/command", self._on_command, 10, callback_group=self._action_group
        )
        self._state_pub = self.create_publisher(ServoCommand, "/servo/state", 10)  # 发布舵机状态
        self._command_pub = self.create_publisher(
            ServoCommand, "/servo/command", 10
        )  # 发布回声指令
        self._echo_pending: dict[tuple[int, int, int], float] = (
            {}
        )  # 自己刚发出的回声：key -> 时间戳。匹配到就"消费掉"，不会挡住之后真正的重复指令
        self._echo_lock = threading.Lock()  # 防止多线程共用

        # ---------------------- 安全下电服务 --------------------------
        self._shutdown_requested = False  # 收到下电请求后拒绝新目标
        self._shutdown_srv = self.create_service(
            Trigger, "/servo_shutdown", self._on_shutdown, callback_group=self._action_group
        )

        # ------------------------ 看 门 狗 ----------------------------
        self.create_timer(0.2, self._watchdog_tick)

        # ------------------------ 启 动 自 检 ---------------------------
        if self._p_bool(self.P_STARTUP_RESET):
            self._startup_reset()

        # 启动横幅：标题与正文都用「4 列前缀」（┌──␠ / │␠␠␠），保证左侧对齐
        ids_text = ", ".join(str(i) for i in self._p_int_list(self.P_SERVO_IDS))
        watchdog = float(self._p(self.P_WATCHDOG))
        self.get_logger().info(
            "\n"
            "┌── servo_driver 就绪 ──────────────────────────\n"
            f"│   端口    {port} @ {baud}\n"
            f"│   舵机    [{ids_text}]\n"
            f"│   限位    {self._p_int(self.P_PWM_MIN)} ~ {self._p_int(self.P_PWM_MAX)}"
            f" · 默认 {self._p_int(self.P_DEFAULT_DUR)} ms\n"
            f"│   监控    看门狗 {watchdog:g} s · 重发阈值 ±{self._p_int(self.P_RESEND)}\n"
            "└───────────────────────────────────────────────"
        )

    # ------------------------------------------------------------------ 参 数
    def _on_set_parameters(self, params) -> SetParametersResult:  # noqa: C901 - 逐项校验，平铺更易读
        """参数变更校验：不合法的写入直接拒绝，并给出原因。"""
        # 用「即将生效的完整参数集」做校验，避免先改 min 后改 max 的中间态误判
        incoming = {p.name: p.value for p in params}  # 获取变更参数
        pwm_min = cast(int, incoming.get(self.P_PWM_MIN, self._p_int(self.P_PWM_MIN)))
        pwm_max = cast(int, incoming.get(self.P_PWM_MAX, self._p_int(self.P_PWM_MAX)))
        try:
            if not (PWM_ABS_MIN <= pwm_min <= PWM_ABS_MAX):  # pwm校验
                return SetParametersResult(
                    successful=False, reason=f"pwm_min 必须在 {PWM_ABS_MIN}~{PWM_ABS_MAX}"
                )
            if not (PWM_ABS_MIN <= pwm_max <= PWM_ABS_MAX):
                return SetParametersResult(
                    successful=False, reason=f"pwm_max 必须在 {PWM_ABS_MIN}~{PWM_ABS_MAX}"
                )
            if pwm_min >= pwm_max:
                return SetParametersResult(successful=False, reason="pwm_min 必须小于 pwm_max")

            if (
                self.P_DEFAULT_DUR in incoming and cast(int, incoming[self.P_DEFAULT_DUR]) <= 0
            ):  # 运动时间校验
                return SetParametersResult(successful=False, reason="default_duration_ms 必须 > 0")

            if self.P_BAUD in incoming and cast(int, incoming[self.P_BAUD]) <= 0:  # 波特率校验
                return SetParametersResult(successful=False, reason="baudrate 必须 > 0")

            if self.P_SERVO_IDS in incoming and len(incoming[self.P_SERVO_IDS]) == 0:  # 舵机ID校验
                return SetParametersResult(successful=False, reason="servo_ids 不能为空")

            if (
                self.P_WATCHDOG in incoming and cast(float, incoming[self.P_WATCHDOG]) < 0
            ):  # 看门狗校验
                return SetParametersResult(successful=False, reason="watchdog_timeout 不能为负")

            if (
                self.P_RESEND in incoming and cast(int, incoming[self.P_RESEND]) < 0
            ):  # 重发送限制校验
                return SetParametersResult(successful=False, reason="resend_threshold 不能为负")

            if (
                self.P_ACCEPT_WAIT in incoming
                and cast(float, incoming[self.P_ACCEPT_WAIT]) < 0
            ):  # 忙等待校验
                return SetParametersResult(successful=False, reason="accept_wait_timeout 不能为负")

            if (
                self.P_SEND_WAIT in incoming and cast(float, incoming[self.P_SEND_WAIT]) < 0
            ):  # 指令等待校验
                return SetParametersResult(successful=False, reason="send_wait_timeout 不能为负")

            if self.P_STOP_ON_FINISH in incoming and not isinstance(
                incoming[self.P_STOP_ON_FINISH], bool
            ):  # 钉住开关校验
                return SetParametersResult(
                    successful=False, reason="stop_on_finish 必须是布尔值"
                )

            if self.P_RELEASE_ON_FINISH in incoming and not isinstance(
                incoming[self.P_RELEASE_ON_FINISH], bool
            ):  # 卸力开关校验
                return SetParametersResult(
                    successful=False, reason="release_on_finish 必须是布尔值"
                )

            if self.P_ECHO_TTL in incoming:  # 回声窗口不能太小
                v = float(incoming[self.P_ECHO_TTL])
                if v < 0.2:
                    return SetParametersResult(
                        successful=False, reason="echo_ttl 不能小于 0.2 秒"
                    )

        except (TypeError, ValueError) as exc:  # 未知错误反馈
            return SetParametersResult(successful=False, reason=f"参数类型错误：{exc}")

        return SetParametersResult(successful=True)

    def _p(self, name: str) -> Any:
        """
        读参数的统一入口：每次都实时读，不缓存
        带类型的取值助手：rclpy 的 Parameter.value 在类型存根里是 Any | None，
        这里用 cast 声明「我们按参数声明时的类型取」，避免运行期无害、VS却报错
        """
        return self.get_parameter(name).value

    def _p_int(self, name: str) -> int:
        v = self._p(name)  # 避免浮点数报错
        return int(v) if v is not None else 0

    def _p_str(self, name: str) -> str:
        return cast(str, self._p(name))

    def _p_bool(self, name: str) -> bool:
        return cast(bool, self._p(name))

    def _p_int_list(self, name: str) -> list[int]:
        return cast("list[int]", self._p(name))

    def _pwm_range(self) -> tuple[int, int]:
        return self._p_int(self.P_PWM_MIN), self._p_int(self.P_PWM_MAX)

    # ------------------------------------------------------------------ 公 共 发 送
    def _send(self, sid: int, pwm: int, duration_ms: int, echo: bool = True) -> bool:
        """所有指令的唯一出口：Action、话题、启动复位都走这里。"""
        lo, hi = self._pwm_range()
        pwm = max(lo, min(hi, int(pwm)))  # 双保险：上层校验之外再夹一次

        lock = self._locks.setdefault(int(sid), threading.Lock())  # 判断舵机是否正在工作
        lock_wait = max(0.05, float(self._p(self.P_SEND_WAIT)))  # 忙时等一会，别立刻丢指令
        if not lock.acquire(timeout=lock_wait):
            self.get_logger().warn(f"⚠ [舵机 {sid}] 等待 {lock_wait:.1f}s 仍忙 · 丢弃这条指令")
            return False

        try:
            if int(sid) in self._torqued_off_ids:  # 看门狗卸过力，先上力再动
                self.get_logger().info(f"▸ [舵机 {sid}] 已卸力 → 自动 PULR")
                self._servo.torque_on(int(sid))
                self._torqued_off_ids.discard(int(sid))
            self._servo.move(sid, pwm, int(duration_ms))  # 操作pwm旋转
            self._publish_state(sid, pwm, duration_ms)  # 发布指令内容及回声
            if echo:
                self._publish_command_echo(sid, pwm, int(duration_ms))
            self._last_cmd_time = time.monotonic()  # 喂狗
            self.get_logger().debug(f"#{sid:03d} → PWM {pwm} in {duration_ms} ms")  # 输出debug日志
            return True
        except Exception as exc:  # 串口掉线也不能把节点带崩
            self.get_logger().error(f"✗ [舵机 {sid}] 发送失败：{exc}")
            return False
        finally:
            lock.release()  # 释放锁

    def _publish_state(
        self, sid: int, pwm: int, duration_ms: int = 0
    ) -> None:  # 向/servo/state发送指令的实际执行内容
        msg = ServoCommand()
        msg.id = int(sid)
        msg.pwm = int(pwm)
        msg.duration_ms = int(duration_ms)
        self._state_pub.publish(msg)

    def _publish_command_echo(
        self, sid: int, pwm: int, duration_ms: int
    ) -> None:  # 向/servo/command发送我的指令内容
        """把实际下发的指令回声到 /servo/command，供 ros2 bag 录制。"""
        msg = ServoCommand()
        msg.id = int(sid)
        msg.pwm = int(pwm)
        msg.duration_ms = int(duration_ms)
        with self._echo_lock:
            self._echo_pending[(int(sid), int(pwm), int(duration_ms))] = time.monotonic()
            self._command_pub.publish(msg)

    # ------------------------------------------------------------------ Action
    def _on_goal(self, goal_request) -> GoalResponse:
        """目标预校验：不合法直接 REJECT，不进执行队列。"""
        if self._shutdown_requested:  # 已请求下电，拒绝新目标
            self.get_logger().warn("✗ 已进入下电流程，拒绝新目标")
            return GoalResponse.REJECT
        ids = self._p_int_list(self.P_SERVO_IDS)
        lo, hi = self._pwm_range()
        g = goal_request

        if int(g.id) not in ids:  # 目标校验
            self.get_logger().warn(f"✗ 拒绝 [舵机 {g.id}] · id 不在 servo_ids={ids}")
            return GoalResponse.REJECT

        if not (lo <= int(g.pwm) <= hi):  # pwm范围校验
            self.get_logger().warn(f"✗ 拒绝 [舵机 {g.id}] · pwm {g.pwm} 超出 {lo}~{hi}")
            return GoalResponse.REJECT

        if int(g.duration_ms) <= 0:  # 时延校验
            self.get_logger().warn(f"✗ 拒绝 [舵机 {g.id}] · duration {g.duration_ms} ms 必须 > 0")
            return GoalResponse.REJECT

        sid = int(g.id)  # 同一舵机只允许一个 goal：预占 id，避免 ACCEPTED 状态下 abort 非法
        wait_s = max(0.0, float(self._p(self.P_ACCEPT_WAIT)))  # 忙时先等一会，别立刻拒绝
        deadline = time.monotonic() + wait_s
        while True:
            with self._goal_lock:
                if sid not in self._busy_ids:
                    self._busy_ids[sid] = time.monotonic()
                    break
            if time.monotonic() >= deadline:
                self.get_logger().warn(f"⚠ [舵机 {sid}] 等待 {wait_s:.1f}s 仍忙 · 拒绝 goal")
                return GoalResponse.REJECT
            time.sleep(0.02)

        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle) -> None:
        """goal 已被 _on_goal 预占；这里只负责启动执行。"""
        goal_handle.execute()

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info(
            "▸ 收到取消请求"
        )  # 是否真的停下由 _execute_move_impl 每步检查 goal_handle.is_cancel_requested 决定
        return CancelResponse.ACCEPT

    def _execute_move(self, goal_handle):
        """包一层异常处理：驱动/串口出错时返回带原因的 result，而不是空的 ABORTED。"""
        self._executing_ids.add(int(goal_handle.request.id))
        try:
            return self._execute_move_impl(goal_handle)
        except Exception as e:  # noqa: BLE001 - 任何异常都要如实回给客户端
            self.get_logger().error(f"执行异常：{e!r}\n{traceback.format_exc()}")
            try:
                goal_handle.abort()  # 防止goal_handle.abort()异常
            except Exception:
                pass
            result = MoveServo.Result()
            result.success = False
            result.final_pwm = 0
            result.message = f"exception: {type(e).__name__}"[:200]
            return result
        finally:
            if self._p_bool(self.P_RELEASE_ON_FINISH):
                self._release_all("动作结束自动卸力")
            sid = int(goal_handle.request.id)
            with self._goal_lock:
                self._executing_ids.discard(sid)
                self._busy_ids.pop(sid, None)

    def _release_all(self, reason: str) -> int:
        """对所有 servo_ids 卸力（PULK），返回本次真正卸掉的台数。"""
        ids = {int(s) for s in self._p_int_list(self.P_SERVO_IDS)}
        pending = ids - self._torqued_off_ids
        done = 0
        for sid in sorted(pending):
            try:
                self._servo.torque_off(sid)
                self._torqued_off_ids.add(sid)
                done += 1
            except Exception as exc:  # noqa: BLE001 - 卸力失败不该让节点崩
                self.get_logger().warn(f"⚠ [舵机 {sid}] 卸力失败：{exc}")
        if done:
            self.get_logger().info(f"▸ {reason}：已 PULK {done} 台（共 {len(ids)} 台）")
        return done

    def _execute_move_impl(self, goal_handle):
        """分步插值 + 高频 feedback + 每步检查取消 + 结束后回读校验。"""
        g = goal_handle.request
        sid = int(g.id)
        target = int(g.pwm)
        duration_ms = int(g.duration_ms)

        if (
            goal_handle.is_cancel_requested
        ):  # 取消可能在我们进入执行前就到了：先查一次，避免舵机先动一小步再停
            self.get_logger().warn(f"⚠ [舵机 {sid}] 启动前已被取消 · 未动作")
            goal_handle.canceled()
            result = MoveServo.Result()
            result.success = False
            result.final_pwm = 0
            result.message = "canceled before start"
            return result

        start = self._servo.read_pos(sid)  # 读不到就从中立位起算
        if start is None:
            start = PWM_NEUTRAL
        steps = max(2, min(int(duration_ms / 100), 200))  # 10 Hz 插值
        period = duration_ms / 1000.0 / steps
        step_ms = max(1, int(round(period * 1000)))
        feedback = MoveServo.Feedback()
        self.get_logger().info(
            f"▸ [舵机 {sid}] {start} → {target} · {duration_ms} ms · {steps} 段"
        )
        deadline = time.monotonic()  # 绝对时间轴
        behind = 0  # 落后次数，用于诊断

        for i in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f"⚠ [舵机 {sid}] 取消（第 {i}/{steps} 段）")
                self._send(sid, int(start), 100)  # 恢复原位
                goal_handle.canceled()
                result = MoveServo.Result()
                result.success = False
                result.final_pwm = int(start)
                result.message = "canceled"
                return result

            pwm = int(round(start + (target - start) * i / steps))
            if not self._send(sid, pwm, step_ms):
                goal_handle.abort()
                result = MoveServo.Result()
                result.success = False
                result.final_pwm = pwm
                result.message = f"在第 {i}/{steps} 步发送失败（舵机忙）"
                return result

            feedback.current_pwm = pwm
            feedback.progress = float(i) / steps
            goal_handle.publish_feedback(feedback)  # ≥5 Hz
            deadline += period  # 落后自动重新对齐
            slack = deadline - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                behind += 1  # 落后次数+1
                deadline = time.monotonic()

        if behind:  # 反馈落后次数
            self.get_logger().warn(f"⚠ 串口应答偏慢：{steps} 段中有 {behind} 段超时，实际时长拉长")
        time.sleep(max(0.2, period * 2))  # 等最后一步真正转到位再回读
        actual = self._servo.read_pos(sid)  # 结束后回读实际位置

        threshold = self._p_int(self.P_RESEND)
        resent = False
        if actual is not None and threshold > 0 and abs(actual - target) > threshold:  # 超额补发
            self.get_logger().warn(
                f"⚠ [舵机 {sid}] 偏差 {abs(actual - target)} > {threshold} → 补发一次"
            )
            self._send(sid, target, self._p_int(self.P_DEFAULT_DUR))
            time.sleep(max(0.3, self._p_int(self.P_DEFAULT_DUR) / 1000.0 * 1.2))
            again = self._servo.read_pos(sid)  # 读取真实pwm
            if again is not None:
                actual = again
            resent = True

        if actual is not None and self._p_bool(self.P_STOP_ON_FINISH):
            # T=0 钉住当前位置：总线舵机收到新指令会立刻停止上一条插值，避免"播完还在动"
            self._send(sid, int(actual), 0, echo=False)
            self.get_logger().info(f"▸ [舵机 {sid}] 钉住 {actual}（T=0，停止残余插值）")

        result = MoveServo.Result()  # 编辑action结果
        if actual is None:
            result.success = False
            result.final_pwm = target
            result.message = "读不到位置（PRAD 无应答）"
        else:
            error = abs(actual - target)
            tol = abs(self._p_int(self.P_SUCCESS))
            if error <= tol:
                self.get_logger().info(f"✓ [舵机 {sid}] 到位 {actual} · 误差 {error}")
            else:
                self.get_logger().warn(f"⚠ [舵机 {sid}] 未到位 {actual} · 误差 {error} > {tol}")
            result.success = error <= tol
            result.final_pwm = int(actual)
            suffix = " (resend)" if resent else ""
            result.message = f"actual={actual}, target={target}, error={error}{suffix}"
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()  # 误差超容差 → ABORTED，但 result 带原因

        self.get_logger().info(f"✓ [舵机 {sid}] 完成 · {result.message}")
        return result

    # ------------------------------------------------------------------ 看 门 狗
    def _watchdog_tick(self) -> None:
        """超过 watchdog_timeout 没有新指令 → 全部卸力，防止堵转发热。"""
        if self._shutdown_requested:  # 已下电，停止看门狗
            return
        timeout = float(self._p(self.P_WATCHDOG))
        if timeout <= 0:  # 0 = 关闭
            return
        ids = {int(s) for s in self._p_int_list(self.P_SERVO_IDS)}  # 获取舵机ID元组
        if not ids:
            return

        now = time.monotonic()  # 清掉"已接受但迟迟没开始执行"的预占（goal 被取消等情况）
        with self._goal_lock:
            stale = [
                sid
                for sid, ts in self._busy_ids.items()
                if sid not in self._executing_ids and now - ts > BUSY_RESERVE_TIMEOUT
            ]
            for sid in stale:
                self._busy_ids.pop(sid, None)
        if stale:
            self.get_logger().warn(f"⚠ 释放超时预占：{sorted(stale)}")

        self._torqued_off_ids &= ids  # 清掉不再存在的 ID，避免集合越积越脏
        if ids <= self._torqued_off_ids:  # 判断舵机卸力状态
            return
        if time.monotonic() - self._last_cmd_time < timeout:  # 判断是否超时
            return
        for sid in ids - self._torqued_off_ids:  # 使所有未卸力舵机卸力
            try:
                self._servo.torque_off(sid)
                self._torqued_off_ids.add(sid)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"看门狗卸力失败（舵机 {sid}）：{exc}")
        if ids <= self._torqued_off_ids:
            self.get_logger().warn(f"⚠ 看门狗：{timeout:.1f} s 无新指令 → 全部 PULR 关闭，已 PULK")

    # ------------------------------------------------------------------ 话 题
    def _on_command(self, msg: ServoCommand) -> None:
        """外部直接发指令（bag 回放走这条路径），与 Action 共用 _send。"""
        if self._shutdown_requested:  # 已下电，不再接受指令
            return
        duration = int(msg.duration_ms) or self._p_int(self.P_DEFAULT_DUR)  # duration_ms=0即未指定
        now = time.monotonic()
        with self._echo_lock:  # 防止并行发送
            ttl = max(0.2, float(self._p(self.P_ECHO_TTL)))  # 去重窗口
            for k in [k for k, t in self._echo_pending.items() if now - t > ttl]:  # 清理超时项
                del self._echo_pending[k]
            key = (int(msg.id), int(msg.pwm), duration)  # 再匹配
            if key in self._echo_pending:  # 自己绕回来的回声，消费掉后忽略
                del self._echo_pending[key]
                return
        self._send(int(msg.id), int(msg.pwm), duration)

    # ------------------------------------------------------------------ 启动复位
    def _startup_reset(self) -> None:
        for sid in self._p_int_list(self.P_SERVO_IDS):  # 不进行回声
            self._send(int(sid), PWM_NEUTRAL, self._p_int(self.P_DEFAULT_DUR), echo=False)
        self.get_logger().info("✓ 启动复位：全部舵机回 1500")

    # ------------------------------------------------------------------ 收尾
    def close_serial_safely(self) -> None:
        """兜底：只发 PULK + 关串口，不碰 rclpy（atexit 场景专用）。"""
        if getattr(self, "_closed", False):
            return
        self._closed = True
        try:
            for sid in self._p_int_list(self.P_SERVO_IDS):
                try:
                    self._servo.torque_off(int(sid))
                except Exception:
                    pass  # 退出路径上静默
        finally:
            try:
                self._servo.close()
            except Exception:
                pass

    def _on_shutdown(self, request, response):
        """安全下电：全部舵机 PULK 卸力，然后关闭串口。"""
        del request  # Trigger 的请求为空
        self._shutdown_requested = True
        ids = self._p_int_list(self.P_SERVO_IDS)
        self.get_logger().info(f"▸ 收到下电请求：对 {len(ids)} 个舵机执行 PULK")
        failed = []
        for sid in ids:
            try:
                self._servo.torque_off(int(sid))
                self.get_logger().info(f"✓ 舵机 {sid} 已卸力")
            except Exception as exc:  # noqa: BLE001 - 单个失败不影响其它舵机
                failed.append(f"{sid}({exc})")
                self.get_logger().error(f"✗ 舵机 {sid} 卸力失败：{exc}")
        try:
            self._servo.close()
        except Exception:  # noqa: BLE001 - 关闭失败不该让服务崩
            pass
        response.success = not failed
        response.message = "已全部卸力，可断电池" if not failed else f"部分失败：{', '.join(failed)}"
        self.get_logger().info(f"▸ 下电流程结束：{response.message}")
        return response

    def destroy_node(self) -> None:
        if self._closed:  # 防止重复调用
            return
        self._closed = True

        try:
            for sid in self._p_int_list(self.P_SERVO_IDS):  # 使舵机全部卸力
                try:
                    self._servo.torque_off(int(sid))
                except Exception as exc:
                    self.get_logger().error(f"退出卸力失败（舵机 {sid}）：{exc}")
        finally:
            try:
                self._servo.close()
            finally:
                super().destroy_node()


def main(args=None) -> None:
    """节点入口：初始化 rclpy、创建节点、自旋。"""
    rclpy.init(args=args)
    node: ServoDriverNode | None = None

    try:
        try:
            node = ServoDriverNode()
        except RuntimeError as exc:  # 串口打不开：给出反馈
            print(f"[servo_driver] 启动失败：{exc}", file=sys.stderr)
            rclpy.try_shutdown()
            raise SystemExit(2)
        atexit.register(node.close_serial_safely)  # ← 只关串口，不 destroy_node
        executor = MultiThreadedExecutor(
            num_threads=4
        )  # 多线程执行器：Action 的 execute 回调 + 取消请求才能并行处理
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("收到 Ctrl+C，退出")
        finally:
            try:
                executor.shutdown()
            except Exception as exc:  # noqa: BLE001
                node.get_logger().warn(f"executor 关闭异常：{exc}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:  # noqa: BLE001
                print(f"destroy_node 异常：{exc}", file=sys.stderr)

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
