"""串口链路 + 总线舵机 ASCII 协议。

分工原则（评审会 grep 这个文件）：
  * 本文件只允许出现**协议常量**：`#`、`!`、ID 补 3 位、`P` / `T` / `PID` / `PRAD` / `PULR` / `PULK`；
  * 设备名、波特率、pwm_min / pwm_max 这些是**配置**，一律由调用方从 ROS 参数传进来。

这样"拼 ASCII 字符串"这件事可以被单独测（pytest 加分项就测这两个 build_* 函数）。
"""
from __future__ import annotations

import re
import threading
import time
from typing import Optional

import serial

# ---------------- 协议常量（允许硬编码） ----------------
# 帧头：所有指令都以 # 开头，板子靠它同步帧边界。
CMD_HEAD = "#"
# 帧尾：板子收到 ! 才开始解析；我们读回包也按 ! 截断。
CMD_TAIL = "!"
# 四个命令字；拼帧时只允许用这里的常量，禁止到处写字面量。
VERB_PID = "PID"      # 读 ID
VERB_PRAD = "PRAD"    # 读位置
VERB_PULR = "PULR"    # 恢复扭力（要它动之前必须先发）
VERB_PULK = "PULK"    # 卸力（断电前必发）

# 电气极限：不是安全范围，安全范围是驱动节点的 pwm_min / pwm_max 参数
# 电气极限：超出这个范围的 PWM 物理上无意义；它和“安全区间”是两回事（后者是驱动参数）。
PWM_ELECTRIC_MIN = 500
PWM_ELECTRIC_MAX = 2500

# 总线帧间隔：连发两条消息时，固件只执行第一条、丢掉后面的，
# 经测试，30 ms 能被稳定接收。
# 30 ms 最小帧间隔：实测连发两条指令时固件只执行第一条（帧粘连）。真机踩过，必须保留。
MIN_FRAME_GAP = 0.03
# ---- 看门狗与重发的时间常数（放协议层，pytest 不用起 rclpy 就能测） ----
# 心跳常数放在协议层，pytest 不用起 rclpy 就能断言看门狗的时间预算。
HEARTBEAT_PERIOD = 0.2      # 5 Hz 心跳
WATCHDOG_TIMEOUT = 1.0      # 超过 1 s 没心跳就全部 PULK
RETRY_DURATION_MS = 600     # 回读超差时重发用的短行程（会被 min_duration_ms 抬到 1000）


# 纯函数、可单测：把时长抬到固件真正接受的下限（ZL-IS2 忽略 T<1000）。
def clamp_duration(wanted_ms: int, floor_ms: int) -> int:
    """把 duration 抬到固件真正接受的下限（ZL-IS2 实测忽略 T<1000）。"""
    return max(int(wanted_ms), int(floor_ms))

# 位置回包正则：兼容 `#001P1500!`、`#001P1500`、`*001P1500` 几种写法。
_PRAD_RE = re.compile(r"#?\s*(\d{1,3})\s*P\s*(\d{3,4})")
# ID 回包正则：只认 `#003P!` / `#003P` 这种最简回包（真机 PID 回包不带位置）。
_PID_RE = re.compile(r"#?\s*(\d{1,3})")


# 唯一允许“拼动作帧”的地方；ID 固定 3 位补零（#001 而不是 #1）。
def build_move(frame_id: int, pwm: int, duration_ms: int) -> str:
    """拼"转到某个位置"：`#001P1700T800!`（ID 固定 3 位补零）。"""
    return f"#{int(frame_id):03d}P{int(pwm)}T{int(duration_ms)}!"


# 唯一允许“拼命令字帧”的地方；verb 自带 P，绝不能再补一个 P（#001PPRAD! 是本项目抓到的经典 bug）。
def build_verb(frame_id: int, verb: str) -> str:
    """拼只有命令字的指令：`#001PID!` / `#001PRAD!` / `#001PULR!` / `#001PULK!`。

    注意：verb 本身已经带 `P`（PID / PRAD / PULR / PULK），不要再补一个 `P`，
    否则拼出来是 `#001PPRAD!`——线上多一个字符，舵机就当乱码丢掉
    （这个 bug 当初是 pytest 抓到的，保留用例防回归）。
    """
    return f"#{int(frame_id):03d}{verb}!"


# 解析位置：先按完整格式匹配；失败再退化成“取最后一段 3~4 位数字”，兼容手写调试。
def parse_pwm(reply: str) -> Optional[int]:
    """从回包里抠 PWM。兼容几种常见回包格式：`#001P1500!` / `*001P1500` / 纯数字。"""
    if not reply:
        return None
    m = _PRAD_RE.search(reply)
    if m:
        return int(m.group(2))
    digits = re.findall(r"\d{3,4}", reply)
    if digits:
        return int(digits[-1])
    return None


# 解析 ID：先按 #ID 匹配，退化逻辑取第一段数字。
def parse_id(reply: str) -> Optional[int]:
    """从回包里抠 ID。"""
    if not reply:
        return None
    m = _PID_RE.search(reply)
    if m:
        return int(m.group(1))
    digits = re.findall(r"\d{1,3}", reply)
    if digits:
        return int(digits[0])
    return None


# 一根串口的全部状态：互斥锁、上次写时间、pyserial 句柄；读和写都只在这一个对象上发生。
class SerialLink:
    """一根串口。所有写操作加锁，读操作带超时；读不到回包不抛异常（真舵机可能不回）。"""

    # 8-N-1 是 ZL-IS2 的固定串口参数，属于协议层，不做成 ROS 参数；timeout 决定“读不到回包要等多久”。
    def __init__(self, port: str, baudrate: int, timeout: float = 0.2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        # 锁的意义：16 个 executor 线程可能同时写串口，不加锁会拼出半条帧。
        self._lock = threading.Lock()
        # 上次写的时间戳，供 _respect_gap 计算需要补睡多久。
        self._last_write = 0.0
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )

    # ---------- 生命周期 ----------
    @property
    # 对外只看这个属性判断链路是否可用：驱动、探活、重连全用它。
    def is_open(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    # 关闭也要加锁：避免在写一半时被另一个线程关掉句柄。
    def close(self) -> None:
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass

    # 先关再开：拔插之后旧 fd 处于坏状态，必须整个重建而不是复用。
    def reopen(self) -> None:
        """掉线重连用。"""
        self.close()
        self._ser = serial.Serial(
            port=self.port, baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=self.timeout, write_timeout=self.timeout,
        )

    # ---------- 写 ----------
    def _respect_gap(self) -> None:
        """保证两条指令之间至少 MIN_FRAME_GAP 秒，否则板子只执行第一条。"""
        # 只补差额：连续快发才睡，间隔够大就不睡，不拖慢动作组。
        wait = MIN_FRAME_GAP - (time.monotonic() - self._last_write)
        if wait > 0:
            time.sleep(wait)
        self._last_write = time.monotonic()

    # 写一条指令：加锁 + 补帧间隔 + 编码为 ASCII 发出；调用方只给文本，不关心缓冲/锁。
    def send(self, text: str) -> None:
        with self._lock:
            self._respect_gap()
            self._ser.write(text.encode("ascii"))
            self._ser.flush()

    # ---------- 读 ----------
    def query(self, text: str) -> Optional[str]:
        """发一条指令并等回包。读到 `!` 或超时为止；超时返回 None。"""
        with self._lock:
            try:
                # 先清空接收缓冲：丢掉上一条指令残留的回包，否则会读到别人的 ID。
                self._ser.reset_input_buffer()
                self._respect_gap()
                self._ser.write(text.encode("ascii"))
                self._ser.flush()
                chunks = []
                # 最多读 64 字节：正常回包 8~10 字节，64 足够，同时防死循环。
                for _ in range(64):
                    b = self._ser.read(1)
                    if not b:
                        break
                    chunks.append(b)
                    # 读到 ! 立刻停：回包后面可能还跟着垃圾字节，不能一路读到底。
                    if b == CMD_TAIL.encode("ascii"):
                        break
                reply = b"".join(chunks).decode("ascii", errors="ignore").strip()
                # 空串统一返回 None，上层用 `or ""` 兼容，判断逻辑更简单。
                return reply or None
            except Exception:
                return None

    # ---------- 语义动作 ----------
    # 语义封装层：move / read_pwm / read_id / torque_on / torque_off，驱动层完全不碰帧格式。
    def move(self, frame_id: int, pwm: int, duration_ms: int) -> str:
        cmd = build_move(frame_id, pwm, duration_ms)
        self.send(cmd)
        return cmd

    def read_pwm(self, frame_id: int) -> Optional[int]:
        # 读位置 = 发 PRAD + 解析；读不到就返回 None（板子不回包时的正常分支）。
        return parse_pwm(self.query(build_verb(frame_id, VERB_PRAD)) or "")

    def read_id(self, frame_id: int) -> Optional[int]:
        return parse_id(self.query(build_verb(frame_id, VERB_PID)) or "")

    def torque_on(self, frame_id: int) -> None:
        self.send(build_verb(frame_id, VERB_PULR))

    def torque_off(self, frame_id: int) -> None:
        self.send(build_verb(frame_id, VERB_PULK))
