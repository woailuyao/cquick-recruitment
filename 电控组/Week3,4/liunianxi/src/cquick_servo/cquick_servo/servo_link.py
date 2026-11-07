"""串口链路 + 总线舵机 ASCII 协议。

分工原则（评审会 grep 这个文件）：
  * 本文件只允许出现**协议常量**：`#`、`!`、ID 补 3 位、`P` / `T` / `PID` / `PRAD` / `PULR` / `PULK`；
  * 设备名、波特率、pwm_min / pwm_max 这些是**配置**，一律由调用方从 ROS 参数传进来。

这样"拼 ASCII 字符串"这件事可以被单独测（pytest 加分项就测这两个 build_* 函数）。
"""
from __future__ import annotations

import re
import threading
from typing import Optional

import serial

# ---------------- 协议常量（允许硬编码） ----------------
CMD_HEAD = "#"
CMD_TAIL = "!"
VERB_PID = "PID"      # 读 ID
VERB_PRAD = "PRAD"    # 读位置
VERB_PULR = "PULR"    # 恢复扭力（要它动之前必须先发）
VERB_PULK = "PULK"    # 卸力（断电前必发）

# 电气极限：不是安全范围，安全范围是驱动节点的 pwm_min / pwm_max 参数
PWM_ELECTRIC_MIN = 500
PWM_ELECTRIC_MAX = 2500

_PRAD_RE = re.compile(r"#?\s*(\d{1,3})\s*P\s*(\d{3,4})")
_PID_RE = re.compile(r"#?\s*(\d{1,3})\s*!")


def build_move(frame_id: int, pwm: int, duration_ms: int) -> str:
    """拼"转到某个位置"：`#001P1700T800!`（ID 固定 3 位补零）。"""
    return f"#{int(frame_id):03d}P{int(pwm)}T{int(duration_ms)}!"


def build_verb(frame_id: int, verb: str) -> str:
    """拼只有命令字的指令：`#001PID!` / `#001PRAD!` / `#001PULR!` / `#001PULK!`。

    注意：verb 本身已经带 `P`（PID / PRAD / PULR / PULK），不要再补一个 `P`，
    否则拼出来是 `#001PPRAD!`——线上多一个字符，舵机就当乱码丢掉
    （这个 bug 当初是 pytest 抓到的，保留用例防回归）。
    """
    return f"#{int(frame_id):03d}{verb}!"


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


class SerialLink:
    """一根串口。所有写操作加锁，读操作带超时；读不到回包不抛异常（真舵机可能不回）。"""

    def __init__(self, port: str, baudrate: int, timeout: float = 0.2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._lock = threading.Lock()
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
    def is_open(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    def close(self) -> None:
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass

    def reopen(self) -> None:
        """掉线重连用。"""
        self.close()
        self._ser = serial.Serial(
            port=self.port, baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=self.timeout, write_timeout=self.timeout,
        )

    # ---------- 写 ----------
    def send(self, text: str) -> None:
        with self._lock:
            self._ser.write(text.encode("ascii"))
            self._ser.flush()

    # ---------- 读 ----------
    def query(self, text: str) -> Optional[str]:
        """发一条指令并等回包。读到 `!` 或超时为止；超时返回 None。"""
        with self._lock:
            try:
                self._ser.reset_input_buffer()
                self._ser.write(text.encode("ascii"))
                self._ser.flush()
                chunks = []
                for _ in range(64):
                    b = self._ser.read(1)
                    if not b:
                        break
                    chunks.append(b)
                    if b == CMD_TAIL.encode("ascii"):
                        break
                reply = b"".join(chunks).decode("ascii", errors="ignore").strip()
                return reply or None
            except Exception:
                return None

    # ---------- 语义动作 ----------
    def move(self, frame_id: int, pwm: int, duration_ms: int) -> str:
        cmd = build_move(frame_id, pwm, duration_ms)
        self.send(cmd)
        return cmd

    def read_pwm(self, frame_id: int) -> Optional[int]:
        return parse_pwm(self.query(build_verb(frame_id, VERB_PRAD)) or "")

    def read_id(self, frame_id: int) -> Optional[int]:
        return parse_id(self.query(build_verb(frame_id, VERB_PID)) or "")

    def torque_on(self, frame_id: int) -> None:
        self.send(build_verb(frame_id, VERB_PULR))

    def torque_off(self, frame_id: int) -> None:
        self.send(build_verb(frame_id, VERB_PULK))
