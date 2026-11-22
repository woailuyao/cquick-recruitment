#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口链路：开、关、写、问、掉线自动重连。

为什么单独一层：驱动节点要「拔出 USB 不崩、插回去自动恢复」（加分项），
把重连逻辑塞在 execute_callback 里会让那条链路又长又难读。
这一层只负责字节，不懂 PWM、不懂 Action。

线程模型：
  - 1 个后台读线程：收字节 -> 切帧 -> 丢进回帧队列并 notify
  - 写：所有线程共用一把写锁，一帧 write + flush 原子完成
  - 问（query）：持「对话锁」发帧，然后 wait 回帧，
    保证两条线程同时问（不同 ID）时不会互相吃错回帧
"""

from __future__ import annotations

import collections
import threading
import time
from typing import Iterable, Optional, Tuple

from cquick_servo.servo_protocol import ServoReply, parse_frames, parse_reply

try:  # pyserial 缺失时不让 import 直接炸，把话说清楚
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - 只在没装库的机器上走
    serial = None

    class SerialException(Exception):
        """pyserial 没装时的占位异常。"""


class SerialLinkError(RuntimeError):
    """串口不可用（没打开 / 写失败 / 已掉线）。"""


class SerialLink:
    """一条到 24 路舵机控制板的串口链路。"""

    def __init__(self, port: str, baudrate: int,
                 read_timeout: float = 0.05, write_timeout: float = 0.2) -> None:
        self._port = port
        self._baudrate = baudrate
        self._read_timeout = read_timeout
        self._write_timeout = write_timeout

        self._ser = None
        self._write_lock = threading.Lock()
        self._dialog_lock = threading.Lock()
        self._reply_cond = threading.Condition()
        self._replies: 'collections.deque[ServoReply]' = collections.deque(maxlen=64)
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.last_error = ''

    # ------------------------------------------------------------ 属性
    @property
    def port(self) -> str:
        return self._port

    @property
    def baudrate(self) -> int:
        return self._baudrate

    @property
    def is_open(self) -> bool:
        ser = self._ser
        return ser is not None and getattr(ser, 'is_open', False)

    # ------------------------------------------------------------ 开 / 关
    def open(self) -> None:
        """打开串口并起读线程。失败抛 SerialLinkError（调用方决定是否重试）。"""
        with self._write_lock:
            if self.is_open:
                return
            if serial is None:
                raise SerialLinkError('pyserial 未安装：sudo apt install python3-serial')
            try:
                self._ser = serial.Serial(
                    port=self._port,
                    baudrate=self._baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self._read_timeout,
                    write_timeout=self._write_timeout,
                )
            except Exception as exc:  # noqa: BLE001 - 底层异常类型五花八门
                self._ser = None
                self.last_error = f'打开 {self._port} 失败：{exc}'
                raise SerialLinkError(self.last_error) from exc
            self._stop.clear()
        self._reader = threading.Thread(
            target=self._reader_loop, name='servo-serial-reader', daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._stop.set()
        reader = self._reader
        if reader is not None and reader.is_alive() and reader is not threading.current_thread():
            reader.join(timeout=0.5)
        self._reader = None
        with self._write_lock:
            self._close_serial_locked()

    def _close_serial_locked(self) -> None:
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def _drop(self, reason: str) -> None:
        """标记掉线：关掉句柄，让上层看到 is_open == False。"""
        self.last_error = reason
        with self._write_lock:
            self._close_serial_locked()

    # ------------------------------------------------------------ 写
    def send(self, frame: bytes) -> None:
        """写一帧并 flush。失败抛 SerialLinkError，同时把链路标记为掉线。"""
        with self._write_lock:
            ser = self._ser
            if ser is None or not getattr(ser, 'is_open', False):
                raise SerialLinkError(self.last_error or '串口未打开')
            try:
                ser.write(frame)
                ser.flush()  # 不 flush 会卡在缓冲区里，表现为「舵机不动」
            except Exception as exc:  # noqa: BLE001
                self.last_error = f'写串口失败：{exc}'
                self._close_serial_locked()
                raise SerialLinkError(self.last_error) from exc

    # ------------------------------------------------------------ 问
    def query(self, frame: bytes, servo_id: int,
              accept_commands: Iterable[str] = (),
              timeout: float = 0.15) -> Optional[ServoReply]:
        """发一帧并等这个 ID 的回帧。超时返回 None。

        accept_commands 为空表示只看 ID；给了就要求回帧的指令字在集合里
        （不同固件对 PRAD 的回显有的写 PRAD、有的只写 P，所以给个集合）。
        """
        accept = {cmd.upper() for cmd in accept_commands}
        with self._dialog_lock:
            with self._reply_cond:
                self._replies.clear()
            self.send(frame)
            deadline = time.monotonic() + timeout
            with self._reply_cond:
                while True:
                    for reply in list(self._replies):
                        if reply.servo_id != servo_id:
                            continue
                        if accept and reply.command not in accept:
                            continue
                        self._replies.clear()
                        return reply
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    self._reply_cond.wait(remaining)

    # ------------------------------------------------------------ 读线程
    def _reader_loop(self) -> None:
        buffer = b''
        while not self._stop.is_set():
            ser = self._ser
            if ser is None:
                time.sleep(0.05)
                continue
            try:
                chunk = ser.read(64)
            except Exception as exc:  # noqa: BLE001
                self._drop(f'读串口失败：{exc}')
                continue
            if not chunk:
                continue
            frames, buffer = parse_frames(buffer + chunk)
            if not frames:
                continue
            with self._reply_cond:
                for frame in frames:
                    reply = parse_reply(frame)
                    if reply is not None:
                        self._replies.append(reply)
                self._reply_cond.notify_all()

    # ------------------------------------------------------------ 便捷
    def send_move(self, servo_id: int, pwm: int, duration_ms: int) -> None:
        from cquick_servo.servo_protocol import build_move_command
        self.send(build_move_command(servo_id, pwm, duration_ms))

    def read_position(self, servo_id: int, timeout: float = 0.15) -> Optional[int]:
        """读一次 PRAD 位置值。"""
        from cquick_servo import servo_protocol as proto
        reply = self.query(proto.build_read_position(servo_id), servo_id,
                           (proto.CMD_READ_POS, proto.CMD_MOVE), timeout)
        return None if reply is None else reply.value

    def __repr__(self) -> str:  # pragma: no cover
        state = 'open' if self.is_open else 'closed'
        return f'<SerialLink {self._port}@{self._baudrate} {state}>'


def describe(exc: BaseException) -> Tuple[str, str]:
    """把异常拆成 (类型名, 文本)，日志里用。"""
    return type(exc).__name__, str(exc)
