#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CQuick 总线舵机 —— ASCII 协议层。

这一层只放「协议本身」，全是纯函数：
  - 不 import rclpy，不 import pyserial
  - 不读参数、不碰串口
所以 test/test_protocol.py 不接硬件、不启节点就能跑。

协议形态（帧头 '#'，帧尾 '!'，ID 固定 3 位补零）：

    #001P1700T1000!   1 号舵机，1000 ms 内从当前位置走到 PWM 1700
    #001PULR!         恢复扭力（不发这条，舵机纹丝不动）
    #001PULK!         卸力
    #001PRAD!         读回当前 PWM 位置
    #001PID!          读 ID

设备名、波特率、限位区间这些属于**配置**，一律由驱动节点的参数提供，
绝不允许出现在本模块里（评审会 grep）。
"""

from __future__ import annotations

import re
from typing import List, NamedTuple, Optional, Tuple

# ---------------------------------------------------------------- 协议常量
# 下面这些是「协议本身」，不是配置，允许硬编码。
FRAME_HEAD = '#'
FRAME_TAIL = '!'
ID_DIGITS = 3          # ID 固定 3 位补零：1 -> '001'
ID_MIN = 1
ID_MAX = 254

CMD_MOVE = 'P'         # 转角度
CMD_READ_POS = 'PRAD'  # 读当前位置
CMD_READ_ID = 'PID'    # 读 ID
CMD_TORQUE_ON = 'PULR'  # 恢复扭力
CMD_TORQUE_OFF = 'PULK'  # 卸力
FIELD_DURATION = 'T'   # 用时字段

_REPLY_PATTERN = re.compile(r'^#(\d{1,3})([A-Z]+)(\d*)$')


class ServoReply(NamedTuple):
    """解析出来的一个回帧。value 为 None 表示回帧没带数字。"""

    servo_id: int
    command: str
    value: Optional[int]
    raw: str


def format_servo_id(servo_id: int) -> str:
    """ID 补零成 3 位。1 -> '001'，12 -> '012'。"""
    servo_id = int(servo_id)
    if not ID_MIN <= servo_id <= ID_MAX:
        raise ValueError(f'舵机 ID 必须是 {ID_MIN}~{ID_MAX}，收到 {servo_id}')
    return f'{servo_id:0{ID_DIGITS}d}'


def build_move_command(servo_id: int, pwm: int, duration_ms: int) -> bytes:
    """拼「转角度」指令：build_move_command(1, 1700, 1000) -> b'#001P1700T1000!'"""
    pwm = int(pwm)
    duration_ms = int(duration_ms)
    if pwm < 0:
        raise ValueError(f'PWM 不能为负：{pwm}')
    if duration_ms < 0:
        raise ValueError(f'duration_ms 不能为负：{duration_ms}')
    text = (
        f'{FRAME_HEAD}{format_servo_id(servo_id)}'
        f'{CMD_MOVE}{pwm}{FIELD_DURATION}{duration_ms}{FRAME_TAIL}'
    )
    return text.encode('ascii')


def _build_simple_command(servo_id: int, command: str) -> bytes:
    """拼不带参数的单字段指令：PULR / PULK / PRAD / PID。"""
    text = f'{FRAME_HEAD}{format_servo_id(servo_id)}{command}{FRAME_TAIL}'
    return text.encode('ascii')


def build_torque_on(servo_id: int) -> bytes:
    """#001PULR! 恢复扭力。要它动之前必须先发。"""
    return _build_simple_command(servo_id, CMD_TORQUE_ON)


def build_torque_off(servo_id: int) -> bytes:
    """#001PULK! 卸力。下电前必发。"""
    return _build_simple_command(servo_id, CMD_TORQUE_OFF)


def build_read_position(servo_id: int) -> bytes:
    """#001PRAD! 读回当前 PWM。"""
    return _build_simple_command(servo_id, CMD_READ_POS)


def build_read_id(servo_id: int) -> bytes:
    """#001PID! 读 ID，用来扫在线的舵机。"""
    return _build_simple_command(servo_id, CMD_READ_ID)


def parse_frames(buffer: bytes) -> Tuple[List[str], bytes]:
    """从接收缓冲里切出完整帧。

    返回 (帧字符串列表, 还没收完的残余缓冲)。
    串口是流式的，一次 read 可能只有半帧，也可能一次来三帧，
    所以残余必须由调用方带着走，不能丢。
    """
    frames: List[str] = []
    data = buffer
    while True:
        start = data.find(FRAME_HEAD.encode('ascii'))
        if start < 0:
            return frames, b''
        end = data.find(FRAME_TAIL.encode('ascii'), start + 1)
        if end < 0:
            # 半帧，留着等下一批字节
            return frames, data[start:]
        chunk = data[start:end + 1]
        frames.append(chunk.decode('ascii', errors='ignore'))
        data = data[end + 1:]


def parse_reply(frame: str) -> Optional[ServoReply]:
    """解析回帧。

    '#001P1500!'   -> ServoReply(1, 'P', 1500)
    '#001PRAD1500!'-> ServoReply(1, 'PRAD', 1500)
    '#001PID!'     -> ServoReply(1, 'PID', None)
    '#001P1500T1000!'（这是自己发出去的指令被回显）-> None
    """
    text = frame.strip()
    raw = text
    if text.endswith(FRAME_TAIL):
        text = text[:-len(FRAME_TAIL)]
    match = _REPLY_PATTERN.match(text)
    if match is None:
        return None
    servo_id = int(match.group(1))
    command = match.group(2)
    digits = match.group(3)
    value = int(digits) if digits else None
    return ServoReply(servo_id, command, value, raw)
