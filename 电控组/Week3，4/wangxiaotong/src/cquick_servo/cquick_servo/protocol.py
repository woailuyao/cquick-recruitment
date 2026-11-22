"""众灵总线舵机 ASCII 协议：拼指令、解析回显。
本文件只放**协议本身**的东西 —— `#` `!`、ID 三位补零、`P`/`T`/`PID`/`PRAD`/`PULR`/`PULK`。
"""
# 真机实测（2026-11-19，24 路总线控制板 + CH340，115200 8N1）：
#   #001PID!          -> b'#001P!'        ← 本板 PID 回显不含 ID 数值
#   #001PRAD!         -> b'#001P1495!'    ← 位置回读，可用
#   #001P1700T1000!   -> b''              ← 动作指令不回显
#   #001PULR!         -> b''              ← 使能指令不回显
# 结论：ID 与位置只能从 PRAD 回显里取，PID 仅用于确认通路。




from __future__ import annotations

import re
from typing import Optional, Tuple

FRAME_START = '#'
FRAME_END = '!'

ID_MIN = 1
ID_MAX = 254

# 电气极限：舵机能接受的 PWM 范围，和结构安全范围不是一回事
PWM_ELEC_MIN = 500
PWM_ELEC_MAX = 2500

DURATION_MIN_MS = 1
DURATION_MAX_MS = 60_000

QUERY_CODES = ('PID', 'PRAD', 'PULR', 'PULK')

# 回显可能长成 "#001P1500!"、"#001P0001!"、"P1500!" 等，容错解析
# （真机到底是什么格式，到实验室核一次，只需要动这一个正则）
_REPLY_PWM_RE = re.compile(r'P\s*(\d{1,4})\s*!')


class ProtocolError(ValueError):
    """参数不合法时抛出，调用方可以据此判长度。"""


def _check_id(servo_id: int) -> int:
    servo_id = int(servo_id)
    if not ID_MIN <= servo_id <= ID_MAX:
        raise ProtocolError(f'舵机 ID {servo_id} 不在 {ID_MIN}~{ID_MAX}')
    return servo_id


def build_move(servo_id: int, pwm: int, duration_ms: int) -> str:
    """拼「转角度」指令：1 / 1700 / 1000 -> '#001P1700T1000!'"""
    servo_id = _check_id(servo_id)
    pwm = int(pwm)
    duration_ms = int(duration_ms)
    if not PWM_ELEC_MIN <= pwm <= PWM_ELEC_MAX:
        raise ProtocolError(f'pwm {pwm} 超出电气范围 {PWM_ELEC_MIN}~{PWM_ELEC_MAX}')
    if not DURATION_MIN_MS <= duration_ms <= DURATION_MAX_MS:
        raise ProtocolError(
            f'duration_ms {duration_ms} 不在 {DURATION_MIN_MS}~{DURATION_MAX_MS}')
    return f'{FRAME_START}{servo_id:03d}P{pwm}T{duration_ms}{FRAME_END}'


def build_query(servo_id: int, code: str) -> str:
    """拼只读指令：'#001PID!' / '#001PRAD!' / '#001PULR!' / '#001PULK!'"""
    servo_id = _check_id(servo_id)
    code = str(code).upper()
    if code not in QUERY_CODES:
        raise ProtocolError(f'不认识的指令字 {code!r}，只支持 {QUERY_CODES}')
    return f'{FRAME_START}{servo_id:03d}{code}{FRAME_END}'


def build_torque_on(servo_id: int) -> str:
    """恢复扭力。不发这一条，后面的转角度指令舵机纹丝不动。"""
    return build_query(servo_id, 'PULR')


def build_unload(servo_id: int) -> str:
    """卸力。下电前必发，防堵转发热。"""
    return build_query(servo_id, 'PULK')


def parse_pwm(frame: Optional[str]) -> Optional[int]:
    """从回显里抠出 PWM / ID 数值；解析不出来返回 None（调用方按「没回」处理）。"""
    if not frame:
        return None
    match = _REPLY_PWM_RE.search(frame)
    return int(match.group(1)) if match else None


def parse_triplet(text: str) -> Tuple[int, int, int]:
    """'#1,1700,1000' -> (1, 1700, 1000)。抄纸面参数、命令行临时试的时候用。"""
    body = str(text).strip().strip(FRAME_START).rstrip(FRAME_END)
    parts = [p for p in re.split(r'[,\s]+', body) if p]
    if len(parts) != 3:
        raise ProtocolError(f'要三段 id,pwm,duration，收到 {text!r}')
    try:
        return int(parts[0]), int(parts[1]), int(parts[2])
    except ValueError as exc:
        raise ProtocolError(f'{text!r} 里有非数字: {exc}') from exc
