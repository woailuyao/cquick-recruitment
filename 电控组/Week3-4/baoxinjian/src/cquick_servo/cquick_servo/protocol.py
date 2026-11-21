"""protocol.py —— 总线舵机 ASCII 协议（纯函数，不依赖 ROS、不依赖串口）

单独拆出来的原因：这些函数可以直接被 pytest 覆盖，不需要接硬件。
"""

import re

# 协议常量（协议本身，不算硬编码配置）
CENTER_PWM = 1500
PWM_HARD_MIN = 500
PWM_HARD_MAX = 2500

REPLY_RE = re.compile(rb'#(\d{3})P?(\d+)\s*!')


def build_move_command(servo_id: int, pwm: int, duration_ms: int) -> str:
    """拼位置指令：build_move_command(1, 1700, 1000) -> '#001P1700T1000!'"""
    return f'#{int(servo_id):03d}P{int(pwm)}T{int(duration_ms)}!'


def build_keyword_command(servo_id: int, keyword: str) -> str:
    """拼关键字指令：build_keyword_command(1, 'PULR') -> '#001PULR!'"""
    return f'#{int(servo_id):03d}{keyword}!'


def parse_reply(raw: bytes):
    """从回显里取出 (id, value)；解析不到返回 None。

    '#001P1500!' -> (1, 1500)
    '#001P1!'    -> (1, 1)
    """
    if not raw:
        return None
    match = REPLY_RE.search(raw)
    if not match:
        return None
    return int(match.group(1)), int(match.group(2))
