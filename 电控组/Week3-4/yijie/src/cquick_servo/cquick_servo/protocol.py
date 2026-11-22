# -*- coding: utf-8 -*-
"""protocol.py —— 总线舵机的 ASCII 协议：拼帧与解析（纯函数，不依赖 ROS / 硬件）。

协议形状：
    #001P1700T1000!
    │└┬┘│└──┬─┘│└──┬─┘│
    │  │ │   │  │   │  └─ 帧结束
    │  │ │   │  │   └──── 时长（ms）
    │  │ │   │  └──────── 时间指令字
    │  │ │   └─────────── 目标 PWM
    │  │ └─────────────── 位置指令字
    │  └───────────────── 舵机 ID，固定 3 位补零
    └──────────────────── 帧起始

这里只放「协议常量」，不放任何配置（端口 / 波特率 / 限位都必须是 ROS 参数）。
"""

from typing import Optional

ID_WIDTH = 3                 # 协议规定 ID 固定 3 位
CENTER_PWM = 1500            # 中立位是协议约定，不是可配置项
PWM_MIN_ELECTRIC = 500       # 电气极限（不是安全范围！）
PWM_MAX_ELECTRIC = 2500

_ID_MIN = 0                  # 合法 ID 下界：有的 24 路控制板从 000 开始编址
_ID_MAX = 254


def format_id(servo_id: int) -> str:
    """1 -> '001'，12 -> '012'，123 -> '123'。"""
    return f'{int(servo_id):0{ID_WIDTH}d}'


def is_valid_id(servo_id: int) -> bool:
    return _ID_MIN <= int(servo_id) <= _ID_MAX


def build_move(servo_id: int, pwm: int, duration_ms: int) -> str:
    """位置指令：舵机 servo_id 在 duration_ms 内走到 pwm。"""
    return f'#{format_id(servo_id)}P{int(pwm)}T{int(duration_ms)}!'


def build_pid(servo_id: int) -> str:
    return f'#{format_id(servo_id)}PID!'


def build_prad(servo_id: int) -> str:
    return f'#{format_id(servo_id)}PRAD!'


def build_pulr(servo_id: int) -> str:
    """恢复扭力：上电默认卸力，不发这条位置指令不会动。"""
    return f'#{format_id(servo_id)}PULR!'


def build_pulk(servo_id: int) -> str:
    """卸力：断电前 / 堵转时 / 看门狗触发时发。"""
    return f'#{format_id(servo_id)}PULK!'


def clamp_pwm(pwm: int, pwm_min: int, pwm_max: int) -> int:
    """把值夹到 [pwm_min, pwm_max]（注意 min/max 必须已经校验过大小关系）。"""
    return max(int(pwm_min), min(int(pwm_max), int(pwm)))


def is_within(pwm: int, pwm_min: int, pwm_max: int) -> bool:
    return int(pwm_min) <= int(pwm) <= int(pwm_max)


def parse_pwm_reply(reply: bytes) -> Optional[int]:
    """从 PRAD 回显里抠出 PWM。

    不同批次控制板回显可能是 b'#001P1500!' / b'001P1500' / b'P1500!'，
    这里统一抓 「P + 1~4 位数字」。拿到你的真实回显后，用 tools 里的 raw 命令核对一次。
    """
    if not reply:
        return None
    import re
    match = re.search(rb'P(\d{1,4})', reply)
    return int(match.group(1)) if match else None


def parse_id_reply(reply: bytes) -> Optional[str]:
    """从 PID 回显里取出 3 位 ID 字符串；只用于任务 1 的扫 ID 工具。"""
    if not reply:
        return None
    import re
    match = re.search(rb'#?(\d{3})', reply)
    return match.group(1).decode('ascii') if match else None
