#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""总线舵机 ASCII 协议：只做字符串拼装与解析，不 import rclpy / pyserial。

单独拆出来的原因是加分项要求「不接硬件能跑 pytest」：驱动节点本身依赖 ROS 2，
把这个模块独立出来后，测试只需要 Python 标准库：

    cd src/cquick_servo && pytest test/ -v
"""
import re

# ---------- 协议常量：协议本身，不是可配置项 ----------
HEAD = '#'
TAIL = '!'
ID_WIDTH = 3
CMD_ID = 'PID'
CMD_POS = 'PRAD'
CMD_TORQUE_ON = 'PULR'
CMD_TORQUE_OFF = 'PULK'
CMD_WRITE_POS = 'P'
CMD_TIME = 'T'

REPLY_TIMEOUT_S = 0.25
FEEDBACK_HZ = 10.0
STATE_HZ = 2.0

# ---------- 电气极限与可选配置 ----------
ELECTRIC_MIN = 500
ELECTRIC_MAX = 2500
BAUD_CHOICES = (9600, 19200, 38400, 57600, 115200, 230400)
MAX_DURATION_MS = 30000
RECONNECT_INTERVAL_S = 2.0


def build_move(sid, pwm, duration_ms):
    """#1,1700,1000 -> '#001P1700T1000!'"""
    return '%s%0*d%s%d%s%d%s' % (HEAD, ID_WIDTH, int(sid), CMD_WRITE_POS,
                                  int(pwm), CMD_TIME, int(duration_ms), TAIL)


def build_query(sid, func):
    """查询类指令，例如 build_query(0, CMD_POS) -> '#000PRAD!'"""
    return '%s%0*d%s%s' % (HEAD, ID_WIDTH, int(sid), func, TAIL)


def parse_pwm(reply):
    """从 '#000P1493!' 里取 1493。

    '#000P!' 是读 ID 的回包，里面没有位置数字，必须返回 None 而不是 0；
    有些控制板回包不带 P 前缀，这时退而求其次找独立的 3~4 位数字。
    """
    text = reply or ''
    found = re.findall(r'%s(\d{3,4})' % CMD_WRITE_POS, text)
    if found:
        return int(found[-1])
    # 先去掉 #000 这样的 ID 前缀，避免把 ID 当成位置
    stripped = re.sub(r'%s\d{%d}' % (re.escape(HEAD), ID_WIDTH), '', text)
    found = re.findall(r'(?<!\d)(\d{3,4})(?!\d)', stripped)
    return int(found[-1]) if found else None


def validate_limits(lo, hi, neutral):
    """限位组合校验：单个值合法，不代表组合起来合法。返回 (ok, reason)。"""
    if not (ELECTRIC_MIN <= lo <= ELECTRIC_MAX):
        return False, '%d out of electrical range %d~%d' % (lo, ELECTRIC_MIN, ELECTRIC_MAX)
    if not (ELECTRIC_MIN <= hi <= ELECTRIC_MAX):
        return False, '%d out of electrical range %d~%d' % (hi, ELECTRIC_MIN, ELECTRIC_MAX)
    if lo >= hi:
        return False, 'pwm_min(%d) must be less than pwm_max(%d)' % (lo, hi)
    if not (lo <= neutral <= hi):
        return False, 'neutral_pwm(%d) must be inside [%d, %d]' % (neutral, lo, hi)
    return True, 'ok'
