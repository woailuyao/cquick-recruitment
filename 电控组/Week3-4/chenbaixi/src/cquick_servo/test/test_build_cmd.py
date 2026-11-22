"""不接硬件就能跑的单元测试：验证 ASCII 指令拼装"""

import pytest

from cquick_servo.servo_driver import build_move_cmd, build_simple_cmd, parse_pwm


def test_move_cmd_basic():
    """细则里的例子：id=1 pwm=1700 dur=1000"""
    assert build_move_cmd(1, 1700, 1000) == b'#001P1700T1000!'


def test_id_zero_padded_3():
    """ID 固定补零到 3 位"""
    assert build_move_cmd(7, 1500, 800) == b'#007P1500T0800!'


def test_pwm_and_duration_zero_padded_4():
    """PWM 和时间固定 4 位"""
    assert build_move_cmd(1, 900, 500) == b'#001P0900T0500!'


def test_simple_commands():
    """无参数指令"""
    assert build_simple_cmd(3, 'PULK') == b'#003PULK!'
    assert build_simple_cmd(1, 'PRAD') == b'#001PRAD!'
    assert build_simple_cmd(1, 'PID') == b'#001PID!'
    assert build_simple_cmd(1, 'PULR') == b'#001PULR!'


def test_parse_pwm_ascii():
    """ASCII 回包里取出 4 位位置"""
    assert parse_pwm(b'#001P1500!') == 1500
    assert parse_pwm(b'#001PRAD=1620!') == 1620


def test_parse_pwm_empty_or_binary():
    """空包或二进制包返回 None，不抛异常"""
    assert parse_pwm(b'') is None
    assert parse_pwm(b'\x55\x55\x01\x00') is None
