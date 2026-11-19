#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pytest 单测：拼指令字符串 / 解析回包（加分项 +2）。

为什么这个文件不接硬件也能跑：
    被测的 `servo_protocol.py` 里没有一行 rclpy、没有一个串口对象，全是纯函数。
    所以 `colcon test` 不需要舵机、不需要控制板、不需要板卡。

跑法：
    cd ~/ros2_ws
    colcon test --packages-select cquick_servo
    colcon test-result --verbose

想单跑一个用例看细节：
    colcon test --packages-select cquick_servo --pytest-args -v
"""

import pytest

from cquick_servo.servo_protocol import (
    ProtocolError,
    build_move_command,
    build_pid_command,
    build_prad_command,
    build_torque_command,
    parse_command_shorthand,
    parse_prad_reply,
)


# --------------------------------------------------------------- 拼移动指令
def test_move_command_pads_id_to_three_digits():
    """细则加分项原文举例：#1,1700,1000 -> #001P1700T1000!"""
    assert build_move_command(1, 1700, 1000) == '#001P1700T1000!'


def test_move_command_keeps_three_digit_id():
    assert build_move_command(123, 1500, 800) == '#123P1500T0800!'


def test_pwm_and_duration_are_zero_padded_to_four_digits():
    """众灵手册的位宽规定：ID 3 位、P 4 位、T 4 位。

    这一条是最阴险的坑：细则的例子 #001P1500T1000! 天然就是 4 位，
    照抄时看不出问题；但 pwm=900、duration=800 不补零就会发成
    #001P900T800!，控制板可能不认，舵机纹丝不动。
    """
    assert build_move_command(1, 900, 800) == '#001P0900T0800!'
    assert build_move_command(1, 80, 5) == '#001P0080T0005!'


def test_duration_over_four_digits_raises():
    """T 只有 4 位，9999 是上限。"""
    with pytest.raises(ProtocolError):
        build_move_command(1, 1500, 10000)


def test_pwm_over_four_digits_raises():
    with pytest.raises(ProtocolError):
        build_move_command(1, 10000, 800)


def test_move_command_ends_with_bang():
    """结尾的 ! 是帧结束标志，丢了控制板就一直等后面的字。"""
    assert build_move_command(1, 1500, 500).endswith('!')


# ------------------------------------------------------------- 简写解析入口
def test_shorthand_with_hash_and_commas():
    assert parse_command_shorthand('#1,1700,1000') == (1, 1700, 1000)


def test_shorthand_with_spaces():
    assert parse_command_shorthand('1 1700 1000') == (1, 1700, 1000)


def test_shorthand_then_build_matches_direct_build():
    """两条路必须拼出同一个字符串 —— 这正是加分项要的那条断言。"""
    assert build_move_command(*parse_command_shorthand('#1,1700,1000')) \
        == build_move_command(1, 1700, 1000)


# ------------------------------------------------------------ 查询 / 扭力指令
def test_query_commands():
    assert build_pid_command(1) == '#001PID!'
    assert build_prad_command(12) == '#012PRAD!'


def test_torque_commands():
    assert build_torque_command(1, True) == '#001PULR!'
    assert build_torque_command(1, False) == '#001PULK!'


# ------------------------------------------------------------------ 回包解析
@pytest.mark.parametrize('raw, expected', [
    (b'#001P1500!', 1500),          # 最常见的回包形状
    (b'#001PRAD1500!', 1500),       # 带 PRAD 字样的
    (b'#001PRAD01500!', 1500),      # 前导零
    (b'1500', 1500),                # 板子只回数字
    (b'#001P900!', 900),
    (b'', None),                    # 超时，什么都没读到
    (b'\xff\xfe\x00', None),        # 乱码（十有八九是波特率不对）
])
def test_parse_prad_reply(raw, expected):
    assert parse_prad_reply(raw) == expected


# ------------------------------------------------------------------ 非法输入
def test_id_zero_raises():
    with pytest.raises(ProtocolError):
        build_move_command(0, 1500, 800)


def test_id_too_big_raises():
    with pytest.raises(ProtocolError):
        build_move_command(255, 1500, 800)


def test_zero_duration_raises():
    """duration=0 会让舵机瞬间猛跳撞结构，必须挡掉。"""
    with pytest.raises(ProtocolError):
        build_move_command(1, 1500, 0)


def test_pwm_over_uint16_raises():
    with pytest.raises(ProtocolError):
        build_move_command(1, 70000, 800)


def test_pwm_just_over_uint16_boundary_raises():
    with pytest.raises(ProtocolError):
        build_move_command(1, 65536, 800)


@pytest.mark.parametrize('bad', ['#1,1700', 'abc', '#1,1700,x', ''])
def test_bad_shorthand_raises(bad):
    with pytest.raises(ProtocolError):
        parse_command_shorthand(bad)
