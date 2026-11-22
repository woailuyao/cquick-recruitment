#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""协议拼装 / 解析 / 限位组合的单元测试。

不需要硬件，也不需要 rclpy 或 pyserial，本地就能跑：

    cd src/cquick_servo && pytest test/ -v
    colcon test --packages-select cquick_servo
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from cquick_servo.servo_protocol import (  # noqa: E402
    CMD_ID, CMD_POS, CMD_TORQUE_OFF, CMD_TORQUE_ON,
    build_move, build_query, parse_pwm, validate_limits)


# ---------------- 拼装 ----------------
def test_spec_example_frame():
    """细则给的例子：#1,1700,1000 -> #001P1700T1000!"""
    assert build_move(1, 1700, 1000) == '#001P1700T1000!'


def test_build_move_exact_frames():
    assert build_move(0, 1500, 1000) == '#000P1500T1000!'
    assert build_move(0, 1700, 1500) == '#000P1700T1500!'
    assert build_move(7, 2000, 800) == '#007P2000T800!'


def test_build_move_pads_id_to_three_digits():
    assert build_move(1, 1500, 800) == '#001P1500T800!'
    assert build_move(23, 1500, 800) == '#023P1500T800!'
    assert build_move(254, 2000, 800).startswith('#254')


def test_build_query_covers_all_four_functions():
    assert build_query(0, CMD_ID) == '#000PID!'
    assert build_query(0, CMD_POS) == '#000PRAD!'
    assert build_query(0, CMD_TORQUE_ON) == '#000PULR!'
    assert build_query(0, CMD_TORQUE_OFF) == '#000PULK!'
    assert build_query(12, CMD_POS) == '#012PRAD!'


def test_frames_are_ascii_and_end_with_bang():
    for frame in (build_move(0, 1500, 800), build_query(0, CMD_POS)):
        assert frame.startswith('#')
        assert frame.endswith('!')
        frame.encode('ascii')          # 串口 8-N-1 发的是 ASCII，拼错了这里就抛


def test_query_and_move_share_id_format():
    assert build_move(9, 1500, 800)[:4] == build_query(9, CMD_POS)[:4] == '#009'


# ---------------- 解析 ----------------
def test_parse_pwm_from_real_replies():
    # 任务 1 实测读回过的几个值
    assert parse_pwm('#000P1493!') == 1493
    assert parse_pwm('#000P1510!') == 1510
    assert parse_pwm('#000P1690!') == 1690
    assert parse_pwm('#000P1691!') == 1691


def test_parse_pwm_tolerates_crlf_and_garbage():
    assert parse_pwm('#000P1500!\r\n') == 1500
    assert parse_pwm('  #000P1492!  ') == 1492


def test_parse_pwm_returns_none_for_id_reply():
    # #000P! 是读 ID 的回包，里面没有位置数字，不能误解析成 0 或 000
    assert parse_pwm('#000P!') is None
    assert parse_pwm('') is None
    assert parse_pwm(None) is None
    assert parse_pwm('(无回包)') is None


def test_parse_pwm_fallback_for_bare_number():
    # 有些控制板回包不带 P 前缀
    assert parse_pwm('1500') == 1500


# ---------------- 限位组合 ----------------
def test_limits_accept_working_config():
    assert validate_limits(900, 2100, 1500)[0] is True


def test_limits_reject_inverted_range():
    ok, reason = validate_limits(2100, 900, 1500)
    assert ok is False and 'less than' in reason


def test_limits_reject_equal_bounds():
    assert validate_limits(1500, 1500, 1500)[0] is False


def test_limits_reject_neutral_outside_range():
    ok, reason = validate_limits(900, 1400, 1500)
    assert ok is False and 'neutral_pwm' in reason


def test_limits_reject_out_of_electrical_range():
    assert validate_limits(400, 2100, 1500)[0] is False     # 低于 500
    assert validate_limits(900, 2600, 1500)[0] is False     # 高于 2500
