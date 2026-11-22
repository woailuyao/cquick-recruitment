# -*- coding: utf-8 -*-
"""protocol.py 的单测：不接硬件、不需要 ROS，任何机器都能跑。"""

from cquick_servo.protocol import (
    build_move, build_pid, build_prad, build_pulk, build_pulr,
    clamp_pwm, format_id, is_valid_id, parse_id_reply, parse_pwm_reply,
)


def test_format_id_pads_to_three():
    assert format_id(1) == '001'
    assert format_id(12) == '012'
    assert format_id(123) == '123'


def test_build_move_exact_frame():
    # 考核文档里的例子：#1,1700,1000 -> #001P1700T1000!
    assert build_move(1, 1700, 1000) == '#001P1700T1000!'


def test_build_move_pads_short_id():
    assert build_move(7, 1500, 500) == '#007P1500T500!'
    assert build_move(12, 900, 800) == '#012P900T800!'


def test_other_frames():
    assert build_pid(3) == '#003PID!'
    assert build_prad(3) == '#003PRAD!'
    assert build_pulr(3) == '#003PULR!'
    assert build_pulk(3) == '#003PULK!'


def test_every_frame_is_hash_bang_no_newline():
    frames = [build_pid(1), build_move(1, 1500, 100), build_prad(1), build_pulr(1), build_pulk(1)]
    for frame in frames:
        assert frame.startswith('#')
        assert frame.endswith('!')
        assert '\n' not in frame and '\r' not in frame


def test_clamp_pwm():
    assert clamp_pwm(3000, 900, 2100) == 2100
    assert clamp_pwm(100, 900, 2100) == 900
    assert clamp_pwm(1500, 900, 2100) == 1500


def test_is_valid_id():
    assert is_valid_id(0) and is_valid_id(254)      # 0 是合法的：有的板子从 000 编址
    assert not is_valid_id(255)


def test_parse_pwm_reply():
    assert parse_pwm_reply(b'#001P1500!') == 1500
    assert parse_pwm_reply(b'#012P0900!') == 900
    assert parse_pwm_reply(b'001P2100') == 2100
    assert parse_pwm_reply(b'') is None
    assert parse_pwm_reply(b'\xff\xfe') is None


def test_parse_id_reply():
    assert parse_id_reply(b'#007!') == '007'
    assert parse_id_reply(b'') is None
