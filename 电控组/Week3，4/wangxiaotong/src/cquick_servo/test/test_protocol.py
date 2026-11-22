"""协议单测：不接硬件就能跑（加分项 +2）。

    colcon test --packages-select cquick_servo
    colcon test-result --verbose
"""
import pytest

from cquick_servo.protocol import (
    ProtocolError,
    build_move,
    build_query,
    build_torque_on,
    build_unload,
    parse_pwm,
    parse_triplet,
)


def test_build_move_basic():
    assert build_move(1, 1700, 1000) == '#001P1700T1000!'


def test_triplet_to_command():
    # 考核文档里给的那条：'#1,1700,1000' -> '#001P1700T1000!'
    assert build_move(*parse_triplet('#1,1700,1000')) == '#001P1700T1000!'


@pytest.mark.parametrize('servo_id, prefix', [(1, '#001P'), (7, '#007P'), (254, '#254P')])
def test_id_zero_padded_three_digits(servo_id, prefix):
    assert build_move(servo_id, 1500, 800).startswith(prefix)


@pytest.mark.parametrize('servo_id', [0, 255, -1])
def test_id_out_of_range(servo_id):
    with pytest.raises(ProtocolError):
        build_move(servo_id, 1500, 800)


@pytest.mark.parametrize('pwm', [499, 2501])
def test_pwm_outside_electrical_range(pwm):
    with pytest.raises(ProtocolError):
        build_move(1, pwm, 800)


def test_pwm_electrical_edges_are_legal():
    assert build_move(1, 500, 800) == '#001P500T800!'
    assert build_move(1, 2500, 800) == '#001P2500T800!'


@pytest.mark.parametrize('duration', [0, -5, 60001])
def test_duration_out_of_range(duration):
    with pytest.raises(ProtocolError):
        build_move(1, 1500, duration)


def test_queries():
    assert build_query(1, 'PID') == '#001PID!'
    assert build_query(3, 'prad') == '#003PRAD!'
    assert build_torque_on(2) == '#002PULR!'
    assert build_unload(6) == '#006PULK!'


def test_bad_query_code():
    with pytest.raises(ProtocolError):
        build_query(1, 'PXXX')


@pytest.mark.parametrize('frame, expected', [
    ('#001P1500!', 1500),
    ('#001P0001!', 1),
    ('P1500!', 1500),
    ('#001P2500!\r\n', 2500),
    ('', None),
    (None, None),
    ('garbage', None),
    ('#001P1500T1000!', None),   # 这是指令不是回显，不能误解析
])
def test_parse_pwm(frame, expected):
    assert parse_pwm(frame) == expected


def test_parse_triplet_rejects_garbage():
    with pytest.raises(ProtocolError):
        parse_triplet('#1,1700')
    with pytest.raises(ProtocolError):
        parse_triplet('#a,b,c')

