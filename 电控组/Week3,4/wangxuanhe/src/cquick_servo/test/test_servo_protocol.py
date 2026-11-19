"""协议拼装单测：不接硬件、不开串口，只验证字符串拼得对不对。

运行：colcon test --packages-select cquick_servo  或  pytest test/
"""
import pytest

from cquick_servo.servo_protocol import (
    cmd_move,
    cmd_pid,
    cmd_prad,
    cmd_torque_off,
    cmd_torque_on,
    format_id,
    parse_prad,
    smooth_duration_ms,
)


def test_id_is_three_digits_with_zero_padding():
    assert format_id(1) == "001"
    assert format_id(12) == "012"
    assert format_id(254) == "254"


def test_move_command_matches_protocol_line():
    # 题目里那条： (#1, 1700, 1000) → '#001P1700T1000!'
    assert cmd_move(1, 1700, 1000) == "#001P1700T1000!"
    assert cmd_move(12, 1500, 800) == "#012P1500T800!"


def test_command_shape_start_hash_end_bang_no_space():
    cmd = cmd_move(1, 1700, 1000)
    assert cmd.startswith("#")
    assert cmd.endswith("!")
    assert " " not in cmd
    assert cmd.count("#") == 1
    assert cmd.count("!") == 1


def test_read_commands():
    assert cmd_pid(1) == "#001PID!"
    assert cmd_prad(3) == "#003PRAD!"
    assert cmd_torque_on(1) == "#001PULR!"
    assert cmd_torque_off(6) == "#006PULK!"


@pytest.mark.parametrize("sid", [0, -1, 255, 1000])
def test_bad_servo_id_rejected(sid):
    with pytest.raises(ValueError):
        format_id(sid)


def test_non_integer_id_rejected():
    with pytest.raises(TypeError):
        format_id("1")


@pytest.mark.parametrize("pwm", [499, 2501, 0, -100])
def test_pwm_out_of_electric_range_rejected(pwm):
    with pytest.raises(ValueError):
        cmd_move(1, pwm, 1000)


@pytest.mark.parametrize("duration_ms", [0, -5])
def test_bad_duration_rejected(duration_ms):
    with pytest.raises(ValueError):
        cmd_move(1, 1500, duration_ms)


def test_parse_prad_reads_back_value():
    assert parse_prad("#001P1794!") == 1794
    assert parse_prad(b"#001P1794!") == 1794
    assert parse_prad("#001P1500!") == 1500
    assert parse_prad("#002P0900!") == 900


def test_parse_prad_tolerates_noise():
    # 串口里混进多余报文时，取最后一段
    assert parse_prad("#001P1200!#001P1206!") == 1206


def test_parse_prad_returns_none_on_garbage():
    assert parse_prad("") is None
    assert parse_prad(None) is None
    assert parse_prad("ERROR") is None


def test_smooth_duration_keeps_requested_when_close():
    # 只差 60，按 600/s 只需 100ms，requested 800 更大 → 用 800
    assert smooth_duration_ms(1500, 1560, 800) == 800


def test_smooth_duration_stretches_for_long_move():
    # 差 1200，按 600/s 要 2000ms，requested 只有 200 → 取 2000
    assert smooth_duration_ms(900, 2100, 200) == 2000


def test_smooth_duration_without_position_falls_back():
    assert smooth_duration_ms(None, 1700, 500) == 500


def test_smooth_duration_never_below_minimum():
    assert smooth_duration_ms(1500, 1500, 50) == 200


def test_smooth_duration_slows_faster_speed_setting():
    # 速度调小一半，花的时间应该翻倍
    assert smooth_duration_ms(1200, 1800, 200, speed_pwm_per_s=300) == 2000
