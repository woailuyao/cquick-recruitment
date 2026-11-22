"""协议拼装单测：不插板子、不开串口，纯字符串校验。

跑法：colcon test --packages-select cquick_servo --event-handlers console_direct+
或   cd src/cquick_servo && python3 -m pytest test/ -v
"""
import pytest

from cquick_servo.servo_ascii import (
    enable_cmd,
    move_cmd,
    parse_position,
    plan_duration,
    read_id_cmd,
    read_pos_cmd,
    release_cmd,
    sid_text,
)


def test_sid_is_three_digits():
    assert sid_text(1) == "001"
    assert sid_text(12) == "012"
    assert sid_text(254) == "254"


def test_move_command_is_the_documented_line():
    # 题目里的标尺： (1, 1700, 1000) -> '#001P1700T1000!'
    assert move_cmd(1, 1700, 1000) == "#001P1700T1000!"
    assert move_cmd(12, 1500, 800) == "#012P1500T800!"


def test_move_command_shape():
    line = move_cmd(3, 1330, 2500)
    assert line.startswith("#")
    assert line.endswith("!")
    assert " " not in line
    assert line.count("#") == 1 and line.count("!") == 1
    assert line == "#003P1330T2500!"


def test_protocol_commands():
    assert read_id_cmd(1) == "#001PID!"
    assert read_pos_cmd(4) == "#004PRAD!"
    assert enable_cmd(1) == "#001PULR!"
    assert release_cmd(6) == "#006PULK!"


@pytest.mark.parametrize("sid", [0, -1, 255, 999])
def test_bad_id_rejected(sid):
    with pytest.raises(ValueError):
        sid_text(sid)


@pytest.mark.parametrize("sid", ["1", 1.0, None])
def test_non_int_id_rejected(sid):
    with pytest.raises(TypeError):
        sid_text(sid)


@pytest.mark.parametrize("pwm", [499, 2501, 0, -50])
def test_pwm_out_of_electric_range_rejected(pwm):
    with pytest.raises(ValueError):
        move_cmd(1, pwm, 800)


@pytest.mark.parametrize("dur", [0, -1])
def test_non_positive_duration_rejected(dur):
    with pytest.raises(ValueError):
        move_cmd(1, 1500, dur)


def test_bool_is_not_int():
    # True 在 python 里等价于 1，放进来会拼出 id=001，必须显式拒掉
    with pytest.raises(TypeError):
        sid_text(True)


def test_parse_position_basic():
    assert parse_position("#001P1794!") == 1794
    assert parse_position(b"#001P1500!") == 1500
    assert parse_position("#002P0900!") == 900
    assert parse_position("#006P2100!") == 2100


def test_parse_position_tolerates_noise():
    # 半双工总线上会混进别的回显，取最后一个 P<数字>
    assert parse_position("#001P1200!#001P1206!") == 1206
    assert parse_position("xx#001P1200!yy") == 1200


def test_parse_position_none_cases():
    assert parse_position(None) is None
    assert parse_position("") is None
    assert parse_position("ERROR") is None
    assert parse_position(b"#001PID!") is None


def test_plan_duration_uses_request_when_close():
    # 差 60，按 600/s 只要 100 ms，请求 800 更大 -> 800
    assert plan_duration(1500, 1560, 800) == 800


def test_plan_duration_stretches_for_long_move():
    # 差 1200，按 600/s 要 2000 ms，请求只有 200 -> 2000
    assert plan_duration(900, 2100, 200) == 2000


def test_plan_duration_floor():
    assert plan_duration(1500, 1500, 50) == 200


def test_plan_duration_without_position():
    assert plan_duration(None, 1700, 500) == 500


def test_plan_duration_speed_scales():
    # 速度减半，时间翻倍
    assert plan_duration(1200, 1800, 200, speed_pwm_per_s=300) == 2000
