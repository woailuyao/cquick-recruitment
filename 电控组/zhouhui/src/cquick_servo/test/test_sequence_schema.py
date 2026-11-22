"""动作 yaml 解析/校验的单测：证明「动作是数据」这件事在代码层面是成立的。

跑法：python3 -m pytest test/test_sequence_schema.py -v
"""
import pytest

from cquick_servo.sequence_schema import (
    SequenceError,
    parse_sequence,
    total_duration_ms,
)


def _good():
    return {
        "name": "demo",
        "default_duration_ms": 800,
        "steps": [
            {"label": "a", "moves": [{"id": 1, "pwm": 1600},
                                     {"id": 2, "pwm": 1400, "duration_ms": 1200}]},
            {"label": "b", "moves": [{"id": 1, "pwm": 1500, "duration_ms": 800}]},
        ],
    }


def test_parse_ok():
    seq = parse_sequence(_good())
    assert seq.name == "demo"
    assert len(seq.steps) == 2
    assert [m.sid for m in seq.steps[0].moves] == [1, 2]
    # 没写 duration_ms 的走 default
    assert seq.steps[0].moves[0].duration_ms == 800
    assert seq.steps[0].moves[1].duration_ms == 1200
    assert seq.steps[0].label == "a"


def test_total_duration_takes_slowest_in_step():
    # 第 1 步最慢 1200，第 2 步 800 -> 2000
    assert total_duration_ms(parse_sequence(_good())) == 2000


def test_empty_steps_rejected():
    with pytest.raises(SequenceError):
        parse_sequence({"steps": []})


def test_steps_must_be_list():
    with pytest.raises(SequenceError):
        parse_sequence({"steps": {"moves": []}})


def test_missing_moves_rejected():
    with pytest.raises(SequenceError):
        parse_sequence({"steps": [{"label": "x"}]})


def test_missing_id_rejected():
    with pytest.raises(SequenceError):
        parse_sequence({"steps": [{"moves": [{"pwm": 1600}]}]})


@pytest.mark.parametrize("pwm", [499, 2501, 0])
def test_pwm_out_of_electric_range(pwm):
    with pytest.raises(SequenceError):
        parse_sequence({"steps": [{"moves": [{"id": 1, "pwm": pwm}]}]})


@pytest.mark.parametrize("sid", [0, 255])
def test_bad_id_rejected(sid):
    with pytest.raises(SequenceError):
        parse_sequence({"steps": [{"moves": [{"id": sid, "pwm": 1500}]}]})


def test_duplicate_id_in_one_step_rejected():
    # 同一个 step 里一个舵机两个目标，等于自己跟自己打架，必须拒
    with pytest.raises(SequenceError):
        parse_sequence({"steps": [{"moves": [{"id": 1, "pwm": 1600},
                                             {"id": 1, "pwm": 1400}]}]})


def test_bad_duration_rejected():
    with pytest.raises(SequenceError):
        parse_sequence({"steps": [{"moves": [{"id": 1, "pwm": 1500,
                                              "duration_ms": 0}]}]})


def test_string_id_rejected():
    with pytest.raises(SequenceError):
        parse_sequence({"steps": [{"moves": [{"id": "1", "pwm": 1500}]}]})


def test_same_id_across_steps_is_fine():
    # 前后两步控制同一个舵机是正常的（先抬后落）
    doc = {"steps": [{"moves": [{"id": 1, "pwm": 1600}]},
                     {"moves": [{"id": 1, "pwm": 1500}]}]}
    assert len(parse_sequence(doc).steps) == 2
