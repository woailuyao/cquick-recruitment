"""动作组 yaml 校验的单元测试：不接硬件、不起节点也能跑。

通过 colcon test 验证「yaml 写错会被人话报错挡下来」，而不是默默发歪的 goal。
"""
import pytest

from cquick_servo.sequence_player import validate_sequence


def test_valid_sequence():
    """正常两 step 动作：并行 + 串行结构都能解析。"""
    seq = {
        "name": "demo",
        "default_duration_ms": 500,
        "steps": [
            {"name": "同时动",
             "joints": [{"id": 1, "pwm": 1600},
                        {"id": 2, "pwm": 1400, "duration_ms": 600}]},
            {"name": "再动", "joints": [{"id": 3, "pwm": 1500}]},
        ],
    }
    steps, default_dur = validate_sequence(seq)
    assert default_dur == 500
    assert steps[0][1] == [(1, 1600, 500), (2, 1400, 600)]
    assert steps[1][1] == [(3, 1500, 500)]


def test_missing_steps():
    """没有 steps 必须报错，不能当作空动作跑过去。"""
    with pytest.raises(ValueError, match="steps"):
        validate_sequence({"name": "empty"})


def test_empty_joints():
    """step 里 joints 为空是写错了，不是「这一步什么都不做」。"""
    with pytest.raises(ValueError, match="joints"):
        validate_sequence({"steps": [{"name": "x", "joints": []}]})


def test_duplicate_id_in_one_step():
    """同一步里同一个舵机给两个目标，结构会自己打架，必须拦。"""
    with pytest.raises(ValueError, match="重复"):
        validate_sequence({"steps": [{"name": "x", "joints": [
            {"id": 1, "pwm": 1500}, {"id": 1, "pwm": 1700}]}]})


def test_pwm_out_of_range():
    """pwm 超接口范围要拦住；安全区间由驱动节点的 pwm_min/pwm_max 再判一次。"""
    with pytest.raises(ValueError, match="pwm"):
        validate_sequence({"steps": [{"name": "x", "joints": [
            {"id": 1, "pwm": 70000}]}]})


def test_bad_duration():
    """duration_ms 为 0 等于让舵机瞬跳，必须拦。"""
    with pytest.raises(ValueError, match="duration_ms"):
        validate_sequence({"steps": [{"name": "x", "joints": [
            {"id": 1, "pwm": 1500, "duration_ms": 0}]}]})
