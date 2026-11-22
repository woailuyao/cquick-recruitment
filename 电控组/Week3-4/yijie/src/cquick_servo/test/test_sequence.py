# -*- coding: utf-8 -*-
"""sequence.py 的单测：yaml 结构校验（用 dict 直接喂，不需要 PyYAML）。"""

from cquick_servo.sequence import SequenceError, parse_sequence, total_duration_ms


def _expect_error(data, keyword):
    try:
        parse_sequence(data, source='test.yaml')
    except SequenceError as exc:
        assert keyword in str(exc), f'错误信息应包含 {keyword!r}，实际：{exc}'
        return
    raise AssertionError(f'应该报 SequenceError（含 {keyword!r}）但没有报')


def test_parse_two_steps():
    sequence = parse_sequence({
        'name': 'demo',
        'steps': [
            {'name': 's1', 'joints': [
                {'id': 1, 'pwm': 1600, 'duration_ms': 800},
                {'id': 2, 'pwm': 1400},
            ]},
            {'joints': [{'id': 3, 'pwm': 1500, 'duration_ms': 500}]},
        ],
    }, source='demo.yaml')

    assert sequence.name == 'demo'
    assert len(sequence.steps) == 2
    assert sequence.repeat == 1
    assert sequence.steps[0].joints[0].servo_id == 1
    assert sequence.steps[0].joints[1].duration_ms == 0      # 缺省 -> 用节点默认时长
    assert sequence.steps[1].name == 'step2'                  # 没写 name 就用默认
    assert total_duration_ms(sequence) == 800 + 500


def test_source_used_as_default_name():
    sequence = parse_sequence({'steps': [{'joints': [{'id': 1, 'pwm': 1500}]}]},
                              source='wave.yaml')
    assert sequence.name == 'wave.yaml'


def test_repeat_from_file():
    sequence = parse_sequence({'repeat': 2, 'steps': [{'joints': [{'id': 1, 'pwm': 1500}]}]})
    assert sequence.repeat == 2


def test_missing_steps():
    _expect_error({'name': 'x'}, 'steps')


def test_empty_steps():
    _expect_error({'steps': []}, 'steps')


def test_step_without_joints():
    _expect_error({'steps': [{'name': 's1'}]}, 'joints')


def test_duplicate_servo_in_one_step_is_rejected():
    _expect_error({'steps': [{'joints': [
        {'id': 1, 'pwm': 1500},
        {'id': 1, 'pwm': 1600},
    ]}]}, '出现了两次')


def test_pwm_out_of_electrical_range():
    _expect_error({'steps': [{'joints': [{'id': 1, 'pwm': 2600}]}]}, '电气范围')


def test_invalid_id():
    _expect_error({'steps': [{'joints': [{'id': 255, 'pwm': 1500}]}]}, '非法')


def test_duration_too_small():
    _expect_error({'steps': [{'joints': [{'id': 1, 'pwm': 1500, 'duration_ms': 10}]}]},
                  'duration_ms')


def test_missing_pwm():
    _expect_error({'steps': [{'joints': [{'id': 1}]}]}, 'pwm')


def test_repeat_out_of_range():
    _expect_error({'repeat': 0, 'steps': [{'joints': [{'id': 1, 'pwm': 1500}]}]}, 'repeat')


def test_top_level_must_be_mapping():
    _expect_error([1, 2, 3], 'mapping')
