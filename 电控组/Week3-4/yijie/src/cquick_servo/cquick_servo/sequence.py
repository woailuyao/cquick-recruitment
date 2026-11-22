# -*- coding: utf-8 -*-
"""sequence.py —— 动作组 yaml 的解析与校验（纯逻辑，不依赖 ROS）。

一套动作 = 若干个 step 组成的有序列表；
每个 step = 若干条「哪个舵机 / 目标 PWM / 用多久」，**同一步内的会同时下发**。

    name: stand_up
    repeat: 1
    steps:
      - name: 01_legs
        joints:
          - {id: 1, pwm: 1620, duration_ms: 800}
          - {id: 2, pwm: 1380, duration_ms: 800}

校验在这里做完，播放器就只负责「发 goal + 等 result」。
"""

from dataclasses import dataclass
from typing import Optional

from .protocol import PWM_MAX_ELECTRIC, PWM_MIN_ELECTRIC, is_valid_id

MIN_DURATION_MS = 50
MAX_DURATION_MS = 20000
MAX_REPEAT = 100


class SequenceError(ValueError):
    """动作 yaml 写错了。消息里一定带上是哪一步、哪个关节、错在哪。"""


@dataclass(frozen=True)
class JointTarget:
    servo_id: int
    pwm: int
    duration_ms: int


@dataclass(frozen=True)
class Step:
    name: str
    joints: tuple


@dataclass(frozen=True)
class Sequence:
    name: str
    steps: tuple
    repeat: int = 1
    source: str = ''


def _as_int(value, where: str, field: str) -> int:
    if isinstance(value, bool) or value is None:
        raise SequenceError(f'{where}: {field} 应该是整数，实际是 {value!r}')
    try:
        return int(value)
    except (TypeError, ValueError):
        raise SequenceError(f'{where}: {field} 应该是整数，实际是 {value!r}')


def parse_joint(raw, where: str) -> JointTarget:
    if not isinstance(raw, dict):
        raise SequenceError(f'{where}: 每个关节必须是 {{id, pwm, duration_ms}} 形式')

    if 'id' not in raw:
        raise SequenceError(f'{where}: 缺少 id')
    if 'pwm' not in raw:
        raise SequenceError(f'{where}: 缺少 pwm')

    servo_id = _as_int(raw['id'], where, 'id')
    if not is_valid_id(servo_id):
        raise SequenceError(f'{where}: id={servo_id} 非法（0~254）')

    pwm = _as_int(raw['pwm'], where, 'pwm')
    if not PWM_MIN_ELECTRIC <= pwm <= PWM_MAX_ELECTRIC:
        raise SequenceError(
            f'{where}: pwm={pwm} 超出电气范围 {PWM_MIN_ELECTRIC}~{PWM_MAX_ELECTRIC}')

    duration = _as_int(raw.get('duration_ms', 0), where, 'duration_ms')
    if duration != 0 and not MIN_DURATION_MS <= duration <= MAX_DURATION_MS:
        raise SequenceError(
            f'{where}: duration_ms={duration} 非法（0 表示用默认值，否则 {MIN_DURATION_MS}~{MAX_DURATION_MS}）')

    return JointTarget(servo_id=servo_id, pwm=pwm, duration_ms=duration)


def parse_step(raw, index: int) -> Step:
    where = f'第 {index} 个 step'
    if not isinstance(raw, dict):
        raise SequenceError(f'{where}: 必须是 {{name, joints}} 形式')

    name = str(raw.get('name') or f'step{index}')
    raw_joints = raw.get('joints')
    if not isinstance(raw_joints, list) or not raw_joints:
        raise SequenceError(f'{where} ({name}): joints 必须是非空列表')

    joints = []
    seen = set()
    for joint_index, raw_joint in enumerate(raw_joints, start=1):
        joint = parse_joint(raw_joint, f'{where} ({name}) 第 {joint_index} 个关节')
        if joint.servo_id in seen:
            raise SequenceError(
                f'{where} ({name}): 同一个 step 里 id={joint.servo_id} 出现了两次；'
                f'要「有先后」请拆成两个 step')
        seen.add(joint.servo_id)
        joints.append(joint)

    return Step(name=name, joints=tuple(joints))


def parse_sequence(data, source: str = '<memory>') -> Sequence:
    """把已经解析好的 dict（yaml.safe_load 的结果）校验成 Sequence。"""
    if not isinstance(data, dict):
        raise SequenceError(f'{source}: 顶层必须是 mapping（name/steps）')

    name = str(data.get('name') or source)
    repeat = _as_int(data.get('repeat', 1), source, 'repeat')
    if not 1 <= repeat <= MAX_REPEAT:
        raise SequenceError(f'{source}: repeat={repeat} 非法（1~{MAX_REPEAT}）')

    raw_steps = data.get('steps')
    if not isinstance(raw_steps, list) or not raw_steps:
        raise SequenceError(f'{source}: steps 必须是非空列表')

    steps = tuple(parse_step(raw_step, i) for i, raw_step in enumerate(raw_steps, start=1))
    return Sequence(name=name, steps=steps, repeat=repeat, source=source)


def load_sequence(path: str) -> Sequence:
    """读磁盘上的 yaml 并校验。PyYAML 在板上由 python3-yaml 提供。"""
    try:
        import yaml
    except ImportError as exc:      # pragma: no cover - 板上一定装了
        raise SequenceError('缺少 PyYAML：sudo apt install -y python3-yaml') from exc

    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return parse_sequence(data, source=str(path))


def total_duration_ms(sequence: Sequence) -> Optional[int]:
    """理论总时长（各 step 取最长关节），只用于日志/自测，不作为同步依据。"""
    if not sequence.steps:
        return None
    total = 0
    for step in sequence.steps:
        total += max(joint.duration_ms for joint in step.joints)
    return total * sequence.repeat
