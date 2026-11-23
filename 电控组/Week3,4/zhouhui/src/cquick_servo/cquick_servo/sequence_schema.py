"""动作组 YAML 的解析与校验 —— 又不碰 ROS，也不碰串口，所以能单测。

一套动作 = steps 的有序列表：
    steps[i].moves  同一个 step 里的若干条 move，会同时下发（并行）
    steps           步与步之间严格串行：上一 step 的 result 全回来才发下一 step

想表达「先后」，就拆成两个 step；不要靠把 duration_ms 调大好让某个关节晚到，
那是靠时间猜，跑三遍必然不一致。
"""
from dataclasses import dataclass, field

ID_LO = 1
ID_HI = 254
PWM_ELEC_LO = 500
PWM_ELEC_HI = 2500
DURATION_HI = 9999


class SequenceError(ValueError):
    """动作文件写得不对。"""


@dataclass
class Move:
    sid: int
    pwm: int
    duration_ms: int


@dataclass
class Step:
    label: str
    moves: list = field(default_factory=list)


@dataclass
class Sequence:
    name: str
    description: str
    default_duration_ms: int
    steps: list = field(default_factory=list)


def _as_int(value, what, where):
    if isinstance(value, bool) or not isinstance(value, int):
        raise SequenceError(f"{where} 的 {what} 必须是整数，收到 {value!r}")
    return value


def parse_sequence(doc, default_duration_ms=800):
    """把 yaml.safe_load 出来的 dict 变成 Sequence 对象，任何不合法都抛 SequenceError。"""
    if not isinstance(doc, dict):
        raise SequenceError("yaml 顶层必须是字典（name / steps / default_duration_ms）")

    steps_raw = doc.get("steps")
    if not isinstance(steps_raw, list) or not steps_raw:
        raise SequenceError("steps 必须是非空列表")

    fallback = _as_int(doc.get("default_duration_ms", default_duration_ms),
                       "default_duration_ms", "顶层")
    if not 1 <= fallback <= DURATION_HI:
        raise SequenceError(f"default_duration_ms={fallback} 不在 1~{DURATION_HI}")

    steps = []
    for idx, raw_step in enumerate(steps_raw, start=1):
        where = f"step{idx}"
        if not isinstance(raw_step, dict):
            raise SequenceError(f"{where} 必须是字典")
        moves_raw = raw_step.get("moves")
        if not isinstance(moves_raw, list) or not moves_raw:
            raise SequenceError(f"{where} 的 moves 必须是非空列表")
        label = str(raw_step.get("label") or raw_step.get("name") or f"step{idx}")

        seen = set()
        moves = []
        for raw_move in moves_raw:
            if not isinstance(raw_move, dict):
                raise SequenceError(f"{where} 的 move 必须是字典")
            for key in ("id", "pwm"):
                if key not in raw_move:
                    raise SequenceError(f"{where} 的 move 缺 {key}")
            sid = _as_int(raw_move["id"], "id", where)
            pwm = _as_int(raw_move["pwm"], "pwm", where)
            dur = _as_int(raw_move.get("duration_ms", fallback), "duration_ms", where)
            if not ID_LO <= sid <= ID_HI:
                raise SequenceError(f"{where} 的 id={sid} 不在 {ID_LO}~{ID_HI}")
            if not PWM_ELEC_LO <= pwm <= PWM_ELEC_HI:
                raise SequenceError(
                    f"{where} id={sid} 的 pwm={pwm} 超出电气范围 "
                    f"{PWM_ELEC_LO}~{PWM_ELEC_HI}")
            if not 1 <= dur <= DURATION_HI:
                raise SequenceError(
                    f"{where} id={sid} 的 duration_ms={dur} 不在 1~{DURATION_HI}")
            if sid in seen:
                raise SequenceError(
                    f"{where} 里 id={sid} 出现两次：同一步内一个舵机只能有一个目标")
            seen.add(sid)
            moves.append(Move(sid, pwm, dur))
        steps.append(Step(label, moves))

    return Sequence(
        name=str(doc.get("name", "unnamed")),
        description=str(doc.get("description", "")),
        default_duration_ms=fallback,
        steps=steps,
    )


def load_sequence(path, default_duration_ms=800):
    """从文件读。yaml 语法错也转成 SequenceError，方便上层统一处理。"""
    import yaml

    try:
        with open(path, "r", encoding="utf-8") as fp:
            doc = yaml.safe_load(fp)
    except OSError as exc:
        raise SequenceError(f"动作文件打不开：{exc}") from exc
    except yaml.YAMLError as exc:
        raise SequenceError(f"动作文件 YAML 语法错：{exc}") from exc
    return parse_sequence(doc, default_duration_ms)


def total_duration_ms(sequence):
    """整套动作的理论时长（每个 step 取最慢的那一条），只用来打印预告。"""
    total = 0
    for step in sequence.steps:
        total += max(m.duration_ms for m in step.moves)
    return total
