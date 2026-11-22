"""总线舵机 ASCII 协议拼装 —— 纯函数，不 import rclpy / serial。

这样单测可以不插板子、不开串口就跑过；驱动节点也调用这里的函数，
保证「线上真正发出去的字符串」和「单测验证过的字符串」是同一条。
"""
import re

ID_LO = 1
ID_HI = 254
ID_PAD = 3

# 电气极限（500~2500 是舵机能接受的脉宽范围；结构安全范围是参数 pwm_min/pwm_max）
PWM_ELEC_LO = 500
PWM_ELEC_HI = 2500

_POS_RE = re.compile(r"P(\d{1,5})")


def _check_int(value, what):
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{what} 必须是 int，收到 {value!r}")


def sid_text(sid):
    """1 -> '001'；ID 固定 3 位补零，这是协议要求。"""
    _check_int(sid, "舵机 id")
    if not ID_LO <= sid <= ID_HI:
        raise ValueError(f"舵机 id 必须落在 {ID_LO}~{ID_HI}，收到 {sid}")
    return f"{sid:0{ID_PAD}d}"


def move_cmd(sid, pwm, duration_ms):
    """(1, 1700, 1000) -> '#001P1700T1000!'"""
    _check_int(pwm, "pwm")
    if not PWM_ELEC_LO <= pwm <= PWM_ELEC_HI:
        raise ValueError(f"pwm 超出电气范围 {PWM_ELEC_LO}~{PWM_ELEC_HI}，收到 {pwm}")
    _check_int(duration_ms, "duration_ms")
    if duration_ms <= 0:
        raise ValueError(f"duration_ms 必须为正，收到 {duration_ms}")
    return f"#{sid_text(sid)}P{pwm}T{duration_ms}!"


def read_id_cmd(sid):
    """扫 ID：'#001PID!'"""
    return f"#{sid_text(sid)}PID!"


def read_pos_cmd(sid):
    """读回位置：'#001PRAD!'"""
    return f"#{sid_text(sid)}PRAD!"


def enable_cmd(sid):
    """恢复扭力：动之前必须发，否则舵机纹丝不动。"""
    return f"#{sid_text(sid)}PULR!"


def release_cmd(sid):
    """卸力：下电前必发。"""
    return f"#{sid_text(sid)}PULK!"


def parse_position(raw):
    """从回显里抠出位置。'#001P1794!' -> 1794；解析不出 -> None。

    回显里可能混着别的报文（半双工总线同一根线），所以取最后一个 P<数字>，
    而不是简单取结尾数字。
    """
    if raw is None:
        return None
    if isinstance(raw, (bytes, bytearray)):
        try:
            text = raw.decode("ascii", errors="ignore")
        except Exception:
            return None
    else:
        text = str(raw)
    hits = _POS_RE.findall(text)
    if not hits:
        return None
    try:
        return int(hits[-1])
    except ValueError:
        return None


def plan_duration(cur_pwm, target_pwm, requested_ms, speed_pwm_per_s=600, floor_ms=200):
    """给「当前位置 → 目标」算一个不会猛跳的时长。

    取值 = max(请求时长, 按限速算出的时长, 地板值)。
    读不到当前位置时只按请求时长走。
    """
    requested = max(int(requested_ms), int(floor_ms))
    if cur_pwm is None:
        return requested
    distance = abs(int(target_pwm) - int(cur_pwm))
    speed = max(int(speed_pwm_per_s), 1)
    need = int(distance * 1000 / speed)
    return max(requested, need)
