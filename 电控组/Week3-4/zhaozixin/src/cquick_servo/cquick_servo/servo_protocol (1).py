"""总线舵机 ASCII 协议：把「哪个舵机 / 什么位置 / 用多久」拼成指令字符串。

刻意做成纯函数模块——不 import rclpy、不 import serial，
所以单测可以不接硬件、不连串口就跑过（pytest 直接调）。
驱动节点从这里取指令，保证「代码发出去的」和「单测验证过的」是同一条字符串。
"""

# 协议常量：这些是协议本身，不算配置，可以写在代码里
ID_MIN = 1
ID_MAX = 254
ID_WIDTH = 3

# 电气极限范围（不是结构安全范围，安全区间走 pwm_min / pwm_max 参数）
PWM_ELEC_MIN = 500
PWM_ELEC_MAX = 2500


def format_id(sid: int) -> str:
    """ID 固定 3 位补零：1 → '001'。"""
    if isinstance(sid, bool) or not isinstance(sid, int):
        raise TypeError(f"舵机 ID 必须是整数，收到 {sid!r}")
    if not ID_MIN <= sid <= ID_MAX:
        raise ValueError(f"舵机 ID 必须在 {ID_MIN}~{ID_MAX} 之间，收到 {sid}")
    return f"{sid:0{ID_WIDTH}d}"


def cmd_pid(sid: int) -> str:
    """读 ID。"""
    return f"#{format_id(sid)}PID!"


def cmd_prad(sid: int) -> str:
    """读当前位置 PWM。"""
    return f"#{format_id(sid)}PRAD!"


def cmd_torque_on(sid: int) -> str:
    """恢复扭力。动之前必须先发这条，否则舵机纹丝不动。"""
    return f"#{format_id(sid)}PULR!"


def cmd_torque_off(sid: int) -> str:
    """卸力。断电前必发。"""
    return f"#{format_id(sid)}PULK!"


def cmd_move(sid: int, pwm: int, duration_ms: int) -> str:
    """转角度： (1, 1700, 1000) → '#001P1700T1000!'"""
    if isinstance(pwm, bool) or not isinstance(pwm, int):
        raise TypeError(f"pwm 必须是整数，收到 {pwm!r}")
    if not PWM_ELEC_MIN <= pwm <= PWM_ELEC_MAX:
        raise ValueError(
            f"pwm 超出电气范围 {PWM_ELEC_MIN}~{PWM_ELEC_MAX}，收到 {pwm}"
        )
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int):
        raise TypeError(f"duration_ms 必须是整数，收到 {duration_ms!r}")
    if duration_ms <= 0:
        raise ValueError(f"duration_ms 必须大于 0，收到 {duration_ms}")
    return f"#{format_id(sid)}P{pwm}T{duration_ms}!"


def parse_prad(raw) -> int:
    """解析 PRAD 回显：'#001P1794!' → 1794，解析不出来返回 None。

    兼容串口里混进来的多余字符（回显里可能带其它报文），取最后一段 P+数字。
    """
    if raw is None:
        return None
    try:
        text = raw.decode() if isinstance(raw, (bytes, bytearray)) else str(raw)
    except Exception:
        return None

    digits = ""
    for ch in reversed(text.strip().rstrip("!")):
        if ch.isdigit():
            digits = ch + digits
        elif digits:
            break
    if not digits:
        return None
    try:
        return int(digits)
    except ValueError:
        return None


def smooth_duration_ms(current_pwm, target_pwm, requested_ms,
                       speed_pwm_per_s=600, min_ms=200):
    """算出「从当前位置平滑走到目标」需要的时间，用于抢占后接上而不跳变。

    current_pwm 为 None（读不到位置）时退回 requested_ms。
    结果取 requested_ms 与按限速算出的时间中较大者，避免短时间内跨远距离猛冲。
    """
    requested = max(int(requested_ms), int(min_ms))
    if current_pwm is None:
        return requested
    distance = abs(int(target_pwm) - int(current_pwm))
    speed = max(int(speed_pwm_per_s), 1)
    need_ms = int(distance / speed * 1000)
    return max(requested, need_ms)
