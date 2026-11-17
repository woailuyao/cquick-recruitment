"""总线舵机指令的拼装与解析。

这里只放纯函数：不 import rclpy、不碰串口，
所以能脱离 ROS 环境和硬件直接跑 pytest。
"""


def build_move_cmd(servo_id, pwm, duration_ms):
    """位置指令: (1, 1700, 1000) -> '#001P1700T1000!'"""
    return f'#{int(servo_id):03d}P{int(pwm)}T{int(duration_ms)}!'


def build_pulr_cmd(servo_id):
    """恢复扭力: (1) -> '#001PULR!'"""
    return f'#{int(servo_id):03d}PULR!'


def build_pulk_cmd(servo_id):
    """卸力: (1) -> '#001PULK!'"""
    return f'#{int(servo_id):03d}PULK!'


def build_prad_cmd(servo_id):
    """读取位置: (1) -> '#001PRAD!'"""
    return f'#{int(servo_id):03d}PRAD!'


def parse_prad(resp):
    """从 PRAD 回显里抠出位置数字，抠不到返回 None。

    b'#001P1500!' -> 1500 ; b'' -> None ; b'#001PULR!' -> None
    """
    if not resp:
        return None
    text = resp.decode(errors='ignore')
    idx = text.find('P')
    if idx < 0:
        return None
    digits = ''
    for ch in text[idx + 1:]:
        if ch.isdigit():
            digits += ch
        else:
            break
    return int(digits) if digits else None
