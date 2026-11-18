"""build_cmd 的单元测试。

为什么测这个函数：它是整条链路里唯一把“数值”翻译成“协议字符串”的地方，
补零写错一位，舵机就完全不动，而且现象和“串口坏了”一模一样，极难排查。
"""
from cquick_servo.servo_driver import build_cmd


def test_normal():
    """最普通的一条：1 号舵机、PWM 1700、1000ms。"""
    assert build_cmd(1, 1700, 1000) == '#001P1700T1000!'


def test_id_padding():
    """ID 必须补到 3 位：1 -> 001。"""
    assert build_cmd(1, 1500, 800) == '#001P1500T0800!'
    assert build_cmd(254, 1500, 800) == '#254P1500T0800!'


def test_pwm_padding():
    """PWM 必须补到 4 位：900 -> 0900。"""
    assert build_cmd(2, 900, 1000) == '#002P0900T1000!'
    assert build_cmd(2, 500, 1000) == '#002P0500T1000!'


def test_time_padding():
    """时间必须补到 4 位：0 -> 0000，800 -> 0800。"""
    assert build_cmd(1, 1500, 0) == '#001P1500T0000!'
    assert build_cmd(1, 1500, 9999) == '#001P1500T9999!'


def test_boundary():
    """边界：最小 ID、最小 PWM、最大时间。"""
    assert build_cmd(0, 500, 9999) == '#000P0500T9999!'
