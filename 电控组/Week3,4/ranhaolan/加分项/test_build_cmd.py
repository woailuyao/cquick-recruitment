from cquick_servo.servo_driver import ServoDriver


def test_build_cmd_basic():
    assert ServoDriver.build_cmd(1, 'P1700T1000') == '#001P1700T1000!'


def test_build_cmd_zero_pad():
    assert ServoDriver.build_cmd(12, 'P1500T800') == '#012P1500T800!'


def test_build_cmd_pulk():
    assert ServoDriver.build_cmd(3, 'PULK') == '#003PULK!'