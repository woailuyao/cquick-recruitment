from cquick_servo.servo_driver import build_move_cmd


def test_command_format():
    assert build_move_cmd(1, 1700, 1000) == "#001P1700T1000!"


def test_id_padding():
    assert build_move_cmd(10, 900, 0) == "#010P0900T0000!"


def test_pwm_padding():
    assert build_move_cmd(3, 500, 50) == "#003P0500T0050!"
