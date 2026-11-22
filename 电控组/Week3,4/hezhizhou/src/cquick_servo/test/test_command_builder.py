from cquick_servo.servo_driver import build_move_cmd


def test_move_cmd_basic():
    assert build_move_cmd(1, 1700, 1000) == '#001P1700T1000!'


def test_padding():
    assert build_move_cmd(10, 900, 500) == '#010P0900T0500!'


def test_boundary():
    assert build_move_cmd(254, 500, 65535) == '#254P0500T65535!'
