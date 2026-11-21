from cquick_servo.real_servo import build_move_command


def test_move_command_formats_one_digit_id():
    assert build_move_command(1, 1700, 1000) == "#001P1700T1000!"


def test_move_command_formats_other_values():
    assert build_move_command(23, 900, 800) == "#023P0900T0800!"
