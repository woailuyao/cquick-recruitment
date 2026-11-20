"""pytest：只测拼 ASCII 指令的纯函数，不接硬件也能跑。

    colcon test --packages-select cquick_servo
    colcon test-result --verbose
"""

from cquick_servo.protocol import (
    build_keyword_command,
    build_move_command,
    parse_reply,
)


def test_build_move_command_pads_id_to_three_digits():
    assert build_move_command(1, 1700, 1000) == '#001P1700T1000!'
    assert build_move_command(12, 900, 500) == '#012P900T500!'


def test_build_move_command_ends_with_bang():
    command = build_move_command(3, 1500, 800)
    assert command.startswith('#')
    assert command.endswith('!')


def test_build_keyword_command():
    assert build_keyword_command(1, 'PULR') == '#001PULR!'
    assert build_keyword_command(254, 'PULK') == '#254PULK!'
    assert build_keyword_command(2, 'PRAD') == '#002PRAD!'


def test_parse_reply_position():
    assert parse_reply(b'#001P1500!') == (1, 1500)


def test_parse_reply_with_noise():
    assert parse_reply(b'\r\n#007P1723!\r\n') == (7, 1723)


def test_parse_reply_garbage_returns_none():
    assert parse_reply(b'') is None
    assert parse_reply(b'garbage') is None
