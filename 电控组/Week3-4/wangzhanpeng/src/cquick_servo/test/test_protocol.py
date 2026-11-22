# -*- coding: utf-8 -*-
from cquick_servo.protocol import (
    move_command,
    parse_prad,
    prad_command,
    pulk_command,
    pulr_command,
)


def test_move_command_pads_id_to_three_digits():
    assert move_command(1, 1700, 1000) == '#001P1700T1000!'
    assert move_command(12, 900, 1200) == '#012P900T1200!'


def test_move_command_keeps_full_range():
    assert move_command(4, 2100, 1000) == '#004P2100T1000!'
    assert move_command(4, 900, 1000) == '#004P900T1000!'


def test_status_commands():
    assert prad_command(1) == '#001PRAD!'
    assert pulr_command(2) == '#002PULR!'
    assert pulk_command(3) == '#003PULK!'


def test_parse_prad():
    assert parse_prad(b'#001P1495!') == 1495
    assert parse_prad(b'#012P900!') == 900
    assert parse_prad(b'') is None
    assert parse_prad(b'garbage') is None
