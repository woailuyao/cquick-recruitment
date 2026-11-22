"""纯函数单测：不依赖 rclpy、不依赖串口、不依赖任何硬件"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cquick_servo.commands import (  # noqa: E402
    build_move_cmd,
    build_pulr_cmd,
    build_pulk_cmd,
    build_prad_cmd,
    parse_prad,
)


def test_move_cmd_basic():
    assert build_move_cmd(1, 1700, 1000) == '#001P1700T1000!'


def test_move_cmd_id_padding():
    assert build_move_cmd(1, 1500, 800) == '#001P1500T800!'
    assert build_move_cmd(12, 1500, 800) == '#012P1500T800!'
    assert build_move_cmd(123, 1500, 800) == '#123P1500T800!'


def test_move_cmd_string_input():
    assert build_move_cmd('1', '1700', '1000') == '#001P1700T1000!'


def test_pulr():
    assert build_pulr_cmd(1) == '#001PULR!'


def test_pulk():
    assert build_pulk_cmd(6) == '#006PULK!'


def test_prad_cmd():
    assert build_prad_cmd(2) == '#002PRAD!'


def test_parse_prad_ok():
    assert parse_prad(b'#001P1500!') == 1500
    assert parse_prad(b'#001P2100!') == 2100


def test_parse_prad_empty():
    assert parse_prad(b'') is None


def test_parse_prad_garbage():
    assert parse_prad(b'\xff\x00garbage') is None


def test_parse_prad_no_digits():
    assert parse_prad(b'#001PULR!') is None
