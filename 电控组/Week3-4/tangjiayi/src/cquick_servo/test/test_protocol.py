#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""协议层的单元测试（不用接硬件，也不用起节点）。

跑法：
    cd ~/ros2_ws && colcon test --packages-select cquick_servo && colcon test-result --verbose
或者单独跑：
    python3 -m pytest src/cquick_servo/test/test_protocol.py -v
"""

import pytest

from cquick_servo import servo_protocol as proto


class TestBuildMoveCommand:
    """拼「转角度」指令 —— 细则要求的那个纯函数。"""

    def test_basic(self):
        # 细则里的例子：#1,1700,1000 -> #001P1700T1000!
        assert proto.build_move_command(1, 1700, 1000) == b'#001P1700T1000!'

    def test_id_is_zero_padded_to_three_digits(self):
        assert proto.build_move_command(7, 1500, 500) == b'#007P1500T500!'
        assert proto.build_move_command(12, 1500, 500) == b'#012P1500T500!'
        assert proto.build_move_command(123, 1500, 500) == b'#123P1500T500!'

    def test_frame_head_and_tail(self):
        frame = proto.build_move_command(3, 900, 200)
        assert frame.startswith(b'#')
        assert frame.endswith(b'!')
        assert frame.count(b'#') == 1
        assert frame.count(b'!') == 1

    def test_is_ascii_bytes(self):
        # 串口写的是字节，不是 str；写 str 会 TypeError
        assert isinstance(proto.build_move_command(1, 1500, 1000), bytes)

    def test_rejects_illegal_id(self):
        for bad in (0, -1, 255):
            with pytest.raises(ValueError):
                proto.build_move_command(bad, 1500, 1000)

    def test_rejects_negative_values(self):
        with pytest.raises(ValueError):
            proto.build_move_command(1, -1, 1000)
        with pytest.raises(ValueError):
            proto.build_move_command(1, 1500, -1)


class TestSimpleCommands:
    def test_torque_on(self):
        assert proto.build_torque_on(1) == b'#001PULR!'

    def test_torque_off(self):
        assert proto.build_torque_off(6) == b'#006PULK!'

    def test_read_position(self):
        assert proto.build_read_position(2) == b'#002PRAD!'

    def test_read_id(self):
        assert proto.build_read_id(1) == b'#001PID!'


class TestParseFrames:
    def test_single_complete_frame(self):
        frames, rest = proto.parse_frames(b'#001P1500!')
        assert frames == ['#001P1500!']
        assert rest == b''

    def test_half_frame_is_kept(self):
        # 串口是流式的，一次 read 可能只有半帧，残余不能丢
        frames, rest = proto.parse_frames(b'#001P15')
        assert frames == []
        assert rest == b'#001P15'

    def test_two_frames_in_one_read(self):
        frames, rest = proto.parse_frames(b'#001P1500!#002P1600!')
        assert frames == ['#001P1500!', '#002P1600!']
        assert rest == b''

    def test_garbage_before_head_is_dropped(self):
        frames, rest = proto.parse_frames(b'\x00\xff#001P1500!')
        assert frames == ['#001P1500!']
        assert rest == b''


class TestParseReply:
    def test_position_reply(self):
        reply = proto.parse_reply('#001P1500!')
        assert reply is not None
        assert (reply.servo_id, reply.command, reply.value) == (1, 'P', 1500)

    def test_prad_reply(self):
        reply = proto.parse_reply('#003PRAD1700!')
        assert reply is not None
        assert (reply.servo_id, reply.command, reply.value) == (3, 'PRAD', 1700)

    def test_id_reply_without_number(self):
        reply = proto.parse_reply('#001PID!')
        assert reply is not None
        assert reply.command == 'PID'
        assert reply.value is None

    def test_own_move_command_echo_is_not_a_reply(self):
        # 自己发出去的 #001P1500T1000! 不该被当成位置回帧
        assert proto.parse_reply('#001P1500T1000!') is None

    def test_noise_is_ignored(self):
        for junk in ('', 'abc', '#', '!', '#abc!'):
            assert proto.parse_reply(junk) is None
