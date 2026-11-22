#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ASCII protocol helpers for the CQuick bus servo.

Frame format: #<id:03d><body>!
    move_command(1, 1700, 1000) -> '#001P1700T1000!'
    prad_command(1)             -> '#001PRAD!'
"""

import re

HEAD = '#'
TAIL = '!'
ID_WIDTH = 3


def _frame(servo_id, body):
    return f'{HEAD}{int(servo_id):0{ID_WIDTH}d}{body}{TAIL}'


def move_command(servo_id, pwm, duration_ms):
    return _frame(servo_id, f'P{int(pwm)}T{int(duration_ms)}')


def prad_command(servo_id):
    return _frame(servo_id, 'PRAD')


def pulr_command(servo_id):
    return _frame(servo_id, 'PULR')


def pulk_command(servo_id):
    return _frame(servo_id, 'PULK')


def parse_prad(reply):
    if not reply:
        return None
    match = re.search(rb'P(\d+)', reply)
    return int(match.group(1)) if match else None
