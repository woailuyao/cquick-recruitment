# -*- coding: utf-8 -*-
"""core.py 的单测：用假串口 + 假时钟，不接硬件也能验证全部关键逻辑。

覆盖：协议下发顺序 / 限位夹取 / 运动估算 / PRAD 闭环重发 / 串口重连 /
      参数热更新重开串口 / 看门狗卸力与安全模式。
"""

import time

from cquick_servo.core import CoreConfig, ServoCore


class FakeSerial:
    """假串口：记录写出的帧，按队列吐出预置回显。"""

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.is_open = True
        self.written = []
        self.replies = []
        self.fail_writes = 0

    def write(self, data):
        if self.fail_writes > 0:
            self.fail_writes -= 1
            raise OSError('simulated write failure')
        self.written.append(data.decode('ascii'))

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def read(self, size):
        return self.replies.pop(0) if self.replies else b''

    def close(self):
        self.is_open = False


class SerialFactory:
    def __init__(self):
        self.created = []
        self.fail_open = 0

    def __call__(self, **kwargs):
        if self.fail_open > 0:
            self.fail_open -= 1
            raise OSError('no such device')
        serial_port = FakeSerial(**kwargs)
        self.created.append(serial_port)
        return serial_port


class FakeTime:
    def __init__(self):
        self.now = 1000.0

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class RecordingLogger:
    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(('info', message))

    def warning(self, message):
        self.lines.append(('warning', message))

    def error(self, message):
        self.lines.append(('error', message))

    def text(self):
        return '\n'.join(message for _, message in self.lines)


def make_config(**overrides):
    values = dict(
        port='/dev/fake',
        baudrate=115200,
        pwm_min=900,
        pwm_max=2100,
        default_duration_ms=800,
        servo_ids=(1, 2, 3),
        position_tolerance=30,
        watchdog_timeout_s=0.0,      # 大多数用例不需要看门狗
    )
    values.update(overrides)
    return CoreConfig(**values)


def make_core(**overrides):
    clock = FakeTime()
    factory = SerialFactory()
    logger = RecordingLogger()
    core = ServoCore(
        make_config(**overrides),
        logger=logger,
        serial_factory=factory,
        clock=clock.clock,
        sleep=clock.sleep,
    )
    return core, clock, factory, logger


# ==================== 下发 ====================

def test_move_sends_pulr_then_position():
    core, _, factory, _ = make_core()
    assert core.move(1, 1700, 1000) is True
    assert factory.created[0].written == ['#001PULR!', '#001P1700T1000!']


def test_serial_opened_with_8n1_and_config():
    core, _, factory, _ = make_core(port='/dev/ttyUSB9', baudrate=57600)
    core.move(1, 1500, 100)
    kwargs = factory.created[0].kwargs
    assert kwargs['port'] == '/dev/ttyUSB9'
    assert kwargs['baudrate'] == 57600
    assert kwargs['bytesize'] == 8
    assert kwargs['parity'] == 'N'
    assert kwargs['stopbits'] == 1


def test_move_rejects_unknown_id_without_touching_serial():
    core, _, factory, logger = make_core()
    assert core.move(9, 1500, 100) is False
    assert factory.created == []
    assert '不在 servo_ids' in logger.text()


def test_move_clamps_out_of_range_pwm():
    core, _, factory, logger = make_core()
    assert core.move(1, 3000, 500) is True
    assert factory.created[0].written[-1] == '#001P2100T500!'
    assert '夹到 2100' in logger.text()


def test_move_uses_default_duration_when_zero():
    core, _, factory, _ = make_core(default_duration_ms=650)
    core.move(1, 1500, 0)
    assert factory.created[0].written[-1] == '#001P1500T650!'


# ==================== 位置估算 ====================

def test_estimated_pwm_interpolates_then_holds():
    core, clock, _, _ = make_core()
    core.move(1, 1700, 1000)          # 起点按中位 1500 算
    clock.now += 0.5
    assert core.estimated_pwm(1) == 1600
    clock.now += 0.6
    assert core.estimated_pwm(1) == 1700


# ==================== PRAD 闭环 ====================

def test_verify_ok():
    core, _, factory, _ = make_core()
    core.move(1, 1700, 1000)
    factory.created[0].replies.append(b'#001P1700!')
    result = core.verify(1, 1700, 1000)
    assert result.ok is True
    assert result.diff == 0
    assert result.retried is False


def test_verify_retries_once_then_ok():
    core, _, factory, logger = make_core()
    core.move(1, 1700, 1000)
    serial_port = factory.created[0]
    serial_port.replies.extend([b'#001P1600!', b'#001P1690!'])
    result = core.verify(1, 1700, 1000)
    assert result.retried is True
    assert result.ok is True
    assert result.final_pwm == 1690
    assert serial_port.written.count('#001P1700T1000!') == 2   # 原发一次 + 重发一次
    assert '未到位' in logger.text()


def test_verify_reports_failure_without_reply():
    core, _, _, _ = make_core()
    core.move(1, 1700, 1000)
    result = core.verify(1, 1700, 1000)
    assert result.ok is False
    assert result.final_pwm is None


# ==================== 掉线重连 / 参数重开 ====================

def test_reconnect_after_write_failure():
    core, clock, factory, logger = make_core()
    assert core.move(1, 1500, 100) is True
    factory.created[0].fail_writes = 1
    assert core.move(1, 1500, 100) is False       # PULR 写失败 -> 自动关串口
    clock.now += 1.1                               # 越过 1 秒重连间隔
    assert core.move(1, 1500, 100) is True
    assert len(factory.created) == 2               # 真的重新打开了
    assert '串口已关闭' in logger.text()


def test_update_config_reopens_new_port_immediately():
    core, _, factory, _ = make_core()
    core.move(1, 1500, 100)
    core.update_config(port='/dev/other')
    core.move(1, 1500, 100)                        # 不需要等 1 秒
    assert len(factory.created) == 2
    assert factory.created[-1].kwargs['port'] == '/dev/other'


# ==================== 状态回调 ====================

def test_state_callback_receives_move_event():
    core, _, _, _ = make_core()
    events = []
    core.on_state = events.append
    core.move(2, 1600, 500)
    assert events and events[0]['event'] == 'move'
    assert events[0]['id'] == 2
    assert events[0]['pwm'] == 1600


# ==================== 看门狗 / 安全模式 ====================

def test_trigger_fault_pulks_and_blocks_move():
    core, _, factory, _ = make_core()
    core.trigger_fault('unit test')
    assert core.fault is True
    assert core.fault_reason == 'unit test'
    assert '#001PULK!' in factory.created[0].written
    assert core.move(1, 1500, 100) is False
    ok, message = core.recover()
    assert ok is True and message == 'recovered'
    assert core.move(1, 1500, 100) is True


def test_recover_without_fault_returns_false():
    core, _, _, _ = make_core()
    ok, message = core.recover()
    assert ok is False and message == 'not in safe mode'


def test_watchdog_fires_when_heartbeat_missing():
    factory = SerialFactory()
    logger = RecordingLogger()
    core = ServoCore(make_config(watchdog_timeout_s=0.2), logger=logger,
                     serial_factory=factory)
    core.start_watchdog()
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not core.fault:
            time.sleep(0.02)
        assert core.fault is True
        assert '#001PULK!' in factory.created[0].written
        assert 'WATCHDOG' in logger.text()
        assert core.move(1, 1500, 100) is False
    finally:
        core.stop_watchdog()
    assert core.recover()[0] is True


def test_heartbeat_keeps_watchdog_quiet():
    core = ServoCore(make_config(watchdog_timeout_s=0.2),
                     serial_factory=SerialFactory())
    core.start_watchdog()
    try:
        for _ in range(6):
            core.heartbeat()
            time.sleep(0.08)
        assert core.fault is False
    finally:
        core.stop_watchdog()
