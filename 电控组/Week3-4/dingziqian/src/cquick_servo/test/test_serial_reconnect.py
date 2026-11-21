"""Serial link recovery without physical hardware."""

import sys
import types

import pytest

from cquick_servo.real_servo import RealServoBus


class FakeSerialException(Exception):
    pass


class FakePort:
    def __init__(self):
        self.is_open = True
        self.fail_write = False
        self.writes = []

    def write(self, data):
        if self.fail_write:
            raise FakeSerialException("USB removed")
        self.writes.append(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def read_until(self, terminator):
        return b"#001P1500!"

    def close(self):
        self.is_open = False


def test_missing_port_reconnects_and_resends_torque(monkeypatch):
    available = {"value": False}
    opened = []

    def open_port(device, baudrate, timeout):
        if not available["value"]:
            raise FakeSerialException("device missing")
        port = FakePort()
        opened.append(port)
        return port

    monkeypatch.setitem(
        sys.modules,
        "serial",
        types.SimpleNamespace(
            Serial=open_port, SerialException=FakeSerialException
        ),
    )
    bus = RealServoBus("/dev/cq_servo", 115200)
    assert not bus.reconnect()
    assert not bus.connected
    available["value"] = True
    assert bus.reconnect()
    bus.move_to(1, 1500, 800)
    assert opened[0].writes[0] == b"#001PULR!\r\n"
    assert opened[0].writes[1] == b"#001P1500T0800!\r\n"

    opened[0].fail_write = True
    with pytest.raises(ConnectionError):
        bus.move_to(1, 1500, 800)
    assert not bus.connected
    assert not bus.torque_enabled_ids

    assert bus.reconnect()
    bus.move_to(1, 1500, 800)
    assert opened[1].writes[0] == b"#001PULR!\r\n"
    bus.close()


def test_idle_device_disappearance_clears_connection(monkeypatch):
    port = FakePort()
    monkeypatch.setitem(
        sys.modules,
        "serial",
        types.SimpleNamespace(
            Serial=lambda *args, **kwargs: port,
            SerialException=FakeSerialException,
        ),
    )
    bus = RealServoBus("/dev/cq_servo", 115200)
    assert bus.reconnect()
    monkeypatch.setattr(
        "cquick_servo.real_servo.os.path.exists", lambda path: False
    )
    assert not bus.refresh_connection()
    assert not port.is_open
    assert bus.last_error == "设备节点已消失"
