import os
import re


def build_move_command(servo_id, target_pwm, duration_ms):
    """Build the servo protocol command without opening a serial port."""
    return f"#{servo_id:03d}P{target_pwm:04d}T{duration_ms:04d}!"


class RealServoBus:
    def __init__(self, port, baudrate):
        import serial

        self.serial = serial
        self.device = port
        self.baudrate = baudrate
        self.port = None
        self.last_error = None
        self.torque_enabled_ids = set()

    @property
    def connected(self):
        return self.port is not None and self.port.is_open

    def refresh_connection(self):
        """Notice when udev removes the configured device while idle."""
        if self.connected and not os.path.exists(self.device):
            self._disconnect("设备节点已消失")
        return self.connected

    def _disconnect(self, error=None):
        old_port = self.port
        self.port = None
        self.torque_enabled_ids.clear()
        if error is not None:
            self.last_error = str(error)
        if old_port is not None:
            try:
                old_port.close()
            except (self.serial.SerialException, OSError):
                pass

    def reconnect(self):
        """Try once to reopen the configured device; never move a servo."""
        if self.connected:
            return True
        self._disconnect()
        try:
            self.port = self.serial.Serial(
                self.device, self.baudrate, timeout=2
            )
        except (self.serial.SerialException, OSError) as exc:
            self.last_error = str(exc)
            return False
        self.last_error = None
        self.torque_enabled_ids.clear()
        return True

    def _require_port(self):
        if not self.connected:
            raise ConnectionError(f"串口 {self.device} 未连接")
        return self.port

    def _write(self, command):
        port = self._require_port()
        try:
            port.write((command + "\r\n").encode("ascii"))
            port.flush()
        except (self.serial.SerialException, OSError) as exc:
            self._disconnect(exc)
            raise ConnectionError(f"串口 {self.device} 发送失败：{exc}") from exc

    def enable_torque(self, servo_id):
        if servo_id not in self.torque_enabled_ids:
            self._write(f"#{servo_id:03d}PULR!")
            self.torque_enabled_ids.add(servo_id)

    def disable_torque(self, servo_id):
        self._write(f"#{servo_id:03d}PULK!")
        self.torque_enabled_ids.discard(servo_id)

    def read_position(self, servo_id):
        port = self._require_port()
        try:
            port.reset_input_buffer()
            port.write(f"#{servo_id:03d}PRAD!\r\n".encode("ascii"))
            port.flush()
            reply = port.read_until(b"!")
        except (self.serial.SerialException, OSError) as exc:
            self._disconnect(exc)
            raise ConnectionError(f"串口 {self.device} 读取失败：{exc}") from exc
        if not reply:
            self._disconnect("PRAD 读取超时")
            raise ConnectionError(f"串口 {self.device} PRAD 读取超时")
        pattern = rb"#" + f"{servo_id:03d}".encode("ascii") + rb"P(\d+)!"
        match = re.fullmatch(pattern, reply)
        if match is None:
            raise RuntimeError(f"舵机 {servo_id} 位置回复无效：{reply!r}")
        return int(match.group(1))

    def move_to(self, servo_id, target_pwm, duration_ms):
        self.enable_torque(servo_id)
        self._write(build_move_command(servo_id, target_pwm, duration_ms))

    def close(self):
        self._disconnect()
