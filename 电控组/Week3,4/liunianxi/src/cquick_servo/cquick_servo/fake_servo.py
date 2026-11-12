"""假舵机：接在 socat 造出来的伪终端另一端，没有硬件也能演示 Action。

用法（两个终端）：
  socat -d -d pty,raw,echo=0,link=/tmp/ttySERVO pty,raw,echo=0,link=/tmp/ttyFAKE
  ros2 run cquick_servo fake_servo --ros-args -p port:=/tmp/ttyFAKE

它只实现协议里用得到的那几条：
  #001PID!        -> #001!
  #001PULR!       -> #001PULR!
  #001PULK!       -> #001PULK!
  #001P1500T1000! -> 记录目标与用时，PRAD 按时间线性插值返回
  #001PRAD!       -> #001P<当前pwm>!
回包格式按「回显 ID + 命令」实现，真舵机若有差异改 _reply 一处即可。
"""
from __future__ import annotations

import re
import time

import rclpy
from rclpy.node import Node


MOVE_RE = re.compile(r"#(\d{1,3})P(\d{3,4})T(\d{1,5})!")
# 注意：PULR / PID / PRAD 里的 P 就是命令字的第一个字母，不是额外的分隔符
VERB_RE = re.compile(r"#(\d{1,3})(PID|PRAD|PULR|PULK|PVER)!")


class FakeServo(Node):
    def __init__(self) -> None:
        super().__init__("fake_servo")
        self.declare_parameter("port", "/tmp/ttyFAKE")
        self.declare_parameter("baudrate", 115200)
        self._port = str(self.get_parameter("port").value)
        # id -> (from_pwm, to_pwm, t0, duration_s)
        self._motion: dict[int, tuple[int, int, float, float]] = {}
        self._pwm: dict[int, int] = {}
        self._buffer = ""
        self._link = None
        try:
            import serial  # pyserial

            self._link = serial.Serial(self._port, int(self.get_parameter("baudrate").value),
                                       timeout=0.2)
            self.get_logger().info(f"假舵机挂在 {self._port}")
        except Exception as exc:
            self.get_logger().error(f"打不开 {self._port}：{exc}")
        self.create_timer(0.01, self._poll)

    # ---------------------------------------------------------------- 计算当前位置
    def current_pwm(self, frame_id: int) -> int:
        motion = self._motion.get(frame_id)
        if motion is None:
            return self._pwm.get(frame_id, 1500)
        from_pwm, to_pwm, t0, duration = motion
        elapsed = time.monotonic() - t0
        if duration <= 0 or elapsed >= duration:
            self._pwm[frame_id] = to_pwm
            self._motion.pop(frame_id, None)
            return to_pwm
        ratio = elapsed / duration
        return int(from_pwm + (to_pwm - from_pwm) * ratio)

    # ---------------------------------------------------------------- 收命令
    def _poll(self) -> None:
        if self._link is None:
            return
        try:
            raw = self._link.read(64)
        except Exception as exc:
            self.get_logger().error(f"读串口失败：{exc}")
            return
        if not raw:
            return
        # 串口一次读到的可能是半条指令，先攒进缓冲区，按 '!' 切出完整指令
        self._buffer += raw.decode("ascii", errors="ignore")
        self.get_logger().debug(f"收到原始字节：{raw!r}")
        while "!" in self._buffer:
            command, self._buffer = self._buffer.split("!", 1)
            self._handle(command + "!")
        return

    def _handle(self, text: str) -> None:
        for frame_id, pwm, duration in MOVE_RE.findall(text):
            frame_id = int(frame_id)
            from_pwm = self.current_pwm(frame_id)
            self._motion[frame_id] = (from_pwm, int(pwm), time.monotonic(), int(duration) / 1000.0)
            self.get_logger().info(f"收到动作 #{frame_id:03d}P{int(pwm)}T{int(duration)}!")
        for frame_id, verb in VERB_RE.findall(text):
            frame_id = int(frame_id)
            if verb == "PID":
                self._reply(f"#{frame_id:03d}!")
            elif verb == "PRAD":
                self._reply(f"#{frame_id:03d}P{self.current_pwm(frame_id):04d}!")
            else:
                self._reply(f"#{frame_id:03d}P{verb}!")

    def _reply(self, text: str) -> None:
        if self._link is None:
            return
        try:
            self._link.write(text.encode("ascii"))
        except Exception as exc:
            self.get_logger().error(f"回包失败：{exc}")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = FakeServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._link is not None:
            node._link.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
