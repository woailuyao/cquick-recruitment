"""假舵机：接在 socat 造出来的伪终端另一端，没有硬件也能演示 Action。

用法（两个终端）：
  socat -d -d pty,raw,echo=0,link=/tmp/ttySERVO pty,raw,echo=0,link=/tmp/ttyFAKE
  ros2 run cquick_servo fake_servo --ros-args -p port:=/tmp/ttyFAKE

它只实现协议里用得到的那几条（与真机 ZL-IS2 的实测回包一致）：
  #001PID!        -> #001P!
  #001PULR!       -> （不回包，静默上力）
  #001PULK!       -> （不回包，静默卸力）
  #001P1500T1000! -> 记录目标与用时，PRAD 按时间线性插值返回
  #001PRAD!       -> #001P<当前pwm>!
回包格式按真机实测实现，真舵机若有差异改 _handle 一处即可。

present_ids：板子上真实存在的舵机编号（默认 1~10）。不在这个列表里的 ID
（比如空通道 11~23）**一律不回包**，与真机"空通道没人应答"的行为一致——
否则扫描 0~23 会得到 24 个假应答，和硬件现场对不上。
"""
from __future__ import annotations

import re
import time

import rclpy
from rclpy.node import Node


# 动作帧正则：#ID P pwm T ms !（ID 1~3 位、pwm 3~4 位、时长 1~5 位）。
MOVE_RE = re.compile(r"#(\d{1,3})P(\d{3,4})T(\d{1,5})!")
# 注意：PULR / PID / PRAD 里的 P 就是命令字的第一个字母，不是额外的分隔符
# 命令字帧正则：P 之后的整段就是命令字（PID/PRAD/PULR/PULK），不再额外补 P。
VERB_RE = re.compile(r"#(\d{1,3})(PID|PRAD|PULR|PULK|PVER)!")


class FakeServo(Node):
    def __init__(self) -> None:
        super().__init__("fake_servo")
        # 假舵机挂在 socat 伪终端的“另一端”，默认 /tmp/ttyFAKE。
        self.declare_parameter("port", "/tmp/ttyFAKE")
        self.declare_parameter("baudrate", 115200)
        # 演示/验证用：>0 时每个 ID 的第一次动作故意少走这么多 pwm，
        # 这样驱动会判"未到位"并自动重发一次（加分项②的验证手段）。平时是 0。
        # 故障注入：>0 时每个 ID 的第一次动作故意少走，用来演示“未到位 -> 自动重发”。
        self.declare_parameter("first_move_short", 0)
        # 板子上真实存在的舵机编号；其它 ID 当作空通道，不回包（与真机一致）
        # 模拟“板子上真实装了哪些舵机”；不在表里的 ID 一律静默，否则扫描 0~23 会得到 24 个假应答。
        self.declare_parameter("present_ids", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self._tested: set[int] = set()
        self._port = str(self.get_parameter("port").value)
        # id -> (from_pwm, to_pwm, t0, duration_s)
        # 每个 ID 的运动状态：(起点, 目标, 起始时刻, 总时长)，用线性插值模拟真实舵机走位。
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
        # 100 Hz 轮询串口：伪终端不会主动通知“有新数据”，只能主动 poll。
        self.create_timer(0.01, self._poll)

    # ---------------------------------------------------------------- 计算当前位置
    # 计算“此刻的位置”：没有运动记录就返回最后位置；运动中的话按时间比例插值。
    def current_pwm(self, frame_id: int) -> int:
        motion = self._motion.get(frame_id)
        if motion is None:
            return self._pwm.get(frame_id, 1500)
        from_pwm, to_pwm, t0, duration = motion
        elapsed = time.monotonic() - t0
        # 到点即锁死在目标值，并删掉运动记录（之后每次读都是静态值）。
        if duration <= 0 or elapsed >= duration:
            self._pwm[frame_id] = to_pwm
            self._motion.pop(frame_id, None)
            return to_pwm
        ratio = elapsed / duration
        return int(from_pwm + (to_pwm - from_pwm) * ratio)

    # ---------------------------------------------------------------- 收命令
    # 每次现读参数：现场 `ros2 param set /fake_servo present_ids` 就能改“在线舵机数”。
    def _present(self) -> set[int]:
        """板子上真实存在的舵机编号（每次现读参数，方便 param set 现场改）。"""
        return {int(v) for v in self.get_parameter("present_ids").value}

    def _poll(self) -> None:
        if self._link is None:
            return
        try:
            # 一次最多读 64 字节：读取可能只拿到半条指令，所以下面要攒缓冲。
            raw = self._link.read(64)
        except Exception as exc:
            self.get_logger().error(f"读串口失败：{exc}")
            return
        if not raw:
            return
        # 串口一次读到的可能是半条指令，先攒进缓冲区，按 '!' 切出完整指令
        # 攒缓冲区：串口是字节流，必须自己按 ! 切帧，不能假设一次读完一条。
        self._buffer += raw.decode("ascii", errors="ignore")
        self.get_logger().debug(f"收到原始字节：{raw!r}")
        # 一次可能读到多条帧，循环切到没有 ! 为止。
        while "!" in self._buffer:
            command, self._buffer = self._buffer.split("!", 1)
            self._handle(command + "!")
        return

    def _handle(self, text: str) -> None:
        present = self._present()
        for frame_id, pwm, duration in MOVE_RE.findall(text):
            frame_id = int(frame_id)
            if frame_id not in present:
                # 空通道静默：真机空通道没人应答，仿真必须与之一致。
                continue  # 空通道：真机上没人应答，这里也不理
            from_pwm = self.current_pwm(frame_id)
            target = int(pwm)
            short = int(self.get_parameter("first_move_short").value)
            if short > 0 and frame_id not in self._tested:
                self._tested.add(frame_id)
                target = target - short if target >= from_pwm else target + short
                self.get_logger().warn(
                    f"演示注入：id={frame_id} 第一次少走 {short} -> 目标 {target}"
                )
            # 登记运动：起点取当前值、终点是目标、时长毫秒转秒。
            self._motion[frame_id] = (from_pwm, target, time.monotonic(), int(duration) / 1000.0)
            self.get_logger().info(f"收到动作 #{frame_id:03d}P{int(pwm)}T{int(duration)}!")
            # 真机收到动作指令会回一条 #IDP<目标>!（当日真机截图里 1700 动作回 #003P1693! 是走完后的位置，
            # 这里回目标值，语义一致：表示“这条动作我收到了”）
            self._reply(f"#{frame_id:03d}P{int(pwm):04d}!")
        for frame_id, verb in VERB_RE.findall(text):
            frame_id = int(frame_id)
            if frame_id not in present:
                continue  # 空通道：静默
            # PID 回 `#003P!`（只有 ID + P），与真机实测一致；PULR/PULK 则完全静默。
            if verb == "PID":
                # 真机回的是 #003P!（只有 ID + P，没有 "ID" 两个字）
                self._reply(f"#{frame_id:03d}P!")
            elif verb == "PRAD":
                # PRAD 回包：4 位补零，和真机 `#003P1500!` 一致。
                self._reply(f"#{frame_id:03d}P{self.current_pwm(frame_id):04d}!")
            else:
                # PULR / PULK：真机不回声（静默执行）
                pass

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
