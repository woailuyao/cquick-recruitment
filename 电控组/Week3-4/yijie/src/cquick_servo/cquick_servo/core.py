# -*- coding: utf-8 -*-
"""core.py —— 舵机核心逻辑：串口、运动模型、PRAD 闭环、看门狗（不依赖 ROS）。

为什么单独一层：
  * servo_driver.py 只负责「ROS 参数 / Action / 话题」，逻辑留在这一层好讲也好测；
  * 不 import rclpy，所以 pytest 能在没有 ROS 的机器上跑（加分项 pytest 就靠它）；
  * 串口通过 `serial_factory` 注入，单测里用假串口，完全不接硬件。

安全约定：
  * 任何「配置」都来自 CoreConfig（由 ROS 参数填），本文件没有端口/波特率/限位的默认值；
  * 触发看门狗后进入安全模式：所有动作拒绝，只有 recover() 能解除。
"""

import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional

try:                        # 板上由 python3-serial 提供；离线自测允许缺失
    import serial
except ImportError:         # pragma: no cover
    serial = None

from .protocol import (CENTER_PWM, build_move, build_prad, build_pulk,
                       build_pulr, clamp_pwm, is_valid_id, parse_pwm_reply)


class NullLogger:
    """默认日志器：单测里不接 ROS 时用它，什么都不打印。"""

    def info(self, message):
        pass

    def warning(self, message):
        pass

    def error(self, message):
        pass


# 控制板固件支持的标准波特率（硬件能力表，不是配置默认值）。
# 参数校验用它拒绝「不认识的波特率」，默认值仍然只出现在节点的 declare_parameter 里。
SUPPORTED_BAUDRATES = (9600, 19200, 38400, 57600, 115200, 230400)


def default_serial_factory(**kwargs):
    """真正的串口工厂。参数全部由 CoreConfig 传进来。"""
    if serial is None:
        raise RuntimeError('未安装 pyserial：sudo apt install -y python3-serial')
    return serial.Serial(**kwargs)


@dataclass
class CoreConfig:
    """运行期配置。字段全部来自 ROS 参数，因此这里不设默认值。"""

    port: str
    baudrate: int
    pwm_min: int
    pwm_max: int
    default_duration_ms: int
    servo_ids: tuple
    position_tolerance: int
    watchdog_timeout_s: float


@dataclass
class Motion:
    """一次位置指令的运动模型，用来估算「现在大概在哪」。"""

    start_pwm: int
    target_pwm: int
    duration_s: float
    started_at: float

    def pwm_at(self, now: float) -> int:
        if self.duration_s <= 0:
            return self.target_pwm
        progress = (now - self.started_at) / self.duration_s
        if progress <= 0:
            return self.start_pwm
        if progress >= 1:
            return self.target_pwm
        return int(round(self.start_pwm + (self.target_pwm - self.start_pwm) * progress))


@dataclass
class VerifyResult:
    ok: bool
    final_pwm: Optional[int]
    retried: bool
    diff: Optional[int]


class ServoCore:
    """串口 + 运动 + 校验 + 看门狗。线程安全（Action 线程和看门狗线程会同时用）。"""

    FEEDBACK_PERIOD_S = 0.1      # 10 Hz，考核要求 ≥5 Hz
    RECONNECT_INTERVAL_S = 1.0   # 掉线后最快 1 秒重试一次，别把 CPU 打满
    READ_TIMEOUT_S = 0.05        # 单次 read 的阻塞上限
    REPLY_TIMEOUT_S = 0.25       # 等一条回显的总超时
    WATCHDOG_POLL_S = 0.05       # 看门狗检查间隔

    def __init__(self, config: CoreConfig, logger=None,
                 serial_factory: Optional[Callable[..., object]] = None,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep):
        self._config = config
        self._logger = logger or NullLogger()
        self._serial_factory = serial_factory or default_serial_factory
        self._clock = clock
        self._sleep = sleep

        self._io_lock = threading.RLock()      # 串口是共享资源
        self._state_lock = threading.Lock()    # fault / motion / last_known
        self._config_lock = threading.Lock()

        self._ser = None
        self._last_open_attempt = float('-inf')
        self._reopen_requested = False

        self._motions = {}          # servo_id -> Motion
        self._last_known = {}       # servo_id -> 最近一次 PRAD 确认的位置

        self._fault = False
        self._fault_reason = ''
        self._last_beat = clock()

        self._watchdog_thread = None
        self._watchdog_stop = threading.Event()

        self.on_state: Optional[Callable[[dict], None]] = None

    # ==================== 只读属性 ====================

    @property
    def config(self) -> CoreConfig:
        return self._config

    @property
    def fault(self) -> bool:
        return self._fault

    @property
    def fault_reason(self) -> str:
        return self._fault_reason

    @property
    def is_open(self) -> bool:
        return self._ser is not None and getattr(self._ser, 'is_open', True)

    # ==================== 配置热更新 ====================

    def update_config(self, **changes) -> CoreConfig:
        """参数回调校验通过后调用；端口/波特率变了就标记重开串口。"""
        with self._config_lock:
            old = self._config
            new = replace(old, **changes) if changes else old
            self._config = new
        if new.port != old.port or new.baudrate != old.baudrate:
            with self._io_lock:
                self._reopen_requested = True
        return new

    # ==================== 串口 ====================

    def open(self) -> bool:
        with self._io_lock:
            return self._ensure_serial_locked()

    def close(self) -> None:
        with self._io_lock:
            self._close_serial_locked()

    def _ensure_serial_locked(self) -> bool:
        forced = False
        if self._reopen_requested:
            self._close_serial_locked('配置变更，重开串口')
            self._reopen_requested = False
            forced = True            # 显式要求重开时跳过 1 秒重连间隔

        if self._ser is not None and getattr(self._ser, 'is_open', True):
            return True

        now = self._clock()
        if not forced and now - self._last_open_attempt < self.RECONNECT_INTERVAL_S:
            return False
        self._last_open_attempt = now

        cfg = self._config
        try:
            self._ser = self._serial_factory(
                port=cfg.port,
                baudrate=cfg.baudrate,
                bytesize=8,            # 8 数据位
                parity='N',            # 无校验
                stopbits=1,            # 1 停止位
                timeout=self.READ_TIMEOUT_S,
                write_timeout=0.2,
            )
            self._logger.info(f'串口已打开：{cfg.port} @ {cfg.baudrate} 8-N-1')
            return True
        except Exception as exc:       # pyserial 的 SerialException / OSError / 参数错误
            self._ser = None
            self._logger.warning(
                f'串口打不开 {cfg.port}：{exc}（{self.RECONNECT_INTERVAL_S:.1f} 秒后自动重试）')
            return False

    def _close_serial_locked(self, why: str = '') -> None:
        if self._ser is None:
            return
        try:
            self._ser.close()
        except Exception:
            pass
        self._ser = None
        if why:
            self._logger.warning(f'串口已关闭：{why}')

    # ==================== 收发 ====================

    def _send(self, command: str) -> bool:
        """只写不读。写失败就关串口，下一次调用会自动重连。"""
        with self._io_lock:
            if not self._ensure_serial_locked():
                return False
            try:
                self._ser.write(command.encode('ascii'))
                self._ser.flush()      # 不 flush 会卡在用户态缓冲区，舵机收不到
                return True
            except Exception as exc:
                self._close_serial_locked(f'写失败：{exc}')
                return False

    def _query(self, command: str, timeout: float = REPLY_TIMEOUT_S) -> bytes:
        """写一条需要回显的指令，读到 '!' 或超时。"""
        with self._io_lock:
            if not self._ensure_serial_locked():
                return b''
            try:
                self._ser.reset_input_buffer()      # 丢掉上一条的残留
                self._ser.write(command.encode('ascii'))
                self._ser.flush()
                deadline = self._clock() + timeout
                buffer = b''
                while self._clock() < deadline:
                    chunk = self._ser.read(64)
                    if chunk:
                        buffer += chunk
                        if buffer.endswith(b'!'):
                            break
                    else:
                        self._sleep(0.002)
                return buffer
            except Exception as exc:
                self._close_serial_locked(f'读写失败：{exc}')
                return b''

    # ==================== 位置 ====================

    def center_pwm(self) -> int:
        cfg = self._config
        return clamp_pwm(CENTER_PWM, cfg.pwm_min, cfg.pwm_max)

    def estimated_pwm(self, servo_id: int) -> int:
        """按运动模型估算当前位置；没有运动记录就返回最近一次 PRAD。"""
        sid = int(servo_id)
        with self._state_lock:
            motion = self._motions.get(sid)
            known = self._last_known.get(sid)
        if motion is not None:
            return motion.pwm_at(self._clock())
        return known if known is not None else self.center_pwm()

    def read_position(self, servo_id: int) -> Optional[int]:
        """发 PRAD 读回真实位置；读不到返回 None。"""
        sid = int(servo_id)
        pwm = parse_pwm_reply(self._query(build_prad(sid)))
        if pwm is None:
            self._logger.warning(f'PRAD 没读到回显：id={sid}')
            return None

        with self._state_lock:
            self._last_known[sid] = pwm
            motion = self._motions.get(sid)
            if motion is not None and self._clock() >= motion.started_at + motion.duration_s:
                self._motions.pop(sid, None)
        return pwm

    # ==================== 动作 ====================

    def move(self, servo_id: int, pwm: int, duration_ms: int = 0) -> bool:
        """先 PULR 再 P...T...，并记录运动模型。返回是否写成功。"""
        sid = int(servo_id)
        cfg = self._config

        if self._fault:
            self._logger.warning(f'安全模式未解除，拒绝动作：id={sid} pwm={pwm}')
            return False
        if not is_valid_id(sid):
            self._logger.warning(f'非法舵机 id={sid}（1~254）')
            return False
        if sid not in cfg.servo_ids:
            self._logger.warning(f'id={sid} 不在 servo_ids={cfg.servo_ids}，拒绝下发')
            return False

        target = int(pwm)
        if not cfg.pwm_min <= target <= cfg.pwm_max:
            clamped = clamp_pwm(target, cfg.pwm_min, cfg.pwm_max)
            self._logger.warning(
                f'pwm={target} 超出安全区间 [{cfg.pwm_min}, {cfg.pwm_max}]，夹到 {clamped}')
            target = clamped

        duration = int(duration_ms) if duration_ms else cfg.default_duration_ms
        if duration <= 0:
            duration = cfg.default_duration_ms

        start = self.estimated_pwm(sid)

        if not self._send(build_pulr(sid)):     # 恢复扭力，否则位置指令不生效
            return False
        self._sleep(0.02)
        if not self._send(build_move(sid, target, duration)):
            return False

        with self._state_lock:
            self._motions[sid] = Motion(
                start_pwm=start,
                target_pwm=target,
                duration_s=duration / 1000.0,
                started_at=self._clock(),
            )

        self._emit({'event': 'move', 'id': sid, 'pwm': target, 'duration_ms': duration})
        return True

    def pulk(self, servo_id: int) -> bool:
        return self._send(build_pulk(int(servo_id)))

    def pulk_all(self) -> int:
        ids = list(self._config.servo_ids)
        sent = 0
        for sid in ids:
            if self._send(build_pulk(int(sid))):
                sent += 1
        self._logger.info(f'已发送 PULK 卸力：{sent}/{len(ids)} 个舵机')
        return sent

    # ==================== PRAD 闭环 ====================

    def verify(self, servo_id: int, target_pwm: int, duration_ms: int,
               tolerance: Optional[int] = None) -> VerifyResult:
        """动作结束后读 PRAD 比对；没到位就重发一次再读（加分项：闭环校验）。"""
        sid = int(servo_id)
        target = int(target_pwm)
        tol = self._config.position_tolerance if tolerance is None else int(tolerance)
        duration = int(duration_ms) or self._config.default_duration_ms

        self._sleep(0.1)                     # 给控制板一点时间落位
        final = self.read_position(sid)
        retried = False

        if final is not None and abs(final - target) > tol:
            retried = True
            self._logger.warning(
                f'[PRAD] 未到位：id={sid} 目标={target} 实际={final} 偏差={final - target} '
                f'超过阈值 ±{tol} → 重发一次')
            if not self.move(sid, target, duration):
                return VerifyResult(False, None, True, None)
            self._sleep(duration / 1000.0 + 0.15)
            final = self.read_position(sid)
            if final is None:
                self._logger.error(f'[PRAD] 重发后仍读不到回显：id={sid}')
                return VerifyResult(False, None, True, None)
            if abs(final - target) <= tol:
                self._logger.info(f'[PRAD] 重发后到位：id={sid} 实际={final} 偏差={final - target}')
            else:
                self._logger.error(
                    f'[PRAD] 重发后仍未到位：id={sid} 目标={target} 实际={final} '
                    f'偏差={final - target}')

        if final is None:
            return VerifyResult(False, None, retried, None)
        diff = final - target
        return VerifyResult(abs(diff) <= tol, final, retried, diff)

    # ==================== 看门狗 ====================

    def heartbeat(self) -> None:
        """节点每 0.5s 调一次；断开超过 watchdog_timeout_s 就触发安全模式。"""
        self._last_beat = self._clock()

    def start_watchdog(self) -> None:
        timeout = self._config.watchdog_timeout_s
        if timeout <= 0:
            self._logger.info('看门狗已关闭（watchdog_timeout_s<=0）')
            return
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name='servo-watchdog', daemon=True)
        self._watchdog_thread.start()
        self._logger.info(f'看门狗已启动：{timeout:.1f} 秒没有心跳就全部卸力')

    def _watchdog_loop(self) -> None:
        while not self._watchdog_stop.wait(self.WATCHDOG_POLL_S):
            if self._fault:
                continue
            timeout = self._config.watchdog_timeout_s
            if timeout <= 0:
                continue
            if self._clock() - self._last_beat > timeout:
                self.trigger_fault(f'超过 {timeout:.1f} 秒没有心跳')

    def stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._watchdog_thread = None

    def trigger_fault(self, reason: str) -> None:
        """进入安全模式：全部 PULK，之后拒绝一切动作，直到 recover()。"""
        with self._state_lock:
            if self._fault:
                return
            self._fault = True
            self._fault_reason = str(reason)
        self._logger.error(
            f'[WATCHDOG] {reason} → 全部舵机 PULK 卸力，进入安全模式；'
            f'需外部确认 ros2 service call /servo/recover std_srvs/srv/Trigger 才能恢复')
        self.pulk_all()
        self._emit({'event': 'fault', 'reason': self._fault_reason})

    def recover(self):
        """外部确认后解除安全模式，返回 (是否成功, 说明)。"""
        with self._state_lock:
            if not self._fault:
                return False, 'not in safe mode'
            self._fault = False
            self._fault_reason = ''
            self._last_beat = self._clock()
        self._logger.info('安全模式已解除（外部确认），可以继续发 goal')
        return True, 'recovered'

    # ==================== 对外状态 ====================

    def _emit(self, payload: dict) -> None:
        callback = self.on_state
        if callback is None:
            return
        try:
            callback(dict(payload))
        except Exception as exc:        # 状态话题挂了不能影响控制
            self._logger.warning(f'state 回调异常：{exc}')
