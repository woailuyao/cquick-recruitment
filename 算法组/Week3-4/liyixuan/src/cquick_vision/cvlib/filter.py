"""位姿滤波：EMA + 角度环绕 + 失效超时复位。

原则：valid=False 时只允许短暂保持，超过 max_frames_hold 立即丢弃，
绝不无限沿用旧位姿（细则 2.10）。
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

from .geom import wrap_rad


class PoseFilter:
    def __init__(self, alpha_xy: float = 0.5, alpha_yaw: float = 0.5,
                 max_hold: int = 5):
        self.alpha_xy = float(alpha_xy)
        self.alpha_yaw = float(alpha_yaw)
        self.max_hold = int(max_hold)
        self.reset()

    def reset(self) -> None:
        self.state: Optional[Tuple[float, float, float]] = None
        self.hold = 0

    @property
    def lost(self) -> bool:
        return self.state is None

    def update(self, pose: Tuple[float, float, float], valid: bool
               ) -> Optional[Tuple[float, float, float]]:
        if valid and pose is not None:
            x, y, th = pose
            th = wrap_rad(th)
            if self.state is None:
                self.state = (x, y, th)
            else:
                sx, sy, sth = self.state
                a = self.alpha_xy
                sx = a * x + (1.0 - a) * sx
                sy = a * y + (1.0 - a) * sy
                dth = wrap_rad(th - sth)      # 跨 ±180° 必须走最短路径
                sth = wrap_rad(sth + self.alpha_yaw * dth)
                self.state = (sx, sy, sth)
            self.hold = 0
            return self.state

        self.hold += 1
        if self.hold > self.max_hold:
            self.reset()
        return None
