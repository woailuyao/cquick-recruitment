from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

__all__ = ["FieldPose", "camera_pose_from_carrier"]


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def camera_pose_from_carrier(carrier_pose, extrinsics) -> tuple:
    R = extrinsics.R
    t = extrinsics.t
    #光轴转到载体系后，取水平投影算相机朝向
    axis = R @ np.array([0.0, 0.0, 1.0])
    yaw_offset = math.degrees(math.atan2(axis[1], axis[0]))
    #相机安装偏移随载体一起转到地图系
    pt = carrier_pose.apply([[t[0], t[1]]])[0]
    return float(pt[0]), float(pt[1]), _wrap_deg(carrier_pose.yaw_deg + yaw_offset)


@dataclass
class FieldPose:
    x_camera_map: float = math.nan
    y_camera_map: float = math.nan
    yaw_camera_map: float = math.nan
    x_carrier_map: float = math.nan
    y_carrier_map: float = math.nan
    yaw_carrier_map: float = math.nan
    valid: bool = False
    marker_count: int = 0
    residual: float = math.nan
    status: str = "lost"
    frame_id: int = 0
    stamp: float = field(default_factory=time.time)
    extra: dict = field(default_factory=dict)

    @classmethod
    def from_match(cls, match, carrier_pose, extrinsics=None, frame_id: int = 0) -> "FieldPose":
        pose = cls(
            valid=bool(match.valid and carrier_pose is not None),
            marker_count=int(match.marker_count),
            residual=float(match.residual) if math.isfinite(match.residual) else math.nan,
            status=str(match.status),
            frame_id=int(frame_id),
        )
        if pose.valid:
            pose.x_carrier_map = float(carrier_pose.x)
            pose.y_carrier_map = float(carrier_pose.y)
            pose.yaw_carrier_map = float(carrier_pose.yaw_deg)
            if extrinsics is not None:
                x, y, yaw = camera_pose_from_carrier(carrier_pose, extrinsics)
            else:
                x, y, yaw = carrier_pose.x, carrier_pose.y, carrier_pose.yaw_deg
            pose.x_camera_map, pose.y_camera_map, pose.yaw_camera_map = x, y, yaw
        return pose

    def to_dict(self) -> dict:
        def r(v, n=4):
            return round(v, n) if isinstance(v, float) and math.isfinite(v) else None
        return {
            "frame_id": self.frame_id,
            "stamp": round(self.stamp, 3),
            "x_camera_map": r(self.x_camera_map),
            "y_camera_map": r(self.y_camera_map),
            "yaw_camera_map": r(self.yaw_camera_map, 3),
            "x_carrier_map": r(self.x_carrier_map),
            "y_carrier_map": r(self.y_carrier_map),
            "yaw_carrier_map": r(self.yaw_carrier_map, 3),
            "valid": self.valid,
            "marker_count": self.marker_count,
            "residual": r(self.residual),
            "status": self.status,
            **self.extra,
        }

    def to_yaml(self) -> str:
        def f(v, n=4):
            return f"{v:.{n}f}" if isinstance(v, float) and math.isfinite(v) else "null"
        lines = [
            "camera_pose_map:",
            f"  x_camera_map: {f(self.x_camera_map, 3)}",
            f"  y_camera_map: {f(self.y_camera_map, 3)}",
            f"  yaw_camera_map: {f(self.yaw_camera_map, 2)}",
            f"  marker_count: {self.marker_count}",
            f"  residual: {f(self.residual, 3)}",
            f"  valid: {'true' if self.valid else 'false'}",
            f"  status: \"{self.status}\"",
        ]
        return "\n".join(lines)
