"""场地地图加载：field.yaml + field_geometry.yaml。

包含官方拼写别名映射（PenalityPoint -> PenaltyPoint），
以及场地合法性检查（标记是否落在场地范围内）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from .common import load_yaml

# 官方拼写 -> 内部统一类别名
CLASS_ALIASES = {
    "PenalityPoint": "PenaltyPoint",
    "LCross": "LCross",
    "TCross": "TCross",
    "XCross": "XCross",
}


@dataclass
class FieldMarker:
    type: str
    x: float
    y: float


class FieldMap:
    """场地地图：加载标志点坐标、轴约定、场地尺寸。"""

    def __init__(self, field_yaml_path: str, geometry_yaml_path: str):
        self.field_yaml_path = field_yaml_path
        self.geometry_yaml_path = geometry_yaml_path

        field = load_yaml(field_yaml_path)
        geometry = load_yaml(geometry_yaml_path)

        self.version = field.get("version", "unknown")
        self.geometry_version = geometry.get("version", "unknown")
        self.unit = field.get("unit", "meter")

        # 轴约定
        axis = geometry.get("axis", {})
        self.yaw_positive = axis.get("yaw_positive", "counter_clockwise")
        self.yaw_unit = axis.get("yaw_unit", "degree")

        # 场地尺寸
        fd = geometry.get("field", {})
        self.length_x = float(fd.get("length_x", 9.0))
        self.length_y = float(fd.get("length_y", 6.0))
        self.origin = np.array(fd.get("origin", [0.0, 0.0]), dtype=float)

        # 解析标志点，做别名映射
        self.markers: List[FieldMarker] = []
        for m in field.get("markers", []):
            t = m.get("type", "")
            t = CLASS_ALIASES.get(t, t)  # 显式别名映射
            self.markers.append(
                FieldMarker(type=t, x=float(m["x"]), y=float(m["y"]))
            )

        self._check_units()

    def _check_units(self) -> None:
        """启动时检查单位、尺寸、轴配置是否一致。"""
        if self.unit != "meter":
            raise ValueError(f"field.yaml 单位应为 meter，实际为 {self.unit}")
        if self.length_x <= 0 or self.length_y <= 0:
            raise ValueError("field_geometry.yaml 场地尺寸非法（应 > 0）")
        if self.yaw_unit != "degree":
            raise ValueError(f"yaw 单位应为 degree，实际为 {self.yaw_unit}")

    def markers_of_type(self, marker_type: str) -> List[FieldMarker]:
        return [m for m in self.markers if m.type == marker_type]

    def all_xy(self) -> np.ndarray:
        """所有标志点的 (N,2) 坐标数组。"""
        return np.array([[m.x, m.y] for m in self.markers], dtype=float)

    def types(self) -> List[str]:
        return [m.type for m in self.markers]

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        """判断地图坐标 (x,y) 是否落在场地范围内（含 margin）。"""
        ox, oy = self.origin
        return (
            ox - margin <= x <= ox + self.length_x + margin
            and oy - margin <= y <= oy + self.length_y + margin
        )
