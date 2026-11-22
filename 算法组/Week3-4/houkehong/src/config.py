# -*- coding: utf-8 -*-
"""
配置加载：field.yaml / field_geometry.yaml / camera.yaml

设计原则：
  * 文件缺失或字段未填时给出**明确可读的错误**，而不是悄悄用默认值；
  * 支持官方 field.yaml 里 `PenalityPoint` 的拼写别名（细则 2.2 要求显式做别名映射）；
  * 地图单位、原点、轴向由 field_geometry.yaml 声明，程序启动时校验（细则 2.10 要求）。
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import yaml

from .geometry import CameraModel, Extrinsics
from .matching import MapFeature

TYPE_ALIASES = {
    "penalitypoint": "PenaltyPoint",
    "penaltypoint": "PenaltyPoint",
    "penalty_point": "PenaltyPoint",
    "l": "LCross", "lcross": "LCross", "l_cross": "LCross",
    "t": "TCross", "tcross": "TCross", "t_cross": "TCross",
    "x": "XCross", "xcross": "XCross", "x_cross": "XCross",
}

CANONICAL = ("LCross", "TCross", "XCross", "PenaltyPoint")

# ---------------------------------------------------------------------------
# 现场实际存在的场地标志类别（**这是唯一配置点，其他地方都从这里 import**）
#
# 2026-11-19 现场盘点结果（依据各段实拍视频）：
#   · L 型：场地角点、球门区角 —— 存在 ✅
#   · T 型：中线/边线与另一条线的交汇 —— 存在 ✅
#   · X 型：**不存在**。视频里出现的"十字"全是中线穿过中圈形成的
#           「直线 × 圆弧」交叉，不是两条直线交叉。
#           这种交叉无法用"两直线求交"精确定位（landmarks.py 的做法），
#           强行标注会引入大误差，所以不训这一类。
#   · 点球点：**不存在**。场地中间没有标志点（用户现场确认）。
#
# 考核细则明确："现场没有的类别不得强行要求训练或评分。"
# 如果以后场地增加了标志，改这个列表即可，其他代码自动跟随。
# ---------------------------------------------------------------------------
FIELD_MARKER_CLASSES = ["LCross", "TCross"]



class ConfigError(RuntimeError):
    pass


def canonical_type(name: str) -> str:
    """
    把各种写法归一成 4 个规范类别名。

    官方 field.yaml 用的是扁平键值格式，键名带编号：
        TCross1: [5.955, 2.930]
        LCross3: [4.945, 2.020]
        PenalityPoint: [2.965, 0.885]      # 官方拼写，少一个 y
    所以这里必须先剥掉尾部编号，否则 TCross1 会被当成未知类型而整条丢掉。
    """
    key = str(name).strip().lower().replace("-", "_").replace(" ", "_")
    key = re.sub(r"_*\d+$", "", key)          # TCross1 / TCross_1 / t_cross_2 -> tcross
    if key in TYPE_ALIASES:
        return TYPE_ALIASES[key]
    for c in CANONICAL:
        if key == c.lower():
            return c
    raise ConfigError(f"未知的标志类型: {name!r}（允许: {', '.join(CANONICAL)}，"
                      f"以及官方拼写 PenalityPoint 和 TCross1 这类带编号的写法）")


def _read_yaml(path: str) -> dict:
    if not os.path.isfile(path):
        raise ConfigError(f"找不到配置文件: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        raise ConfigError(f"配置文件为空: {path}")
    if not isinstance(data, dict):
        raise ConfigError(f"配置文件顶层必须是字典: {path}")
    return data


# ---------------------------------------------------------------- 场地

@dataclass
class FieldMap:
    features: list[MapFeature]
    geometry: dict
    version: str = "unknown"
    source_path: str = ""
    origin: tuple = (0.0, 0.0)
    axes: tuple = ("+X", "+Y")
    unit: str = "m"

    def by_type(self) -> dict:
        out = {}
        for f in self.features:
            out.setdefault(f.type, []).append(f)
        return out

    def bounds(self) -> tuple[float, float, float, float]:
        """
        返回 (minx, maxx, miny, maxy)，按优先级：
          1. geometry 里显式声明的 bounds（推荐，官方地图原点常在场内，不能用角落推算）
          2. geometry 里的 length/width + origin（假设原点在场角）
          3. 退回特征点的外包围框
        """
        g = self.geometry or {}
        b = g.get("bounds")
        if isinstance(b, dict) and all(k in b for k in ("min_x", "max_x", "min_y", "max_y")):
            return (float(b["min_x"]), float(b["max_x"]),
                    float(b["min_y"]), float(b["max_y"]))

        length = g.get("length_m") or g.get("length") or g.get("field_length")
        width = g.get("width_m") or g.get("width") or g.get("field_width")
        ox, oy = self.origin
        if length and width:
            return (ox, ox + float(length), oy, oy + float(width))

        xs = [f.x for f in self.features]
        ys = [f.y for f in self.features]
        if not xs:
            return (0.0, 1.0, 0.0, 1.0)
        return (min(xs) - 0.5, max(xs) + 0.5, min(ys) - 0.5, max(ys) + 0.5)


def load_field(field_path: str, geometry_path: Optional[str] = None) -> FieldMap:
    data = _read_yaml(field_path)
    raw = data.get("features")

    feats: list[MapFeature] = []
    if raw is None:
        # ---------------------------------------------------------------
        # 兼容官方 field.yaml 的**扁平键值**写法：
        #     TCross1: [5.955, 2.930]
        #     PenalityPoint: [2.965, 0.885]
        # 每个键对应**一个**坐标对（注意：不是坐标对列表！）
        #
        # 同时兼容按类型分组的写法：
        #     LCross: [[0,0], [9,0]]
        #     TCross: [{x: 3, y: 0}, ...]
        # ---------------------------------------------------------------
        def is_number(v):
            return isinstance(v, (int, float)) and not isinstance(v, bool)

        for k, v in data.items():
            if str(k) in ("version", "meta", "field", "features"):
                continue
            try:
                t = canonical_type(k)
            except ConfigError:
                continue
            if not isinstance(v, (list, tuple)):
                continue

            # 情况 A：单个坐标对 [x, y]（官方 field.yaml 就是这种）
            if len(v) >= 2 and is_number(v[0]) and is_number(v[1]):
                feats.append(MapFeature(fid=len(feats), type=t,
                                        x=float(v[0]), y=float(v[1]),
                                        name=str(k)))
                continue

            # 情况 B：坐标对列表 [[x,y], ...] 或 [{x,y}, ...]
            for item in v:
                if isinstance(item, dict):
                    if "x" in item and "y" in item:
                        feats.append(MapFeature(fid=len(feats), type=t,
                                                x=float(item["x"]), y=float(item["y"]),
                                                name=str(item.get("name", ""))))
                elif isinstance(item, (list, tuple)) and len(item) >= 2:
                    feats.append(MapFeature(fid=len(feats), type=t,
                                            x=float(item[0]), y=float(item[1]),
                                            name=str(k)))

        if not feats:
            raise ConfigError(
                f"{field_path} 里没有解析到任何场地特征。\n"
                f"  支持两种格式：\n"
                f"    A) features: [{{id, type, x, y}}, ...]\n"
                f"    B) 官方扁平写法: TCross1: [5.955, 2.930] / PenalityPoint: [2.965, 0.885]\n"
                f"  已读到的顶层键: {list(data.keys())[:12]}"
            )
    else:
        if not isinstance(raw, list):
            raise ConfigError(f"{field_path}: features 必须是列表")
        for i, item in enumerate(raw):
            if not isinstance(item, dict):
                raise ConfigError(f"{field_path}: features[{i}] 必须是字典")
            t = canonical_type(item.get("type") or item.get("class") or "")
            if "x" not in item or "y" not in item:
                raise ConfigError(f"{field_path}: features[{i}] 缺少 x / y")
            feats.append(MapFeature(fid=int(item.get("id", i)), type=t,
                                    x=float(item["x"]), y=float(item["y"])))

    geometry = {}
    version = str(data.get("version", "unknown"))
    if geometry_path:
        if os.path.isfile(geometry_path):
            geometry = _read_yaml(geometry_path)
            version = str(geometry.get("map_version", geometry.get("version", version)))
        else:
            raise ConfigError(f"找不到 field_geometry.yaml: {geometry_path}")

    origin = geometry.get("origin", (0.0, 0.0))
    if isinstance(origin, dict):
        origin = (float(origin.get("x", 0.0)), float(origin.get("y", 0.0)))
    else:
        origin = (float(origin[0]), float(origin[1]))

    fm = FieldMap(features=feats, geometry=geometry, version=version,
                  source_path=os.path.abspath(field_path), origin=origin)
    return fm


def check_field(fm: FieldMap) -> list[str]:
    """启动时检查地图配置（细则 2.10）。返回警告列表，不抛异常。"""
    warns = []
    if not fm.features:
        warns.append("地图中没有任何特征点，无法定位")
    counts = {t: len(v) for t, v in fm.by_type().items()}
    if not counts:
        warns.append("地图特征按类型统计为空")
    for t, c in counts.items():
        if c == 1:
            warns.append(f"类型 {t} 在整张地图中只有 1 个实例，"
                         f"观测到该类型时无法靠同类点对消除歧义")
    g = fm.geometry or {}
    if "length_m" not in g and "length" not in g:
        warns.append("field_geometry.yaml 未声明场地长度，地图按特征外包围框绘制")
    if str(g.get("unit", "m")).lower() not in ("m", "meter", "metre", "米"):
        warns.append(f"地图单位是 {g.get('unit')}，本程序按米计算，请确认已换算")
    return warns


# ---------------------------------------------------------------- 相机

def load_camera(path: str) -> CameraModel:
    data = _read_yaml(path)
    cam = data.get("camera", data)
    K = cam.get("K") or data.get("K")
    if K is not None:
        K = np.asarray(K, dtype=np.float64)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    else:
        fx, fy = cam.get("fx"), cam.get("fy")
        cx, cy = cam.get("cx"), cam.get("cy")
    if fx is None or fy is None or cx is None or cy is None:
        raise ConfigError(
            f"{path} 里内参未填写（fx/fy/cx/cy 为 null）。\n"
            f"请先用棋盘格完成标定（细则 2.5 要求，不允许用标称值代替）。"
        )
    dist = cam.get("dist_coeffs") or data.get("D") or [0, 0, 0, 0, 0]
    dist = np.asarray([0.0 if d is None else float(d) for d in dist], dtype=np.float64)
    return CameraModel(fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy),
                       dist=dist,
                       width=int(cam.get("image_width", data.get("image_width", 1280))),
                       height=int(cam.get("image_height", data.get("image_height", 720))))


def load_extrinsics(path: str) -> Extrinsics:
    data = _read_yaml(path)
    ex = data.get("extrinsics", data)

    R = ex.get("R_carrier_camera") or ex.get("R")
    t = ex.get("t_carrier_camera") or ex.get("t")

    if R is not None and t is not None:
        R = np.asarray(R, dtype=np.float64).reshape(3, 3)
        t = np.asarray(t, dtype=np.float64).reshape(3)
        # 简单的正交性检查，防止手抄错
        if not np.allclose(R @ R.T, np.eye(3), atol=1e-3):
            raise ConfigError(f"{path}: R_carrier_camera 不是正交矩阵")
        return Extrinsics(R=R, t=t)

    h = ex.get("camera_height_m")
    if h is None:
        raise ConfigError(
            f"{path} 里外参未填写（R_carrier_camera / t_carrier_camera 为 null，"
            f"camera_height_m 也为 null）。\n"
            f"请完成外参标定（细则 2.6 要求），或先用 --demo 模式看合成数据演示。"
        )
    return Extrinsics.from_mount(
        height_m=float(h),
        pitch_deg=float(ex.get("pitch_deg") or 0.0),
        roll_deg=float(ex.get("roll_deg") or 0.0),
        yaw_offset_deg=float(ex.get("yaw_offset_deg") or 0.0),
        tx=float(ex.get("tx") or 0.0),
        ty=float(ex.get("ty") or 0.0),
    )
