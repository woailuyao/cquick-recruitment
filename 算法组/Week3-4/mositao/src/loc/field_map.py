# -*- coding: utf-8 -*-
"""场地标志地图加载：field.yaml + field_geometry.yaml。

官方 field.yaml 的实际结构已经取到（BoosterRobotics/robocup_demo，369 B）：
顶层扁平映射，键名 = 类别 + 序号，值 = [x, y] 两元素列表：

    TCross1: [ 5.955,  2.930]
    PenalityPoint: [ 2.965,  0.885]

因此这里两条解析路径都保留：扁平写法（官方）与嵌套写法（自建地图 / 未来修订版），
再按类别别名归一。仍不预设层级，递归扫描兜底。

注意（细则 2.2）：官方点球点键名写作 `PenalityPoint`（官方拼写错误），
模型类别统一为 `PenaltyPoint`，此处显式做别名映射。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..common.config_io import load_yaml, pick, sha256_of
from ..common.paths import rel

# 类别别名 -> 统一类别名
TYPE_ALIASES = {
    "l": "LCross", "lcross": "LCross", "l_cross": "LCross", "lmark": "LCross",
    "lcorner": "LCross", "l型": "LCross", "l型角点": "LCross", "l交点": "LCross",
    "t": "TCross", "tcross": "TCross", "t_cross": "TCross", "tmark": "TCross",
    "t型": "TCross", "t型交点": "TCross", "t交点": "TCross",
    "x": "XCross", "xcross": "XCross", "x_cross": "XCross", "xmark": "XCross",
    "x型": "XCross", "x型交点": "XCross", "x交点": "XCross", "cross": "XCross",
    "p": "PenaltyPoint", "pp": "PenaltyPoint", "penaltypoint": "PenaltyPoint",
    "penalitypoint": "PenaltyPoint",   # 官方拼写
    "penalty": "PenaltyPoint", "penality": "PenaltyPoint",
    "penalty_point": "PenaltyPoint", "点球点": "PenaltyPoint",
}

VALID_TYPES = ("LCross", "TCross", "XCross", "PenaltyPoint")


def normalize_type(raw) -> str | None:
    if raw is None:
        return None
    key = str(raw).strip().lower().replace(" ", "").replace("-", "")
    # 官方 field.yaml 的键名带序号（TCross1 / LCross5 / PenalityPoint），
    # 先剥掉尾部序号再查表，避免落到子串匹配而误判类别（"penalitypoint" 里有 'l'）。
    key_nodigit = key.rstrip("0123456789")
    if key_nodigit in TYPE_ALIASES:
        return TYPE_ALIASES[key_nodigit]
    if key in TYPE_ALIASES:
        return TYPE_ALIASES[key]
    for k, v in TYPE_ALIASES.items():
        if k in key:
            return v
    return None


@dataclass
class FieldMarker:
    type: str
    x: float
    y: float
    name: str = ""

    def as_dict(self) -> dict:
        return {"type": self.type, "x": self.x, "y": self.y, "name": self.name}


@dataclass
class FieldMap:
    markers: list = field(default_factory=list)
    geometry: dict = field(default_factory=dict)
    field_path: str = ""
    geometry_path: str = ""
    field_sha256: str = ""
    warnings: list = field(default_factory=list)

    # ---------- 构造 ----------
    @classmethod
    def from_files(cls, field_path, geometry_path=None) -> "FieldMap":
        data = load_yaml(field_path)
        markers, warns = parse_markers(data)
        geom = {}
        gpath = ""
        if geometry_path:
            gdata = load_yaml(geometry_path)
            geom = gdata.get("field_geometry", gdata)
            gpath = rel(geometry_path)
        else:
            # 没有独立的 field_geometry.yaml 时，用 field.yaml 里的 _meta 自描述几何。
            # 这样单个地图文件就是自洽的，tools/build_field_map.py 再把它展开成正式文件。
            meta = data.get("_meta", {}) if isinstance(data.get("_meta"), dict) else {}
            if meta:
                name = meta.get("name", "C0")
                rev = meta.get("rev", "v1")
                geom = {
                    "version": f"{name}-{rev}",
                    "map_version": rev,
                    "field_length_m": meta.get("field_length_m"),
                    "field_width_m": meta.get("field_width_m"),
                    "units": "m",
                    "origin": {"description": meta.get("origin", "场地中心")},
                    "source": "field.yaml/_meta",
                }
        fm = cls(
            markers=markers,
            geometry=geom,
            field_path=rel(field_path),
            geometry_path=gpath,
            field_sha256=sha256_of(field_path),
            warnings=warns,
        )
        fm.validate_against_geometry()
        return fm

    # ---------- 查询 ----------
    def by_type(self) -> dict:
        out: dict = {}
        for m in self.markers:
            out.setdefault(m.type, []).append(m)
        return out

    def counts(self) -> dict:
        out: dict = {}
        for m in self.markers:
            out[m.type] = out.get(m.type, 0) + 1
        return out

    def bounds(self):
        if not self.markers:
            return None
        xs = [m.x for m in self.markers]
        ys = [m.y for m in self.markers]
        return min(xs), max(xs), min(ys), max(ys)

    # ---------- 校验 ----------
    def validate_against_geometry(self) -> None:
        """检查地图与 field_geometry 是否自洽；只告警，不擅自改学生数据。"""
        size = pick(self.geometry, "length_m", "field_length_m", "length", default=None)
        width = pick(self.geometry, "width_m", "field_width_m", "width", default=None)
        b = self.bounds()
        if not self.markers:
            self.warnings.append("field.yaml 中未解析到任何标志点，请检查文件结构")
            return
        if b and size and width:
            span_x = b[1] - b[0]
            span_y = b[3] - b[2]
            if span_x > float(size) + 0.5 or span_y > float(width) + 0.5:
                self.warnings.append(
                    f"标志点跨度({span_x:.2f} x {span_y:.2f} m) 超出场地尺寸"
                    f"({float(size):.2f} x {float(width):.2f} m)，疑似坐标单位或原点不一致"
                )
        missing = [t for t in VALID_TYPES if t not in self.counts()]
        if missing:
            self.warnings.append("地图中缺少类别: " + ", ".join(missing))

    def summary(self) -> str:
        c = self.counts()
        b = self.bounds()
        lines = [
            f"地图文件: {self.field_path}",
            f"SHA256  : {self.field_sha256}",
            "类别分布: " + (", ".join(f"{k}={v}" for k, v in sorted(c.items())) if c else "空"),
        ]
        if b:
            lines.append(f"坐标范围: x[{b[0]:.2f}, {b[1]:.2f}]  y[{b[2]:.2f}, {b[3]:.2f}] m")
        if self.geometry:
            g = self.geometry
            lines.append(
                "场地几何: "
                f"version={pick(g, 'version', 'map_version', default='?')} "
                f"length={pick(g, 'length_m', 'field_length_m', default='?')} "
                f"width={pick(g, 'width_m', 'field_width_m', default='?')} "
                f"units={pick(g, 'units', default='m')}"
            )
        for w in self.warnings:
            lines.append("警告    : " + w)
        return "\n".join(lines)


def _as_xy(node: dict):
    """从一个映射里取出 (x, y)；取不到返回 None。"""
    x = pick(node, "x", "x_m", "px", "gx")
    y = pick(node, "y", "y_m", "py", "gy")
    if isinstance(x, (int, float)) and isinstance(y, (int, float)):
        return float(x), float(y)
    for key in ("pos", "position", "point", "xy", "location", "coord", "coordinates"):
        v = node.get(key)
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            try:
                return float(v[0]), float(v[1])
            except (TypeError, ValueError):
                pass
        if isinstance(v, dict):
            xx = pick(v, "x", "x_m")
            yy = pick(v, "y", "y_m")
            if isinstance(xx, (int, float)) and isinstance(yy, (int, float)):
                return float(xx), float(yy)
    return None


def _type_of(node: dict, path: tuple) -> str | None:
    for key in ("type", "class", "cls", "kind", "label", "category", "marker_type"):
        t = normalize_type(node.get(key))
        if t:
            return t
    for seg in reversed(path):
        t = normalize_type(seg)
        if t:
            return t
    return None


def _walk(node, path: tuple = ()):
    if isinstance(node, dict):
        xy = _as_xy(node)
        if xy is not None:
            t = _type_of(node, path)
            if t:
                name = "/".join(str(p) for p in path if not str(p).startswith("_"))
                yield FieldMarker(type=t, x=xy[0], y=xy[1], name=name)
        for k, v in node.items():
            if str(k).startswith("_"):      # _meta 等注释块跳过
                continue
            # 官方 field.yaml 的实际写法：`TCross1: [ 5.955, 2.930]`
            # 键名即类别，值是 [x, y] 两元素列表。这一支专门吃这种扁平写法。
            if (isinstance(v, (list, tuple)) and len(v) >= 2
                    and all(isinstance(q, (int, float)) for q in v[:2])):
                t = normalize_type(k)
                if t:
                    yield FieldMarker(type=t, x=float(v[0]), y=float(v[1]), name=str(k))
                    continue
            yield from _walk(v, path + (str(k),))
    elif isinstance(node, (list, tuple)):
        for i, v in enumerate(node):
            yield from _walk(v, path + (str(i),))


def parse_markers(data: dict):
    """递归解析出所有场地标志点，返回 (markers, warnings)。"""
    markers = list(_walk(data))
    warnings = []
    seen = {}
    uniq = []
    for m in markers:
        key = (m.type, round(m.x, 6), round(m.y, 6))
        if key in seen:
            seen[key] += 1
            continue
        seen[key] = 1
        uniq.append(m)
    dup = sum(v - 1 for v in seen.values() if v > 1)
    if dup:
        warnings.append(f"检测到 {dup} 个重复坐标点，已去重")
    if not uniq:
        warnings.append(
            "未能从 field.yaml 解析出标志点：可能键名不在已知别名表内，"
            "请把真实文件片段补进 TYPE_ALIASES 或 _as_xy"
        )
    return uniq, warnings


def load_field_map(field_path, geometry_path=None) -> FieldMap:
    return FieldMap.from_files(field_path, geometry_path)


# ----------------------------------------------------------------------------
# 坐标系换算 + 对称补全
#
# 官方 field.yaml 的数值不是场地中心系：它相对场地中心旋转并平移（实测 -90° 与
# 平移 (4.010, 3.035)，见 config/field_geometry.yaml 的 file_to_field）。
# 官方文件只覆盖半个场地，而标准场地关于中线对称，所以另半场可由镜像推出。
# 注意：镜像结果是**派生**数据，不改官方文件；默认不启用，需显式调用。
# ----------------------------------------------------------------------------
def _frame_params(geometry: dict):
    """从 field_geometry 取 file->field 的 (旋转角, 平移)，取不到返回 None。"""
    ft = pick(geometry or {}, "file_to_field", default=None)
    if not isinstance(ft, dict):
        return None
    rot = pick(ft, "rotation_deg", default=None)
    tx = pick(ft, "translation_x_m", default=None)
    ty = pick(ft, "translation_y_m", default=None)
    if rot is None or tx is None or ty is None:
        return None
    return math.radians(float(rot)), float(tx), float(ty)


def to_field_xy(x, y, geometry: dict):
    """文件坐标 -> 场地中心系。无换算配置时原样返回。"""
    prm = _frame_params(geometry)
    if prm is None:
        return x, y
    th, tx, ty = prm
    return (x * math.cos(th) - y * math.sin(th) + tx,
            x * math.sin(th) + y * math.cos(th) + ty)


def from_field_xy(x, y, geometry: dict):
    """场地中心系 -> 文件坐标（to_field_xy 的逆）。"""
    prm = _frame_params(geometry)
    if prm is None:
        return x, y
    th, tx, ty = prm
    dx, dy = x - tx, y - ty
    return (dx * math.cos(th) + dy * math.sin(th),
            -dx * math.sin(th) + dy * math.cos(th))


def symmetric_completion(fm: FieldMap, x0: float = 0.0, tol: float = 1e-6) -> FieldMap:
    """关于场地中线 x = x0 镜像补出另半场，返回新的 FieldMap（不写文件）。

    标准足球场关于中线对称，官方地图只给了半场，因此另半场的位置可由镜像得出。
    已有同位置同类别的点不会重复添加。派生点 name 后缀 `_mirror`，便于区分。
    """
    markers = list(fm.markers)
    if not markers:
        return FieldMap(markers=[], geometry=fm.geometry, field_path=fm.field_path,
                        geometry_path=fm.geometry_path, field_sha256=fm.field_sha256,
                        warnings=list(fm.warnings))
    if _frame_params(fm.geometry) is None:
        raise ValueError(
            "field_geometry.yaml 缺少 file_to_field 换算参数（rotation_deg / "
            "translation_x_m / translation_y_m），无法判断中线位置")

    existing = {(m.type, round(m.x, 6), round(m.y, 6)) for m in markers}
    added = 0
    for m in list(fm.markers):
        fx, fy = to_field_xy(m.x, m.y, fm.geometry)
        if abs(fx - x0) < tol:          # 落在中线上的点，镜像即自身
            continue
        mx, my = from_field_xy(2 * x0 - fx, fy, fm.geometry)
        key = (m.type, round(mx, 6), round(my, 6))
        if key in existing:
            continue
        existing.add(key)
        markers.append(FieldMarker(type=m.type, x=mx, y=my, name=f"{m.name}_mirror"))
        added += 1

    warns = list(fm.warnings)
    warns.append(f"已按中线 x={x0:g} 镜像补全 {added} 个派生点（不改官方文件，仅内存）")
    return FieldMap(markers=markers, geometry=dict(fm.geometry), field_path=fm.field_path,
                    geometry_path=fm.geometry_path, field_sha256=fm.field_sha256,
                    warnings=warns)


def coverage(fm: FieldMap):
    """返回标志点在场地中心系下的 x 覆盖范围。"""
    xs = [to_field_xy(m.x, m.y, fm.geometry)[0] for m in fm.markers]
    return (min(xs), max(xs)) if xs else (float("nan"), float("nan"))
