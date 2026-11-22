"""field.yaml / field_geometry.yaml 解析。

field.yaml 的真实格式是扁平的 `键名: [x, y]`，键名如 TCross1 / LCross3 /
XCross2 / PenalityPoint（官方把 PenaltyPoint 拼错成 PenalityPoint）。
"""
from __future__ import annotations

import hashlib
import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# YOLO 类别顺序，必须与标注一致（细则 2.3）
CLASSES: List[str] = ["LCross", "TCross", "XCross", "PenaltyPoint"]
CLASS_TO_ID = {c: i for i, c in enumerate(CLASSES)}

_ALIASES = {
    "lcross": "LCross", "tcross": "TCross", "xcross": "XCross",
    "penalitypoint": "PenaltyPoint",   # 官方拼写
    "penaltypoint": "PenaltyPoint",    # 正确拼写
    "penalty": "PenaltyPoint", "penality": "PenaltyPoint",
}
_KEY_RE = re.compile(r"^([A-Za-z_]+?)(\d*)$")


@dataclass
class FieldPoint:
    mid: str      # 地图 ID，如 "TCross1"
    type: str     # 类别名，如 "TCross"
    x: float
    y: float


@dataclass
class FieldMap:
    points: List[FieldPoint]
    by_type: Dict[str, List[FieldPoint]]
    sha256: str
    path: str
    bounds: Tuple[float, float, float, float]   # xmin, xmax, ymin, ymax
    margin: float = 0.6
    meta: dict = field(default_factory=dict)

    def in_field(self, x: float, y: float) -> bool:
        xmin, xmax, ymin, ymax = self.bounds
        m = self.margin
        return (xmin - m) <= x <= (xmax + m) and (ymin - m) <= y <= (ymax + m)

    def summary(self) -> str:
        cnt = ", ".join(f"{k}:{len(v)}" for k, v in sorted(self.by_type.items()))
        xmin, xmax, ymin, ymax = self.bounds
        return (f"{os.path.basename(self.path)} sha256={self.sha256[:12]} "
                f"points={len(self.points)} ({cnt}) "
                f"bounds=[{xmin:.3f},{xmax:.3f}]x[{ymin:.3f},{ymax:.3f}]")


def sha256_of(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _parse_flat_yaml(text: str) -> dict:
    """极简解析 `key: [a, b]` / `key: value`，pyyaml 不可用时兜底。"""
    out = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        k, v = line.split(":", 1)
        k, v = k.strip(), v.strip()
        if v.startswith("[") and v.endswith("]"):
            out[k] = [float(t) for t in v[1:-1].split(",") if t.strip()]
        else:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(text)
        return data if isinstance(data, dict) else {}
    except Exception:
        return _parse_flat_yaml(text)


def parse_type(key: str) -> Optional[str]:
    m = _KEY_RE.match(key.strip())
    base = (m.group(1) if m else key).strip()
    return _ALIASES.get(base.lower())


def load_field(path: str, geometry_path: Optional[str] = None) -> FieldMap:
    data = _load_yaml(path)
    points: List[FieldPoint] = []
    for key, val in data.items():
        if not isinstance(key, str):
            continue
        if isinstance(val, (list, tuple)) and len(val) >= 2:
            try:
                cx, cy = float(val[0]), float(val[1])
            except (TypeError, ValueError):
                continue
            t = parse_type(key)
            if t is None:
                continue
            points.append(FieldPoint(mid=key, type=t, x=cx, y=cy))

    if not points:
        raise ValueError(f"{path} 中没有解析到任何场地特征点")

    by_type: Dict[str, List[FieldPoint]] = {}
    for p in points:
        by_type.setdefault(p.type, []).append(p)

    xs = [p.x for p in points]
    ys = [p.y for p in points]
    bounds = (min(xs), max(xs), min(ys), max(ys))
    margin, meta = 0.6, {}

    if geometry_path and os.path.exists(geometry_path):
        g = _load_yaml(geometry_path)
        b = g.get("bounds") if isinstance(g, dict) else None
        if isinstance(b, dict):
            try:
                bounds = (float(b["x_min"]), float(b["x_max"]),
                          float(b["y_min"]), float(b["y_max"]))
            except (KeyError, TypeError, ValueError):
                pass
        if isinstance(g, dict):
            margin = float(g.get("valid_margin_m", margin))
            meta = {"map_version": g.get("map_version", ""),
                    "axes": g.get("axes", {})}

    return FieldMap(points=points, by_type=by_type, sha256=sha256_of(path),
                    path=path, bounds=bounds, margin=margin, meta=meta)
