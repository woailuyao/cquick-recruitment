"""场地地图加载与校验（field.yaml / field_geometry.yaml）。

坐标约定：地图 +X 向右、+Y 向上，单位 m，yaw 从 +X 逆时针为正（deg）。
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

MARKER_TYPES = ("LCross", "TCross", "XCross", "PenaltyPoint")

# 解析别名，官方 field.yaml 里点球点写作 PenalityPoint
TYPE_ALIASES = {
    "LCross": "LCross", "L": "LCross", "l": "LCross",
    "TCross": "TCross", "T": "TCross", "t": "TCross",
    "XCross": "XCross", "X": "XCross", "x": "XCross",
    "PenaltyPoint": "PenaltyPoint", "PenalityPoint": "PenaltyPoint",
    "P": "PenaltyPoint", "p": "PenaltyPoint",
}


@dataclass
class MapMarker:
    id: str
    type: str
    x: float
    y: float

    def xy(self):
        return (self.x, self.y)


def type_from_key(key):
    """'TCross1' -> 'TCross' ; 'PenalityPoint' -> 'PenaltyPoint' ; 'L_2' -> 'LCross'?"""
    base = key.strip()
    base = re.sub(r"[-_]?\d+$", "", base)          # 去掉末尾编号
    base = re.sub(r"\s+", "", base)
    t = TYPE_ALIASES.get(base, TYPE_ALIASES.get(base.lower()))
    if t is None:
        raise SystemExit(f"[field] cannot infer marker type from key {key!r}")
    return t


class FieldMap:
    """场地特征地图。"""

    def __init__(self, markers, version="unknown", size=None, source=""):
        self.markers = list(markers)
        self.version = str(version)
        self.size = size or {}
        self.source = str(source)
        self._by_type = {}
        for m in self.markers:
            self._by_type.setdefault(m.type, []).append(m)

    # ------------------------------------------------------------------
    @classmethod
    def load(cls, path):
        path = Path(path)
        if not path.exists():
            raise SystemExit(f"[field] map file not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}

        items = raw.get("markers") or raw.get("features") or []
        flat_mode = False
        if not items:
            # 兼容 Booster 官方格式：{ "TCross1": [x, y], ... } 扁平字典
            items = []
            for key, val in raw.items():
                if (isinstance(val, (list, tuple)) and len(val) >= 2
                        and all(isinstance(t, (int, float)) for t in val[:2])):
                    items.append({"id": str(key), "type": type_from_key(str(key)),
                                  "x": val[0], "y": val[1]})
            flat_mode = bool(items)
        if not items:
            raise SystemExit(f"[field] no 'markers' list (or official flat format) in {path}")
        if flat_mode:
            print(f"[field] parsed official flat format: {len(items)} entries")

        markers, problems, seen = [], [], set()
        for i, item in enumerate(items):
            if not isinstance(item, dict):
                problems.append(f"item #{i} is not a dict: {item!r}")
                continue
            raw_type = str(item.get("type", "")).strip()
            mtype = TYPE_ALIASES.get(raw_type, TYPE_ALIASES.get(raw_type.lower()))
            if mtype is None:
                problems.append(f"unknown type {raw_type!r} (allowed: {MARKER_TYPES})")
                continue
            try:
                x = float(item["x"])
                y = float(item["y"])
            except (KeyError, TypeError, ValueError):
                problems.append(f"missing/non-numeric x,y: {item!r}")
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                problems.append(f"x,y not finite: {item!r}")
                continue
            mid = str(item.get("id") or f"{mtype[0]}{i}")
            if mid in seen:
                problems.append(f"duplicate id: {mid}")
                continue
            seen.add(mid)
            markers.append(MapMarker(id=mid, type=mtype, x=x, y=y))

        if problems:
            raise SystemExit("[field] bad map file:\n  - " + "\n  - ".join(problems))
        if len(markers) < 3:
            print(f"[field] WARNING: only {len(markers)} markers, 2D pose may be ambiguous")

        size = raw.get("size") or {}
        version = str(raw.get("version") or "").strip()
        if not version:
            # 官方扁平格式没有 version 字段，用文件名 + SHA 前 8 位代替
            try:
                version = f"{path.name}:{sha256_of(path)[:8]}"
            except OSError:
                version = path.name
        fmap = cls(markers, version=version, size=size, source=path)
        fmap.min_inliers = int(raw.get("min_inliers", 3) or 3)
        return fmap

    # ------------------------------------------------------------------
    def by_type(self, mtype):
        return self._by_type.get(mtype, [])

    def types_present(self):
        return sorted(self._by_type.keys())

    def get(self, marker_id):
        for m in self.markers:
            if m.id == marker_id:
                return m
        return None

    def bbox(self, pad=0.5):
        xs = [m.x for m in self.markers]
        ys = [m.y for m in self.markers]
        return (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)

    def pairwise_distances(self):
        out = []
        for i in range(len(self.markers)):
            for j in range(i + 1, len(self.markers)):
                a, b = self.markers[i], self.markers[j]
                out.append((a.id, b.id, math.hypot(a.x - b.x, a.y - b.y)))
        return out

    def summary(self):
        lines = [
            f"map file    : {self.source}",
            f"version     : {self.version}",
            f"total markers: {len(self.markers)}",
        ]
        for t in self.types_present():
            ids = ", ".join(m.id for m in self.by_type(t))
            lines.append(f"  {t:<13} x{len(self.by_type(t))}  [{ids}]")
        return "\n".join(lines)


def sha256_of(path):
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


if __name__ == "__main__":
    import sys

    p = sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "config" / "field.yaml"
    fm = FieldMap.load(p)
    print(fm.summary())
    if Path(p).exists():
        print("SHA256:", sha256_of(p))