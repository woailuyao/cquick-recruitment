#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
field_map.py —— 读取 field.yaml（场地标志地图）并做类别归一化

field.yaml 采用组织方/官方 demo 的格式：一个扁平字典，"标志名 -> [x, y]"，
单位米，坐标原点与轴向由地图定义：

    TCross1: [5.955, 2.930]
    LCross2: [-1.485, 2.930]
    XCross1: [4.505, -4.010]
    PenalityPoint: [2.965, 0.885]     # 注意官方原拼写就是 PenalityPoint（少一个 t）

类别名与任务要求对齐（class_id 固定）：

    0  LCross
    1  TCross
    2  XCross
    3  PenaltyPoint

所以读取时必须把 "PenalityPoint" 归一化成 "PenaltyPoint"，否则点球点永远匹配不上。
"""

import hashlib
import os
import re

CLASS_NAMES = ["LCross", "TCross", "XCross", "PenaltyPoint"]
CLASS_ID = {n: i for i, n in enumerate(CLASS_NAMES)}

# 各种可能出现的拼写/别名 -> 标准名
_ALIASES = {
    "penalitypoint": "PenaltyPoint",
    "penaltypenaltypoint": "PenaltyPoint",
    "penaltypoint": "PenaltyPoint",
    "penalty": "PenaltyPoint",
    "lcross": "LCross",
    "l": "LCross",
    "tcross": "TCross",
    "t": "TCross",
    "xcross": "XCross",
    "x": "XCross",
    "cross": "XCross",
}


def _canonical_class_of(name):
    """从标志名（如 TCross3、PenalityPoint）里解析出类别名。"""
    low = re.sub(r"[^a-z]", "", name.lower())
    # 先做精确别名匹配，再退回前缀匹配
    if low in _ALIASES:
        return _ALIASES[low]
    for key, cls in (("penality", "PenaltyPoint"), ("penalty", "PenaltyPoint"),
                     ("lcross", "LCross"), ("tcross", "TCross"), ("xcross", "XCross")):
        if low.startswith(key):
            return cls
    return None


class FieldMap:
    """场地标志地图。"""

    def __init__(self, markers, source_path=None, sha256=None):
        # markers: [(name, class_name, class_id, x, y)]
        self.markers = list(markers)
        self.source_path = source_path
        self.path = source_path
        self.sha256 = sha256
        self.version = ""

    # ------------------------------------------------------------------
    @classmethod
    def from_yaml(cls, path):
        import yaml
        with open(path, "rb") as f:
            raw_bytes = f.read()
        sha = hashlib.sha256(raw_bytes).hexdigest()
        data = yaml.safe_load(raw_bytes.decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("%s 不是 '名称: [x, y]' 形式的字典" % path)

        markers, skipped = [], []
        for name, val in data.items():
            cls_name = _canonical_class_of(str(name))
            if cls_name is None:
                skipped.append(str(name))
                continue
            try:
                x, y = float(val[0]), float(val[1])
            except Exception:
                try:
                    x, y = float(val["x"]), float(val["y"])
                except Exception:
                    skipped.append(str(name))
                    continue
            markers.append((str(name), cls_name, CLASS_ID[cls_name], x, y))
        if skipped:
            print("[field_map] 忽略无法归类/解析的键: %s" % ", ".join(skipped))
        if not markers:
            raise ValueError("%s 里没有可用的标志点" % path)
        obj = cls(markers, source_path=path, sha256=sha)
        # 版本号写在 yaml 注释里（make_field_map.py 生成），形如 "# version: cquick-field-map/3"
        try:
            for line in raw_bytes.decode("utf-8", "replace").splitlines():
                t = line.strip().lstrip("#").strip()
                for key in ("version:", u"版本:"):
                    if t.lower().startswith(key):
                        obj.version = t.split(":", 1)[1].strip()
                        break
                if obj.version:
                    break
        except Exception:
            pass
        return obj

    # ------------------------------------------------------------------
    @property
    def points(self):
        import numpy as np
        return np.array([[m[3], m[4]] for m in self.markers], dtype=float)

    @property
    def class_ids(self):
        import numpy as np
        return np.array([m[2] for m in self.markers], dtype=int)

    def names(self):
        return [m[0] for m in self.markers]

    @property
    def shorthash(self):
        """sha256 前 8 位，画在俯视图标题上，方便核对"现场用的是哪版地图"。"""
        return (self.sha256 or "?")[:8]

    def class_id_of(self, name):
        """按标志名查类别 id（画图用）。"""
        for m in self.markers:
            if m[0] == name:
                return m[2]
        return -1

    def bounds(self, margin=0.5):
        pts = self.points
        return (pts[:, 0].min() - margin, pts[:, 1].min() - margin,
                pts[:, 0].max() + margin, pts[:, 1].max() + margin)

    def summary(self):
        from collections import Counter
        c = Counter(CLASS_NAMES[m[2]] for m in self.markers)
        return "field.yaml: %d 个标志 (%s), sha256=%s" % (
            len(self.markers),
            ", ".join("%s×%d" % (k, v) for k, v in sorted(c.items())),
            (self.sha256 or "?")[:12])


def load_field(path):
    return FieldMap.from_yaml(path)


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        sys.exit("用法: python -m cquick_field_localizer.field_map config/field.yaml")
    fm = load_field(sys.argv[1])
    print(fm.summary())
    for name, cls_name, cid, x, y in fm.markers:
        print("  %-14s %-12s id=%d  (%7.3f, %7.3f)" % (name, cls_name, cid, x, y))
