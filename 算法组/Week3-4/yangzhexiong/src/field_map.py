"""field.yaml 加载与校验：发现重复坐标、越界、未知类别、格式错误。

用法：
    python3 field_map.py                       # 校验 config/field.yaml
    python3 field_map.py --path field.yaml --plot

产物：
    outputs/field_preview.png   场地俯视图（带 id 标注）
"""
import argparse
import os
from collections import defaultdict

import cv2
import numpy as np
import yaml

KNOWN_TYPES = {"LCross", "TCross", "XCross", "PenaltyPoint"}
ALIASES = {"PenalityPoint": "PenaltyPoint", "penaltypoint": "PenaltyPoint"}


class FieldMap:
    def __init__(self, data):
        self.meta = data.get("meta", {})
        self.field = data.get("field", {})
        self.markers = []
        for m in data.get("markers", []):
            typ = ALIASES.get(m.get("type", ""), m.get("type", ""))
            self.markers.append({
                "id": m.get("id", "?"),
                "type": typ,
                "x": float(m["x"]),
                "y": float(m["y"]),
                "yaw": float(m.get("yaw", 0.0)),
            })

    def by_type(self, typ):
        return [m for m in self.markers if m["type"] == typ]


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "markers" not in data:
        raise ValueError("缺少 markers 字段")
    return FieldMap(data)


def validate(fm):
    problems, warns = [], []

    # 类别
    for m in fm.markers:
        if m["type"] not in KNOWN_TYPES:
            problems.append(f"{m['id']}: 未知类别 {m['type']}")

    # 重复坐标（同类型、坐标接近 → 无法消歧）
    groups = defaultdict(list)
    for m in fm.markers:
        groups[(m["type"], round(m["x"], 2), round(m["y"], 2))].append(m["id"])
    for (typ, x, y), ids in groups.items():
        if len(ids) > 1:
            problems.append(f"{typ} 在 ({x}, {y}) 有 {len(ids)} 个重复点位：{ids}")

    # 跨类别同坐标（可能是复制粘贴错误）
    pos = defaultdict(list)
    for m in fm.markers:
        pos[(round(m["x"], 2), round(m["y"], 2))].append((m["id"], m["type"]))
    for (x, y), items in pos.items():
        types = {t for _, t in items}
        if len(items) > 1 and len(types) > 1:
            warns.append(f"({x}, {y}) 同时存在多个不同类别：{items}（请与场地核对）")

    # 越界
    b = fm.field.get("boundary")
    if b:
        for m in fm.markers:
            if not (b["x_min"] - 1e-6 <= m["x"] <= b["x_max"] + 1e-6 and
                    b["y_min"] - 1e-6 <= m["y"] <= b["y_max"] + 1e-6):
                warns.append(f"{m['id']} 在边界外：({m['x']}, {m['y']})")

    # 类别覆盖
    counts = defaultdict(int)
    for m in fm.markers:
        counts[m["type"]] += 1
    if counts.get("LCross", 0) < 2:
        warns.append("LCross 少于 2 个，单目位姿估计可能不可解")
    if len(set((m["type"], round(m["x"], 2), round(m["y"], 2)) for m in fm.markers)) < 3:
        problems.append("唯一标志点少于 3 个，无法唯一确定 2D 位姿")

    return problems, warns, counts


def plot(fm, out="outputs/field_preview.png"):
    f = fm.field
    L = f.get("length", 2 * f.get("half_length", 4.0))
    W = f.get("width", 2 * f.get("half_width", 3.0))
    scale = 100  # px/m
    pad = 60
    img_w, img_h = int(W * scale) + 2 * pad, int(L * scale) + 2 * pad
    img = np.full((img_h, img_w, 3), 40, np.uint8)

    def to_px(x, y):
        # 俯视图：X 向上（屏幕 y 减小），Y 向左（屏幕 x 减小）
        return int(pad + (W / 2 - y) * scale), int(pad + (L / 2 - x) * scale)

    p1, p2 = to_px(-L / 2, W / 2), to_px(L / 2, -W / 2)
    cv2.rectangle(img, p1, p2, (0, 160, 0), 2)

    colors = {"LCross": (0, 200, 255), "TCross": (255, 160, 0),
              "XCross": (0, 100, 255), "PenaltyPoint": (200, 0, 255)}
    for m in fm.markers:
        p = to_px(m["x"], m["y"])
        cv2.circle(img, p, 6, colors.get(m["type"], (255, 255, 255)), -1)
        cv2.putText(img, f"{m['id']}", (p[0] + 8, p[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.putText(img, f"({m['x']:.1f},{m['y']:.1f})", (p[0] + 8, p[1] + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1)

    o = to_px(0, 0)
    cv2.arrowedLine(img, o, (o[0], o[1] - 40), (255, 255, 255), 2, tipLength=0.3)
    cv2.putText(img, "+X", (o[0] + 6, o[1] - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.arrowedLine(img, o, (o[0] - 40, o[1]), (255, 255, 255), 2, tipLength=0.3)
    cv2.putText(img, "+Y", (o[0] - 56, o[1] + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    cv2.imwrite(out, img)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", default="config/field.yaml")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(args.path):
        print(f"找不到 {args.path}")
        return 1
    try:
        fm = load(args.path)
    except Exception as exc:  # noqa: BLE001
        print(f"解析失败：{exc}")
        print("常见原因：YAML 缩进不一致（列表项前面只用了一个空格）")
        return 1

    print(f"单位：{fm.meta.get('unit', '?')}  坐标系：{fm.meta.get('frame_id', '?')}")
    print(f"场地：{fm.field.get('length', '?')} x {fm.field.get('width', '?')} m")
    print(f"标志：{len(fm.markers)} 个\n")

    problems, warns, counts = validate(fm)
    print("类别统计：", dict(counts))
    for m in fm.markers:
        print(f"  {m['id']:<18} {m['type']:<14} ({m['x']:+.2f}, {m['y']:+.2f})")

    if problems:
        print("\n【错误】必须修好才能用于定位：")
        for p in problems:
            print("  ✗", p)
    if warns:
        print("\n【提醒】建议与场地核对：")
        for w in warns:
            print("  !", w)
    if not problems and not warns:
        print("\n校验通过")

    if args.plot:
        out = plot(fm)
        print(f"\n俯视图：{out}")

    return 0 if not problems else 2


if __name__ == "__main__":
    raise SystemExit(main())
