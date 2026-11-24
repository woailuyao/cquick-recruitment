#!/usr/bin/env python3

import argparse
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(ROOT, "config", "field.yaml")
TYPES = ("LCross", "TCross", "XCross", "PenaltyPoint")

def load(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)

def save(path, data):
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

def cmd_new(args):
    if os.path.exists(args.out) and os.path.getsize(args.out) > 0 and not args.force:
        print(f"[拒绝] {args.out} 已存在内容，加 --force 才会覆盖")
        return 1
    data = {
        "frame": "map",
        "unit": "m",
        "map_version": args.version,
        "origin": args.origin,
        "axis": {
            "x_direction": args.x_dir,
            "y_direction": args.y_dir,
            "yaw_positive": "逆时针为正",
        },
        "field_size": {"length_m": args.length, "width_m": args.width},
        "markers": [],
    }
    save(args.out, data)
    print(f"已新建 {args.out}")
    print("下一步用 add 逐个加标志点；加完跑 plot 看俯视图。")
    return 0

def cmd_add(args):
    if args.type not in TYPES:
        print(f"[错误] 类别只能是 {TYPES}")
        return 1
    data = load(args.out) or {"frame": "map", "unit": "m", "markers": []}
    data.setdefault("markers", []).append(
        {"type": args.type, "x": args.x, "y": args.y, **({"note": args.note} if args.note else {})}
    )
    save(args.out, data)
    print(f"已添加 {args.type} ({args.x}, {args.y})，现在共 {len(data['markers'])} 个点")
    return 0

def cmd_remove(args):
    data = load(args.out)
    if not data or not data.get("markers"):
        print("[错误] 文件里没有标志点")
        return 1
    idx = args.index - 1
    if idx < 0 or idx >= len(data["markers"]):
        print(f"[错误] 序号超出范围 1~{len(data['markers'])}")
        return 1
    removed = data["markers"].pop(idx)
    save(args.out, data)
    print(f"已删除第 {args.index} 个：{removed}")
    return 0

def cmd_list(args):
    data = load(args.out)
    if not data:
        print("文件为空或不存在")
        return 1
    print(f"地图：{data.get('map_version', '未命名')}  单位：{data.get('unit', '?')}")
    print(f"origin: {data.get('origin', '未填')}")
    print(f"field_size: {data.get('field_size')}")
    markers = data.get("markers") or []
    if not markers:
        print("（还没有标志点）")
        return 0
    print(f"{'序号':<4}{'类别':<14}{'x':>8}{'y':>8}  备注")
    for i, m in enumerate(markers, 1):
        print(f"{i:<4}{m.get('type', '?'):<14}{m.get('x', 0):>8.3f}{m.get('y', 0):>8.3f}  {m.get('note', '')}")
    return 0

def cmd_check(args):
    data = load(args.out)
    if not data:
        print("[错误] 文件为空或不存在")
        return 1
    markers = data.get("markers") or []
    problems = []
    if not markers:
        problems.append("没有任何标志点")
    seen = {}
    for i, m in enumerate(markers, 1):
        t = m.get("type")
        if t not in TYPES:
            problems.append(f"第 {i} 个类别不对：{t}")
        key = (t, round(float(m.get("x", 0)), 3), round(float(m.get("y", 0)), 3))
        if key in seen:
            problems.append(f"第 {i} 个与第 {seen[key]} 个重复：{key}")
        seen[key] = i

    size = data.get("field_size") or {}
    length = float(size.get("length_m", 0) or 0)
    width = float(size.get("width_m", 0) or 0)
    if length > 0 and width > 0:
        for i, m in enumerate(markers, 1):
            x, y = float(m.get("x", 0)), float(m.get("y", 0))
            if not (0 <= x <= length and 0 <= y <= width):
                problems.append(f"第 {i} 个超出场地范围：({x}, {y}) 不在 0~{length} / 0~{width} 内")

    if problems:
        print("发现问题：")
        for p in problems:
            print(" -", p)
        return 1
    print(f"检查通过：{len(markers)} 个标志点，坐标都在场地范围内。")
    return 0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=DEFAULT_OUT, help="输出文件，默认 config/field.yaml")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("new")
    p.add_argument("--length", type=float, required=True, help="场地长（米）")
    p.add_argument("--width", type=float, required=True, help="场地宽（米）")
    p.add_argument("--origin", default="待填")
    p.add_argument("--x-dir", default="待填")
    p.add_argument("--y-dir", default="待填")
    p.add_argument("--version", default="C0-self")
    p.add_argument("--force", action="store_true")
    p.set_defaults(func=cmd_new)

    p = sub.add_parser("add")
    p.add_argument("type")
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("--note", default="")
    p.set_defaults(func=cmd_add)

    p = sub.add_parser("remove")
    p.add_argument("index", type=int)
    p.set_defaults(func=cmd_remove)

    p = sub.add_parser("list")
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("check")
    p.set_defaults(func=cmd_check)

    args = ap.parse_args()
    return args.func(args)

if __name__ == "__main__":
    sys.exit(main())
