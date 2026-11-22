# -*- coding: utf-8 -*-
"""
按新坐标系生成场地地图（field.yaml + field_geometry.yaml）

坐标系定义（组长发布，见「场地坐标系示意图（俯视图）」）
------------------------------------------------------
    原点 O  ：场地中心（中圈圆心）
    +X      ：指向球门
    +Y      ：面向 +X 时的左侧
    +Z      ：垂直地面向上
    地面    ：Z = 0
    单位    ：米
    点球点  ：定在场地中心，即 (0, 0)

为什么需要这个工具
------------------
组长只给了坐标系定义，没有给坐标数值；图上最后一行明确写着
「场地尺寸与标志坐标由本人实测」。所以必须去场上量，然后把数字录进来。

量法（推荐）
------------
1. 在中圈圆心插一根标记当前原点。
2. 面朝球门站稳 —— 你正对的方向就是 +X，你的左手边就是 +Y。
3. 从原点出发，量每个标志在 +X 方向偏了多少、在 +Y 方向偏了多少。
   用两条互相垂直的卷尺（一条沿 X、一条沿 Y）对着量最快。
4. 记下 (X, Y)，回来填进输入文件。

输入文件格式（CSV，逗号或空格分隔，以 # 开头是注释）
----------------------------------------------------
    # 名称, 类型, X, Y
    LCross1, L, -3.200,  3.100
    LCross2, L,  3.200,  3.100
    TCross1, T,  0.000, -3.100
    TCross2, T,  2.500,  0.000

类型写 L / T（也接受 LCross / TCross / X / P / PenaltyPoint）。

用法
----
    # 生成地图并出预览图
    python scripts/build_field_map.py --from measurements.txt

    # 只校验不写文件
    python scripts/build_field_map.py --from measurements.txt --dry-run

    # 生成一份空白模板照抄
    python scripts/build_field_map.py --template > measurements.txt
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

TYPE_MAP = {
    "l": "LCross", "lcross": "LCross", "l_cross": "LCross", "l型": "LCross", "角": "LCross",
    "t": "TCross", "tcross": "TCross", "t_cross": "TCross", "t型": "TCross",
    "x": "XCross", "xcross": "XCross", "x_cross": "XCross", "x型": "XCross",
    "p": "PenaltyPoint", "penaltypoint": "PenaltyPoint", "penalitypoint": "PenaltyPoint",
    "点球点": "PenaltyPoint", "点": "PenaltyPoint",
}
TYPE_COLOR = {"LCross": (0, 165, 255), "TCross": (255, 170, 0),
              "XCross": (0, 255, 170), "PenaltyPoint": (255, 0, 255)}

TEMPLATE = """\
# ============================================================================
# 场地标志实测数据
#
# 坐标系：原点 = 场地中心（中圈圆心），+X 指向球门，+Y = 面向球门时的左侧，单位 米
# 格式  ：名称, 类型, X, Y       （以 # 开头是注释；逗号或空格分隔都行）
# 类型  ：L / T / X / P
#
# 点球点已按定义固定在 (0, 0)，不用填。
# 至少填 3 个点，且**同类型至少 2 个** —— 定位需要两个同类标志才能确定朝向。
# ============================================================================

# 名称,      类型,  X,       Y
# LCross1,   L,   -3.200,  3.100
# LCross2,   L,    3.200,  3.100
# TCross1,   T,    0.000, -3.100
# TCross2,   T,    2.500, -3.100
"""


def _read_text_any(path):
    """
    自动识别编码读取。

    重要：Windows PowerShell 5.1 用 `>` 重定向时写的是 UTF-16 LE（带 BOM），
    而 pwsh 7 / 记事本一般是 UTF-8。用户手写文件还可能是 ANSI(GBK)。
    这里逐个尝试，避免"文件明明有内容却读不出来"。
    """
    with open(path, "rb") as f:
        raw = f.read()
    for enc in ("utf-8-sig", "utf-16", "utf-16-le", "gbk", "utf-8"):
        try:
            return raw.decode(enc)
        except (UnicodeDecodeError, UnicodeError):
            continue
    return raw.decode("utf-8", errors="replace")


def _infer_type_from_name(name):
    """
    从名称推断类别。

    用户习惯直接写 `T1 3.98 1.97`（名称里已经带类型首字母），
    不必再单独写一列类型。这里按首字母判断：
        L / T / X / P（也认中文"角/丁/叉/点"）
    """
    s = str(name).strip().lower()
    if not s:
        return None
    # 去掉开头的编号，只保留字母/汉字部分
    head = "".join(ch for ch in s if ch.isalpha())
    if not head:
        return None
    c = head[0]
    return {"l": "LCross", "t": "TCross", "x": "XCross",
            "p": "PenaltyPoint", "点": "PenaltyPoint",
            "角": "LCross", "丁": "TCross", "叉": "XCross"}.get(c)


def parse_file(path):
    feats = []
    text = _read_text_any(path)
    if not text.strip():
        print(f"[警告] {path} 是空的")
    for ln_no, raw in enumerate(text.splitlines(), 1):
        s = raw.split("#")[0].strip()
        if not s:
            continue
        s = s.replace(",", " ").replace("\t", " ").replace("，", " ")
        parts = [p for p in s.split(" ") if p]
        if len(parts) < 3:
            print(f"[跳过] 第 {ln_no} 行字段不足: {raw.strip()}")
            continue

        name = parts[0]
        typ = None
        nums = None

        # 形式 A：名称, 类型, X, Y   （显式类型列）
        if len(parts) >= 4:
            t = TYPE_MAP.get(parts[1].strip().lower())
            if t is not None:
                typ = t
                nums = parts[2:4]
        # 形式 B：名称, X, Y        （类型从名称首字母推断）
        if typ is None:
            typ = _infer_type_from_name(name)
            nums = parts[1:3]
            if typ is None:
                print(f"[跳过] 第 {ln_no} 行无法判断类型（名称里应有 L/T/X/P 开头）: {raw.strip()}")
                continue
        try:
            x, y = float(nums[0]), float(nums[1])
        except (ValueError, IndexError):
            print(f"[跳过] 第 {ln_no} 行坐标不是数字: {raw.strip()}")
            continue
        feats.append({"name": name, "type": typ, "x": x, "y": y})
    return feats


def validate(feats):
    ok = True
    if len(feats) < 3:
        print(f"❌ 只有 {len(feats)} 个标志，至少需要 3 个（定位需 2 个同类 + 1 个校验）")
        ok = False

    by_type = {}
    for f in feats:
        by_type.setdefault(f["type"], []).append(f)
    for t, lst in by_type.items():
        if t == "PenaltyPoint":
            continue
        if len(lst) < 2:
            print(f"❌ {t} 只有 {len(lst)} 个。定位至少需要 2 个同类型标志才能定住朝向"
                  f"（细则 2.9：仅有一个标志通常不足以唯一确定二维位置和朝向）")
            ok = False

    # 重复坐标检查
    for i in range(len(feats)):
        for j in range(i + 1, len(feats)):
            d = math.hypot(feats[i]["x"] - feats[j]["x"], feats[i]["y"] - feats[j]["y"])
            if d < 0.25:
                print(f"⚠️ {feats[i]['name']} 与 {feats[j]['name']} 相距只有 {d:.2f} m，"
                      f"可能填错了")
                ok = False

    # 中点是否接近原点（场地特征通常关于中心对称）
    xs = [f["x"] for f in feats]
    ys = [f["y"] for f in feats]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    off = math.hypot(cx, cy)
    print(f"\n测量点包围框: X [{min(xs):+.2f}, {max(xs):+.2f}]  "
          f"Y [{min(ys):+.2f}, {max(ys):+.2f}]")
    print(f"包围框中心偏离原点 {off:.2f} m")
    if off > 1.0:
        print("⚠️ 偏离较大。若场地特征本来就不对称可忽略；"
              "若是量错，检查是否把原点当成场角而不是中心")
    return ok


def build_yaml(feats):
    # 官方格式：Name: [x, y]，load_field 直接支持
    return {f["name"]: [round(f["x"], 3), round(f["y"], 3)] for f in feats}


def build_geometry(feats):
    xs = [f["x"] for f in feats]
    ys = [f["y"] for f in feats]
    pad = 0.8
    return {
        "map_version": "C0 measured (personal)",
        "source": "本人卷尺实测，坐标系按组长发布定义",
        "demo": False,
        "verified": True,
        "field_name": "C0 field, origin at center circle, +X toward goal",
        "unit": "m",
        "origin": {"x": 0.0, "y": 0.0,
                   "description": "场地中心（中圈圆心）"},
        "axes": {"x": "指向球门", "y": "面向 +X 时的左侧", "yaw_positive": "从 +X 逆时针"},
        "bounds": {
            "min_x": round(min(xs) - pad, 2), "max_x": round(max(xs) + pad, 2),
            "min_y": round(min(ys) - pad, 2), "max_y": round(max(ys) + pad, 2),
        },
        "ground_plane_z": 0.0,
        "notes": "坐标由本人卷尺实测。点球点按定义固定在场地中心 (0,0)。",
    }


def render(feats, out_png):
    xs = [f["x"] for f in feats]
    ys = [f["y"] for f in feats]
    minx, maxx = min(xs) - 1.0, max(xs) + 1.0
    miny, maxy = min(ys) - 1.0, max(ys) + 1.0
    W, H, M = 1100, 900, 80
    s = min((W - 2 * M) / (maxx - minx), (H - 2 * M) / (maxy - miny))

    def px(x, y):
        return (int(M + (x - minx) * s), int(H - M - (y - miny) * s))

    img = np.full((H, W, 3), 30, np.uint8)

    # 网格 1 m
    for gx in range(math.ceil(minx), int(maxx) + 1):
        cv2.line(img, px(gx, miny), px(gx, maxy), (55, 58, 64), 1)
        p = px(gx, miny)
        cv2.putText(img, str(gx), (p[0] - 8, p[1] + 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (130, 135, 140), 1)
    for gy in range(math.ceil(miny), int(maxy) + 1):
        cv2.line(img, px(minx, gy), px(maxx, gy), (55, 58, 64), 1)
        p = px(minx, gy)
        cv2.putText(img, str(gy), (p[0] - 34, p[1] + 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (130, 135, 140), 1)

    # 坐标轴
    o = px(0, 0)
    L = int(1.2 * s)
    cv2.arrowedLine(img, o, (o[0] + L, o[1]), (90, 160, 250), 3, tipLength=0.22)
    cv2.arrowedLine(img, o, (o[0], o[1] - L), (250, 160, 90), 3, tipLength=0.22)
    cv2.putText(img, "+X -> goal", (o[0] + L + 8, o[1] + 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 160, 250), 2)
    cv2.putText(img, "+Y", (o[0] + 6, o[1] - L - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (250, 160, 90), 2)
    cv2.circle(img, o, 7, (255, 255, 255), 2)

    for f in feats:
        p = px(f["x"], f["y"])
        c = TYPE_COLOR.get(f["type"], (200, 200, 200))
        cv2.drawMarker(img, p, c, cv2.MARKER_DIAMOND, 16, 3)
        cv2.putText(img, f"{f['name']} ({f['x']:+.2f},{f['y']:+.2f})",
                    (p[0] + 12, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
    cv2.putText(img, "C0 field map (measured, origin = center circle, +X toward goal)",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    cv2.imwrite(out_png, img)
    print(f"\n预览图 -> {out_png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="src", default=None, help="测量数据文件")
    # 默认写到 c0_field*.yaml，不覆盖 config/field.yaml（那是 demo 合成场景用的）
    ap.add_argument("--out-field",
                    default=os.path.join(ROOT, "config", "c0_field.yaml"))
    ap.add_argument("--out-geometry",
                    default=os.path.join(ROOT, "config", "c0_field_geometry.yaml"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--template", action="store_true", help="打印空白模板")
    args = ap.parse_args()

    if args.template:
        print(TEMPLATE, end="")
        return 0
    if not args.src:
        print("需要 --from <文件>，或用 --template 生成模板")
        return 2

    feats = parse_file(args.src)
    print(f"读到 {len(feats)} 个标志")
    for f in feats:
        print(f"  {f['name']:<12} {f['type']:<13} ({f['x']:+.3f}, {f['y']:+.3f})")

    # 点球点按定义固定在原点
    if not any(f["type"] == "PenaltyPoint" for f in feats):
        feats.append({"name": "PenalityPoint", "type": "PenaltyPoint", "x": 0.0, "y": 0.0})
        print("已按定义补上点球点 (0, 0)（命名用官方拼写 PenalityPoint）")

    ok = validate(feats)
    if args.dry_run:
        print("\n--dry-run：未写文件")
        return 0 if ok else 1
    if not ok:
        print("\n校验未通过，仍会写文件但请先修正数据。")

    fy = build_yaml(feats)
    fg = build_geometry(feats)
    os.makedirs(os.path.dirname(args.out_field), exist_ok=True)
    with open(args.out_field, "w", encoding="utf-8") as f:
        yaml.safe_dump(fy, f, allow_unicode=True, sort_keys=False)
    print(f"\n已写 {args.out_field}（{len(fy)} 个特征）")
    with open(args.out_geometry, "w", encoding="utf-8") as f:
        yaml.safe_dump(fg, f, allow_unicode=True, sort_keys=False)
    print(f"已写 {args.out_geometry}")

    render(feats, os.path.join(ROOT, "snapshots", "field_map_measured.png"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
