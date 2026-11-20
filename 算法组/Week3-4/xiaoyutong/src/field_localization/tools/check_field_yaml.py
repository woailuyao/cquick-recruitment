"""检查 config/field.yaml 并画出场地俯视图。

用途：确认"地图和脑子里的场地是不是一个东西"，顺便查坐标有没有明显错误。

用法：
    python tools/check_field_yaml.py
    python tools/check_field_yaml.py --yaml config/field.yaml --out config/field_map.png
"""

import argparse
import os
import sys

import cv2
import numpy as np

def _repo_root():
    """向上找到包含 config/field.yaml 的目录，作为仓库根。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        if os.path.exists(os.path.join(d, "config", "field.yaml")):
            return d
        d = os.path.dirname(d)
    return os.getcwd()

ROOT = _repo_root()
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision2.console import setup_console  # noqa: E402
from vision2.io_utils import imwrite  # noqa: E402
from vision2.yaml_mini import load_yaml  # noqa: E402

setup_console()

TYPE_COLOR = {
    "T": (60, 200, 60),
    "L": (0, 200, 255),
    "X": (255, 120, 60),
    "P": (0, 0, 255),
}


def kind_of(key):
    if key.startswith("TCross"):
        return "T"
    if key.startswith("LCross"):
        return "L"
    if key.startswith("XCross"):
        return "X"
    if key in ("PenalityPoint", "PenaltyPoint"):
        return "P"
    if key == "CenterSpot":
        return "C"
    return "?"


def draw_map(points, geom, out_path):
    depth = geom["field"]["depth_m"]
    ys = [p[1] for p in points.values()]
    y_min, y_max = min(ys), max(ys)
    x_min_draw, x_max_draw = -1.0, depth + 1.0
    y_min_draw, y_max_draw = y_min - 0.6, y_max + 0.6

    scale = 110.0
    W = int((x_max_draw - x_min_draw) * scale)
    H = int((y_max_draw - y_min_draw) * scale)
    img = np.full((H, W, 3), 32, np.uint8)

    def px(x, y):
        return (int(round((x - x_min_draw) * scale)), int(round(H - (y - y_min_draw) * scale)))

    # 半场外的区域（x<0，不在考核范围）
    cv2.rectangle(img, px(x_min_draw, y_min_draw), px(0, y_max_draw), (55, 55, 55), -1)
    cv2.putText(img, "out of pitch (x<0)", px(x_min_draw + 0.1, y_max_draw - 0.35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (110, 110, 110), 1, cv2.LINE_AA)

    # 半场底色
    cv2.rectangle(img, px(0, y_min), px(depth, y_max), (28, 88, 42), -1)
    # 外框 + 中线
    for a, b in [((0, y_min), (depth, y_min)), ((0, y_max), (depth, y_max)),
                 ((0, y_min), (0, y_max)), ((depth, y_min), (depth, y_max))]:
        cv2.line(img, px(*a), px(*b), (225, 225, 225), 2, cv2.LINE_AA)

    # 中圈（整圆，一半在场外）
    r = geom["field"]["center_circle_radius_m"]
    cv2.circle(img, px(0, 0), int(r * scale), (225, 225, 225), 2, cv2.LINE_AA)

    # 球门区（用图里的 T/L 点推出来的矩形）
    ly = [points[k][1] for k in ("LCross3", "LCross4") if k in points]
    lx = [points[k][0] for k in ("LCross3", "LCross4") if k in points]
    if ly and lx:
        cv2.rectangle(img, px(min(lx), min(ly)), px(depth, max(ly)), (225, 225, 225), 2, cv2.LINE_AA)

    # 球门
    gy = [points[k][1] for k in ("TCross2", "TCross3") if k in points]
    if gy:
        cv2.line(img, px(depth, min(gy)), px(depth, max(gy)), (0, 200, 255), 6, cv2.LINE_AA)
        cv2.putText(img, "goal", (px(depth, min(gy))[0] + 8, px(depth, 0)[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)

    # 坐标轴
    ox, oy = px(1.0, -0.9)
    cv2.arrowedLine(img, (ox, oy), (ox + int(0.9 * scale), oy), (0, 0, 255), 2, tipLength=0.2)
    cv2.putText(img, "+X", (ox + int(0.95 * scale), oy + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.arrowedLine(img, (ox, oy), (ox, oy - int(0.9 * scale)), (255, 80, 80), 2, tipLength=0.2)
    cv2.putText(img, "+Y", (ox + 6, oy - int(0.95 * scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 80, 80), 2)

    # 特征点
    for k, (x, y) in sorted(points.items()):
        t = kind_of(k)
        c = TYPE_COLOR.get(t, (200, 200, 200))
        p = px(x, y)
        cv2.circle(img, p, 6, c, -1, cv2.LINE_AA)
        cv2.circle(img, p, 6, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, k.replace("Cross", ""), (p[0] + 8, p[1] - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, c, 1, cv2.LINE_AA)

    # 图例
    yy = 22
    for t, name in (("T", "T 型交点"), ("L", "L 型拐角"), ("X", "X 型交点"), ("P", "罚球点")):
        cv2.circle(img, (16, yy - 5), 6, TYPE_COLOR[t], -1, cv2.LINE_AA)
        cv2.putText(img, "%s (%d)" % (name, sum(1 for k in points if kind_of(k) == t)),
                    (30, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
        yy += 20
    cv2.putText(img, "C0 half pitch %.2f x %.2f m | origin = centre circle | +X to goal" %
                (depth, geom["field"]["width_m"]), (30, yy + 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    imwrite(out_path, img)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default=os.path.join(ROOT, "config", "field.yaml"))
    ap.add_argument("--geometry", default=os.path.join(ROOT, "config", "field_geometry.yaml"))
    ap.add_argument("--out", default=os.path.join(ROOT, "config", "field_map.png"))
    args = ap.parse_args()

    data = load_yaml(args.yaml)
    geom = load_yaml(args.geometry)

    points = {}
    for k, v in data.items():
        if isinstance(v, (list, tuple)) and len(v) == 2:
            points[k] = (float(v[0]), float(v[1]))

    depth = geom["field"]["depth_m"]
    width = geom["field"]["width_m"]
    print("== 场地 ==")
    print("  深度(中线→球门线) %.2f m，宽度 %.2f m，中圈半径 %.2f m"
          % (depth, width, geom["field"]["center_circle_radius_m"]))
    print("  原点：%s；+X %s；+Y %s" % (geom["origin"], geom["x_axis"], geom["y_axis"]))
    print("")

    print("== 特征点（%d 个）==" % len(points))
    groups = {}
    for k, v in points.items():
        groups.setdefault(kind_of(k), []).append((k, v))
    for t in ("T", "L", "X", "P", "C", "?"):
        if t not in groups:
            continue
        print("  [%s] %d 个" % (t, len(groups[t])))
        for k, (x, y) in groups[t]:
            print("      %-14s x=%6.3f  y=%6.3f" % (k, x, y))
    print("")

    # 检查
    problems = []
    ys = [p[1] for p in points.values()]
    if abs(min(ys) + max(ys)) > 0.15:
        problems.append("左右不对称：y 范围 %.2f ~ %.2f，中线可能没在最中间" % (min(ys), max(ys)))
    for k, (x, y) in points.items():
        if x < -0.05 or x > depth + 0.05:
            problems.append("%s 在场地外（x=%.2f，应在 0~%.2f）" % (k, x, depth))
        if abs(y) > width / 2 + 0.15:
            problems.append("%s 在场地外（y=%.2f，应在 ±%.2f 内）" % (k, y, width / 2))
    seen = {}
    for k, (x, y) in points.items():
        key = (round(x, 2), round(y, 2))
        if key in seen:
            problems.append("%s 和 %s 坐标重复：(%.2f, %.2f)" % (k, seen[key], x, y))
        seen[key] = k

    if problems:
        print("== 发现 %d 个问题 ==" % len(problems))
        for p in problems:
            print("  [注意] " + p)
    else:
        print("== 检查通过：没有越界、没有重复坐标 ==")

    draw_map(points, geom, args.out)
    print("")
    print("  俯视图已保存：" + args.out)
    print("  打开看看形状对不对（球门在右边、中圈在左边、+X 朝球门）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
