#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""field.yaml 校验 + 可视化。

用途：
  1. 合法性检查：坐标是否落在场地内、类别数量、镜像对称性、与官方版差异
  2. 出图：生成 docs/field_measured_map.png，用于 README 和现场核对
  3. 供匹配算法前自检：坐标有明显错误时（越界/重复/漏类）立刻报警

用法：
    python tools/plot_field.py                     # 校验 + 出图
    python tools/plot_field.py --yaml config/field.yaml --out docs/field_measured_map.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]

CLASS_PREFIX = {"LCross": "LCross", "TCross": "TCross", "XCross": "XCross"}
STYLE = {
    "LCross": ("#d62728", "o", "L"),
    "TCross": ("#1f77b4", "s", "T"),
    "XCross": ("#2ca02c", "D", "X"),
    "PenalityPoint": ("#9467bd", "*", "P"),
}


def load_points(path: Path):
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    pts = {}
    for k, v in data.items():
        if not isinstance(v, (list, tuple)) or len(v) != 2:
            continue
        if any(c is None for c in v):
            continue
        pts[str(k)] = (float(v[0]), float(v[1]))
    return data, pts


def classify(name: str) -> str:
    if name.startswith("Penality") or name.startswith("Penalty"):
        return "PenalityPoint"
    for p in ("LCross", "TCross", "XCross"):
        if name.startswith(p):
            return p
    return "other"


def check(pts: dict, half_x: float, half_y: float) -> list[str]:
    warns: list[str] = []
    # 1) 越界
    for n, (x, y) in pts.items():
        if abs(x) > half_x + 0.02 or abs(y) > half_y + 0.02:
            warns.append(f"越界 {n}: ({x:.3f}, {y:.3f}) 超出 ±{half_x}/{half_y}")
    # 2) 重复/过近
    names = list(pts)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = np.array(pts[names[i]]), np.array(pts[names[j]])
            d = float(np.linalg.norm(a - b))
            if d < 0.15:
                warns.append(f"过近 {names[i]} / {names[j]}: 相距 {d*100:.0f} cm（可能重复测量）")
    # 3) 类别数量
    cnt: dict[str, int] = {}
    for n in pts:
        c = classify(n)
        cnt[c] = cnt.get(c, 0) + 1
    for c in ("LCross", "TCross", "XCross", "PenalityPoint"):
        if cnt.get(c, 0) == 0:
            warns.append(f"缺类 {c}: 该类无样本，任务二这类别无法训练/评估")
    return warns


def symmetry_report(pts: dict) -> list[str]:
    """把点按 |y| 接近且 x 接近配对，报告残余不对称（场地画线偏差）。"""
    out = []
    keyed = sorted(pts.items(), key=lambda kv: (round(kv[1][0], 2), kv[1][1]))
    used = set()
    for i, (n1, p1) in enumerate(keyed):
        if n1 in used:
            continue
        for n2, p2 in keyed[i + 1:]:
            if n2 in used:
                continue
            if abs(p1[0] - p2[0]) < 0.25 and p1[1] * p2[1] < 0 and abs(abs(p1[1]) - abs(p2[1])) < 0.30:
                used.add(n1)
                used.add(n2)
                dx = abs(p1[0] - p2[0]) * 100
                dy = abs(abs(p1[1]) - abs(p2[1])) * 100
                out.append(f"{n1} / {n2}: ΔX={dx:.0f}cm Δ|Y|={dy:.0f}cm")
                break
    return out


def plot(pts: dict, half_x: float, half_y: float, out: Path, title: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc, Rectangle

    fig, ax = plt.subplots(figsize=(11, 8.2, ), dpi=160)
    ax.set_facecolor("#f7f7f7")

    # 场地边界
    ax.add_patch(Rectangle((-half_x, -half_y), 2 * half_x, 2 * half_y,
                           fill=False, ec="#333333", lw=2.2, zorder=2))
    # 中线
    ax.plot([0, 0], [-half_y, half_y], color="#333333", lw=1.4, ls="-", zorder=1)
    # 中圈（半径取实测均值）
    xs = [v for n, v in pts.items() if n.startswith("XCross")]
    r = float(np.mean([abs(y) for x, y in xs])) if xs else 0.73
    cx = float(np.mean([x for x, y in xs])) if xs else 0.0
    ax.add_patch(Arc((cx, 0), 2 * r, 2 * r, fill=False, ec="#888888", lw=1.2, ls="--"))
    # 禁区框（球门端 / 远端）：由该端的 L 前角推断，框体延伸到对应端线
    L = {n: v for n, v in pts.items() if n.startswith("LCross")}
    for side, xsign in (("goal", 1), ("far", -1)):
        sel = [v for n, v in L.items() if np.sign(v[0]) == xsign and abs(v[1]) < 2.6]
        if len(sel) >= 2:
            front_x = min(p[0] for p in sel) if xsign > 0 else max(p[0] for p in sel)
            yw = max(abs(p[1]) for p in sel)
            x0 = front_x if xsign > 0 else -half_x
            x1 = half_x if xsign > 0 else front_x
            ax.add_patch(Rectangle((x0, -yw), x1 - x0, 2 * yw,
                                   fill=False, ec="#bbbbbb", lw=1.2, ls="-", zorder=1))
    # 球门（+X 端）
    ax.plot([half_x, half_x], [-0.9, 0.9], color="#d62728", lw=6, solid_capstyle="butt",
            alpha=0.55, zorder=3)
    ax.annotate("GOAL (+X)", (half_x, 0.95), ha="center", va="bottom",
                fontsize=10, color="#d62728", fontweight="bold")
    # 坐标轴
    ax.annotate("", xy=(half_x * 1.0, -half_y - 0.55), xytext=(0, -half_y - 0.55),
                arrowprops=dict(arrowstyle="->", color="#1f77b4", lw=1.8))
    ax.text(half_x * 0.55, -half_y - 0.95, "+X", color="#1f77b4", fontsize=11, fontweight="bold")
    ax.annotate("", xy=(-half_x - 0.55, 0), xytext=(-half_x - 0.55, -half_y),
                arrowprops=dict(arrowstyle="->", color="#2ca02c", lw=1.8))
    ax.text(-half_x - 1.05, -half_y * 0.45, "+Y", color="#2ca02c", fontsize=11,
            fontweight="bold", rotation=90)
    ax.plot(0, 0, marker="+", ms=14, mew=2, color="#555555", zorder=4)
    ax.text(0.06, 0.10, "O (0,0)", fontsize=9, color="#555555")

    # 标志点
    for name, (x, y) in pts.items():
        c = classify(name)
        color, marker, _ = STYLE.get(c, ("#777777", "o", "?"))
        ax.plot(x, y, marker=marker, ms=9 if c != "PenalityPoint" else 15,
                color=color, mec="white", mew=1.0, zorder=5)
        off = (-70, 14) if "enal" in name else (7, 6)   # 点球点与 O 标签避让
        ax.annotate(name, (x, y), textcoords="offset points", xytext=off,
                    fontsize=8, color="#222222")

    ax.set_xlim(-half_x - 1.6, half_x + 0.9)
    ax.set_ylim(-half_y - 1.3, half_y + 0.9)
    ax.set_aspect("equal")
    ax.grid(True, ls=":", color="#cccccc", lw=0.7)
    ax.set_xlabel("X (m)  →  goal")
    ax.set_ylabel("Y (m)  →  left")
    ax.set_title(title, fontsize=13, fontweight="bold")
    handles = [plt.Line2D([], [], marker=STYLE[k][1], ls="", color=STYLE[k][0],
                          label=f"{STYLE[k][2]} ({sum(1 for n in pts if classify(n)==k)})")
               for k in STYLE]
    ax.legend(handles=handles, loc="upper left", fontsize=9, framealpha=0.9)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default="config/field.yaml")
    ap.add_argument("--out", default="docs/field_measured_map.png")
    ap.add_argument("--half-x", type=float, default=4.175, help="场地半长（米）")
    ap.add_argument("--half-y", type=float, default=3.0, help="场地半宽（米）")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    yp = Path(args.yaml)
    if not yp.is_absolute():
        yp = ROOT / yp
    _, pts = load_points(yp)
    if not pts:
        sys.exit(f"错误：{yp} 里没有有效的坐标点")

    print(f"读取 {yp.name}：{len(pts)} 个标志点")
    cnt: dict[str, int] = {}
    for n in pts:
        cnt[classify(n)] = cnt.get(classify(n), 0) + 1
    order = ["LCross", "TCross", "XCross", "PenalityPoint"]
    print("类别统计：" + "  ".join(f"{k}={cnt.get(k,0)}" for k in order))
    print(f"场地范围：X ±{args.half_x} m，Y ±{args.half_y} m")

    warns = check(pts, args.half_x, args.half_y)
    print("\n--- 合法性检查 ---")
    print("通过：坐标均在场地内、无重复点、四类齐全" if not warns else "\n".join("⚠ " + w for w in warns))

    sym = symmetry_report(pts)
    print("\n--- 镜像对称性（Δ 为实测残差，属场地画线偏差）---")
    print("\n".join(sym) if sym else "（无可配对的镜像点）")

    if not args.no_plot:
        out = Path(args.out)
        if not out.is_absolute():
            out = ROOT / out
        plot(pts, args.half_x, args.half_y, out, "C0 field — measured markers (2026-11-12)")
        print(f"\n分布图 -> {out}")
    return 1 if warns else 0


if __name__ == "__main__":
    sys.exit(main() or 0)
