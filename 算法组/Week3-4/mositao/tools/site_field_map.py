# -*- coding: utf-8 -*-
"""按「现场实测尺寸」参数化重建 C0 场地地图。

为什么这样做：
    官方 field.yaml 是 14×9 m 的成年组场地，现场只有 7.6×6.03 m，且各条线的比例
    并不一致（长度方向缩到 0.54、宽度方向 0.67、球门区深 0.94），所以不是一个缩放
    能对齐的。但场地标志点从来都是「白线交点」，只要知道各条线在哪，交点就能算出来，
    不必逐点拿卷尺去量。

    于是这里用 8 个实测尺寸把 14 个（含对称补全共 16 个）标志点的坐标算出来：
        L  场地长      W  场地宽        G  球门内宽      C  中圈直径（X1↔X2）
        GB 球门区宽    DB 球门区深      AW 罚球区宽      DA 罚球区深

输出：
    outputs/field_site_map_preview.png   现场地图预览（用来和实地对照）
    config/field_site.yaml 的 site_geometry 段（--write-site 时才写，先看预览再写）

用法（项目根目录）：
    run.bat tools\\site_field_map.py --L 7.6 --W 6.03 --G 2.95 --GB 2.95 ^
        --DB 0.94 --AW 3.95 --DA 1.95 --C 1.5
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib import font_manager                  # noqa: E402
from matplotlib.patches import Arc, Rectangle        # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR              # noqa: E402

_CJK = None
for _n in ("msyh.ttc", "simhei.ttf", "simsun.ttc"):
    _fp = Path("C:/Windows/Fonts") / _n
    if _fp.exists():
        try:
            font_manager.fontManager.addfont(str(_fp))
            _CJK = font_manager.FontProperties(fname=str(_fp)).get_name()
            break
        except Exception:
            continue
if _CJK:
    plt.rcParams["font.family"] = [_CJK, "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

LINE, GRASS, MARK = "#2f2f2f", "#eaf5ea", "#c0392b"


def build_site_markers(g: dict) -> list[dict]:
    """按实测尺寸算标志点。g 的单位是米，全部是「全长/全宽」，不是半宽。

    每个点都来自两条白线的交点，改尺寸后坐标自动跟着变：
        T1/T4  球门线 × 罚球区两条竖线      T2/T3  球门线 × 球门区两条竖线
        T5/T6  中线   × 两条边线            L1/L2  球门线 × 边线（角点）
        L3/L4  球门区前沿 × 球门区竖线      L5/L6  罚球区前沿 × 罚球区竖线
        X1/X2  中线   × 中圈
    """
    hl, hw = g["L"] / 2.0, g["W"] / 2.0
    pen_w, pen_d = g["AW"] / 2.0, g["DA"]
    goal_w, goal_d = g["GB"] / 2.0, g["DB"]
    circ = g["C"] / 2.0
    pts = [
        ("T1", "TCross", hl, -pen_w, "球门线 × 罚球区左线"),
        ("T2", "TCross", hl, -goal_w, "球门线 × 球门区左线"),
        ("T3", "TCross", hl, +goal_w, "球门线 × 球门区右线"),
        ("T4", "TCross", hl, +pen_w, "球门线 × 罚球区右线"),
        ("T5", "TCross", 0.0, -hw, "中线 × 左边线"),
        ("T6", "TCross", 0.0, +hw, "中线 × 右边线"),
        ("L1", "LCross", hl, -hw, "球门线左角点"),
        ("L2", "LCross", hl, +hw, "球门线右角点"),
        ("L3", "LCross", hl - goal_d, -goal_w, "球门区前沿左角"),
        ("L4", "LCross", hl - goal_d, +goal_w, "球门区前沿右角"),
        ("L5", "LCross", hl - pen_d, -pen_w, "罚球区前沿左角"),
        ("L6", "LCross", hl - pen_d, +pen_w, "罚球区前沿右角（官方缺，对称补）"),
        ("X1", "XCross", 0.0, -circ, "中线 × 中圈（下）"),
        ("X2", "XCross", 0.0, +circ, "中线 × 中圈（上）"),
    ]
    if g.get("P") is not None:
        pts.append(("P0", "PenaltyPoint", hl - float(g["P"]), 0.0, "点球点"))
    return [dict(code=c, type=t, x=x, y=y, note=n) for c, t, x, y, n in pts]


def label_xy(code: str, x: float, y: float, hl: float) -> tuple[float, float]:
    """标签放在哪儿：端线上的点统一甩到场外一列，其余贴点旁。

    为什么不用固定偏移：球门线 × 球门区线（T2/T3）与球门区前沿角（L3/L4）的 y 完全相同，
    固定偏移会让两块标签叠在一起。
    """
    if code in ("T1", "T2", "T3", "T4", "L1", "L2"):
        return hl + 0.60, y
    if code in ("L3", "L4", "L5", "L6"):
        return x - 0.36, y
    if code in ("T5", "T6", "X1", "X2"):
        return 0.36, y
    if code == "P0":
        return x, y + 0.32
    return x + 0.3, y + 0.25


def draw(g: dict, markers: list[dict], title: str) -> plt.Figure:
    hl, hw = g["L"] / 2.0, g["W"] / 2.0
    pen_w, pen_d = g["AW"] / 2.0, g["DA"]
    goal_w, goal_d = g["GB"] / 2.0, g["DB"]
    circ = g["C"] / 2.0
    goal_in = g["G"] / 2.0

    fig = plt.figure(figsize=(11.6, 8.0))
    ax = fig.add_axes([0.03, 0.05, 0.58, 0.82])
    axt = fig.add_axes([0.635, 0.05, 0.355, 0.82])
    axt.axis("off")

    ax.add_patch(Rectangle((-hl, -hw), 2 * hl, 2 * hw, facecolor=GRASS, edgecolor=LINE, lw=1.8))
    ax.plot([0, 0], [-hw, hw], color=LINE, lw=1.8)
    ax.add_patch(Arc((0, 0), 2 * circ, 2 * circ, color=LINE, lw=1.8))
    ax.plot(0, 0, marker="o", ms=5, mfc="none", mec=LINE, mew=1.2)
    for s in (+1, -1):
        x0, x1 = hl * s, (hl - pen_d) * s
        ax.add_patch(Rectangle((min(x0, x1), -pen_w), pen_d, 2 * pen_w,
                               facecolor="none", edgecolor=LINE, lw=1.8))
        gx0, gx1 = hl * s, (hl - goal_d) * s
        ax.add_patch(Rectangle((min(gx0, gx1), -goal_w), goal_d, 2 * goal_w,
                               facecolor="none", edgecolor=LINE, lw=1.8))
        ax.add_patch(Rectangle((min(hl * s, (hl + 0.22) * s), -goal_in), 0.22,
                               2 * goal_in, facecolor="none", edgecolor="#8a6d3b",
                               lw=1.4, ls="--"))

    for m in markers:
        lx, ly = label_xy(m["code"], m["x"], m["y"], hl)
        if abs(lx - (hl + 0.60)) < 1e-6:            # 端线上的点：加引线
            ax.plot([m["x"], lx - 0.26], [m["y"], ly], color=MARK, lw=0.7, ls=":", zorder=5)
        ax.plot(m["x"], m["y"], marker="o", ms=8, mfc=MARK, mec="white", mew=1.1, zorder=6)
        ax.text(lx, ly, m["code"], fontsize=10.5, color=MARK,
                fontweight="bold", ha="center", va="center", zorder=7,
                bbox=dict(boxstyle="round,pad=0.18", fc="white", ec=MARK, lw=0.7))

    ax.annotate("", xy=(hl + 1.2, -hw - 0.55), xytext=(-hl - 0.7, -hw - 0.55),
                arrowprops=dict(arrowstyle="-|>", color="#999", lw=1.0))
    ax.text(hl + 1.2, -hw - 0.78, "+X（朝球门端）", ha="right", va="top", fontsize=8.5, color="#777")
    ax.text(-hl, hw + 0.55, "虚线框 = 球门（现场若有）；红色圆点 = 白线交点",
            ha="left", va="bottom", fontsize=8.2, color="#7a6a4f")
    ax.set_xlim(-hl - 0.9, hl + 1.45)
    ax.set_ylim(-hw - 1.15, hw + 0.95)
    ax.set_aspect("equal")
    ax.axis("off")

    fig.suptitle(title, fontsize=14, fontweight="bold", x=0.03, y=0.968, ha="left")
    fig.text(0.03, 0.918,
             "尺寸：L %.2f  W %.2f  G %.2f  GB %.2f  DB %.2f  AW %.2f  DA %.2f  C %.2f（m）"
             % (g["L"], g["W"], g["G"], g["GB"], g["DB"], g["AW"], g["DA"], g["C"]),
             fontsize=9, color="#555", ha="left", va="top")

    rows = [["代号", "x", "y", "位置"]]
    for m in markers:
        rows.append([m["code"], f"{m['x']:+.2f}", f"{m['y']:+.2f}", m["note"]])
    tbl = axt.table(cellText=rows[1:], colLabels=rows[0],
                    colWidths=[0.11, 0.15, 0.15, 0.59], cellLoc="left",
                    bbox=[0.0, 0.0, 1.0, 1.0])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(6.9)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#ccc")
        cell.set_linewidth(0.5)
        if r == 0:
            cell.set_facecolor("#eef2f7")
            cell.set_text_props(fontweight="bold")
    return fig


def main() -> int:
    ap = argparse.ArgumentParser(description="按实测尺寸重建现场地图")
    for k, dv in (("L", None), ("W", None), ("G", None), ("GB", None),
                  ("DB", None), ("AW", None), ("DA", None), ("C", None)):
        ap.add_argument("--" + k, type=float, default=dv, help=f"实测 {k}（米）")
    ap.add_argument("--P", type=float, default=None, help="点球点到球门线距离（可选）")
    ap.add_argument("--out", default=str(OUTPUT_DIR / "field_site_map_preview.png"))
    args = ap.parse_args()

    g = {k: getattr(args, k) for k in ("L", "W", "G", "GB", "DB", "AW", "DA", "C")}
    missing = [k for k, v in g.items() if v is None]
    if missing:
        print("缺少实测值：" + ", ".join(missing))
        return 2
    g["P"] = args.P

    markers = build_site_markers(g)
    fig = draw(g, markers, "C0 现场地图（按实测尺寸重建）· 待阁下确认拓扑")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=190)
    plt.close(fig)

    print(f"输出: {out.relative_to(ROOT)}  ({out.stat().st_size:,} B)")
    print("标志点坐标（场地中心系，米）：")
    for m in markers:
        print(f"  {m['code']:3s} {m['type']:13s} ({m['x']:+.3f}, {m['y']:+.3f})  {m['note']}")
    (OUTPUT_DIR / "field_site_map.json").write_text(
        json.dumps({"geometry": g, "markers": markers}, ensure_ascii=False, indent=1),
        encoding="utf-8")
    print(f"输出: {(OUTPUT_DIR / 'field_site_map.json').relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
