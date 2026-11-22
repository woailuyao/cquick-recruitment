# -*- coding: utf-8 -*-
"""场地实测图纸：把所有要量的尺寸、以及 14 个标志点的位置，用字母标在一张图上。

为什么需要它：
    单目相机的尺度不可观测 —— 地图假设多大，解出的相机高度与地面坐标就按那个
    尺度走。官方场地 14×9 m，C0 现场若不等于这个尺寸，就必须先量出现场真实的
    长、宽、门宽（以及中圈/罚球区的几个特征尺寸），把尺度钉死，再解外参。

    量的时候需要一张「对着就能念字母」的图，这个脚本负责生成它：
      · 场地按官方 field_geometry.yaml 的真实比例画出（14×9，球门 2.6，中圈 r=1.5…）
      · 尺寸项用蓝色字母（L/W/G/C/WA/DA/GB/DB）
      · 14 个标志点用红色编号（T1–T6 为 T 形、L1–L5 为 L 形、X1–X2 为 X 形、P0 为点球点）
      · 右半场加浅色底纹，标注「官方 field.yaml 覆盖范围」——官方只给了 x∈[0,7] 半场

输出（项目根目录下运行）：
    outputs/field_survey_A4.pdf        两页，可直接打印 A4 横向
    outputs/field_survey_map.png       第 1 页位图（场地图 + 尺寸表）
    outputs/field_survey_points.png    第 2 页位图（点位对照表 + 核对清单）

用法：
    run.bat tools\\field_survey_sheet.py
    run.bat tools\\field_survey_sheet.py --out outputs\\my_sheet
"""

from __future__ import annotations

import argparse
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
from tools.realtime_field import build_field_map     # noqa: E402

# ── 中文字体 ────────────────────────────────────────────────────────────────
_CJK = None
for name in ("msyh.ttc", "msyhbd.ttc", "simhei.ttf", "simsun.ttc"):
    fp = Path("C:/Windows/Fonts") / name
    if fp.exists():
        try:
            font_manager.fontManager.addfont(str(fp))
            _CJK = font_manager.FontProperties(fname=str(fp)).get_name()
            break
        except Exception:
            continue
if _CJK:
    plt.rcParams["font.family"] = [_CJK, "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# ── 颜色 ────────────────────────────────────────────────────────────────────
LINE = "#2f2f2f"       # 场地白线（画成深色，打印清楚）
GRASS = "#eaf5ea"
DIM = "#1f5fb0"        # 尺寸标注（蓝）
MARK = "#c0392b"       # 标志点（红）
HALF = "#fff6e0"       # 官方覆盖半场底纹

# ── 标志点显示编号（官方名字 → 图上字母）────────────────────────────────────
LETTER = {
    "TCross1": "T1", "TCross2": "T2", "TCross3": "T3",
    "TCross4": "T4", "TCross5": "T5", "TCross6": "T6",
    "LCross1": "L1", "LCross2": "L2", "LCross3": "L3",
    "LCross4": "L4", "LCross5": "L5",
    "XCross1": "X1", "XCross2": "X2",
    "PenalityPoint": "P0",
}
MEANING = {
    "T1": "球门线 × 罚球区左线", "T2": "球门线 × 球门区左线",
    "T3": "球门线 × 球门区右线", "T4": "球门线 × 罚球区右线",
    "T5": "中线 × 左侧边线", "T6": "中线 × 右侧边线",
    "L1": "球门线左角点", "L2": "球门线右角点",
    "L3": "球门区前沿左角", "L4": "球门区前沿右角",
    "L5": "罚球区前沿左角",
    "X1": "中线 × 中圈（下）", "X2": "中线 × 中圈（上）",
    "P0": "点球点（官方反推位）",
}
# 标签偏移（m），逐个手调，避免压线
OFFSET = {
    "T1": (-0.70, -0.10), "T2": (-0.70, -0.24), "T3": (-0.70, 0.10),
    "T4": (-0.72, 0.16), "T5": (0.48, -0.40), "T6": (0.22, 0.34),
    "L1": (-0.64, 0.24), "L2": (-0.62, -0.34), "L3": (0.30, -0.34),
    "L4": (0.32, 0.26), "L5": (-0.12, -0.48), "X1": (0.32, -0.18),
    "X2": (0.32, 0.06), "P0": (0.18, 0.32),
}

# ── 尺寸项（代号 / 名称 / 官方值 / 图上怎么量）──────────────────────────────
DIM_ROWS = [
    ("L",  "场地长度（端线↔端线）",          14.00, "沿长边，左右两个球门线之间"),
    ("W",  "场地宽度（边线↔边线）",           9.00, "沿短边，两条边线之间"),
    ("G",  "球门内宽（两门柱内侧）",          2.60, "量球门，别量球门区"),
    ("C",  "中圈直径 = X1↔X2",               3.00, "量 X1 到 X2 两个交点"),
    ("WA", "罚球区宽（球门端）",              6.00, "罚球区两条竖线之间"),
    ("DA", "罚球区深（球门线→前沿）",          3.00, "球门线到前沿横线"),
    ("GB", "球门区宽",                       4.00, "球门区两条竖线之间"),
    ("DB", "球门区深",                       1.00, "球门线到球门区前沿"),
]


def draw_field(ax, geom, markers):
    """按官方几何画场地与标志点。"""
    half_l = float(geom["field_length_m"]) / 2.0        # 7.0
    half_w = float(geom["field_width_m"]) / 2.0         # 4.5
    pa_d = float(geom["penalty_area_length_m"])         # 3.0
    pa_w = float(geom["penalty_area_width_m"]) / 2.0    # 3.0
    ga_d = float(geom["goal_area_length_m"])            # 1.0
    ga_w = float(geom["goal_area_width_m"]) / 2.0       # 2.0
    circ = float(geom["circle_radius_m"])               # 1.5
    goal_w = float(geom["goal_width_m"]) / 2.0          # 1.3

    # 官方覆盖半场底纹
    ax.add_patch(Rectangle((0, -half_w), half_l, 2 * half_w,
                           facecolor=HALF, edgecolor="none", zorder=0))
    # 草地
    ax.add_patch(Rectangle((-half_l, -half_w), 2 * half_l, 2 * half_w,
                           facecolor=GRASS, edgecolor="none", zorder=1))
    lw = 1.6
    # 边线 / 端线
    ax.add_patch(Rectangle((-half_l, -half_w), 2 * half_l, 2 * half_w,
                           facecolor="none", edgecolor=LINE, lw=lw, zorder=2))
    # 中线
    ax.plot([0, 0], [-half_w, half_w], color=LINE, lw=lw, zorder=2)
    # 中圈
    ax.add_patch(Arc((0, 0), 2 * circ, 2 * circ, color=LINE, lw=lw, zorder=2))
    # 中点（现场无可见标记）
    ax.plot(0, 0, marker="o", ms=5, mfc="none", mec=LINE, mew=1.2, zorder=3)
    for s in (+1, -1):
        x0, x1 = half_l * s, (half_l - pa_d) * s
        ax.add_patch(Rectangle((min(x0, x1), -pa_w), pa_d, 2 * pa_w,
                               facecolor="none", edgecolor=LINE, lw=lw, zorder=2))
        gx0, gx1 = half_l * s, (half_l - ga_d) * s
        ax.add_patch(Rectangle((min(gx0, gx1), -ga_w), ga_d, 2 * ga_w,
                               facecolor="none", edgecolor=LINE, lw=lw, zorder=2))
        # 球门（画在端线外）
        gx = half_l * s
        ax.add_patch(Rectangle((min(gx, gx + 0.6 * s), -goal_w), 0.6,
                              2 * goal_w, facecolor="none", edgecolor=LINE,
                              lw=lw, zorder=2))

    # 坐标轴说明（放在图外，避免压线）
    ax.annotate("", xy=(7.95, -5.85), xytext=(-7.95, -5.85),
                arrowprops=dict(arrowstyle="-|>", color="#999", lw=1.0))
    ax.text(7.95, -6.05, "+X（朝球门端）", ha="right", va="top", fontsize=8, color="#777")

    # ── 尺寸标注 ──
    def dim(p0, p1, label, text_xy, ha="center", va="center", rot=0):
        ax.annotate("", xy=p1, xytext=p0,
                    arrowprops=dict(arrowstyle="<|-|>", color=DIM, lw=1.2,
                                    shrinkA=0, shrinkB=0))
        ax.text(*text_xy, label, ha=ha, va=va, rotation=rot, fontsize=10,
                color=DIM, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.18", fc="white", ec=DIM, lw=0.8))

    dim((-half_l, -5.42), (half_l, -5.42), "L", (0, -5.42))
    dim((-7.75, -half_w), (-7.75, half_w), "W", (-8.15, 0), rot=90)
    dim((7.82, -goal_w), (7.82, goal_w), "G", (7.82, 0), rot=90)
    dim((0.34, -circ), (0.34, circ), "C", (0.72, 0.80), ha="left")
    dim((3.72, -pa_w), (3.72, pa_w), "WA", (3.50, 0), ha="right", rot=90)
    dim((half_l - pa_d, 2.58), (half_l, 2.58), "DA", (5.5, 2.58))
    dim((-6.52, -ga_w), (-6.52, ga_w), "GB", (-6.52, 0))
    dim((-half_l, -3.42), (-(half_l - ga_d), -3.42), "DB", (-6.5, -3.74))

    # ── 标志点 ──
    for m in markers:
        name = m.name or ""
        code = LETTER.get(name)
        if code is None:
            continue
        dx, dy = OFFSET.get(code, (0.2, 0.2))
        ax.plot(m.x, m.y, marker="o", ms=7.5, mfc=MARK, mec="white",
                mew=1.1, zorder=6)
        ax.text(m.x + dx, m.y + dy, code, fontsize=10.5, color=MARK,
                fontweight="bold", ha="center", va="center", zorder=7,
                bbox=dict(boxstyle="round,pad=0.16", fc="white", ec=MARK, lw=0.7))

    ax.set_xlim(-9.0, 8.7)
    ax.set_ylim(-6.3, 5.3)
    ax.set_aspect("equal")
    ax.axis("off")


def page1(map_png, pdf_pages):
    fig = plt.figure(figsize=(11.69, 8.27))          # A4 横向
    gs = fig.add_gridspec(1, 2, width_ratios=[1.62, 1.0],
                          left=0.03, right=0.98, top=0.855, bottom=0.05, wspace=0.05)
    ax = fig.add_subplot(gs[0, 0])
    axt = fig.add_subplot(gs[0, 1])
    axt.axis("off")

    fm = build_field_map(ROOT / "config" / "field.yaml",
                         ROOT / "config" / "field_geometry.yaml",
                         mirror=False, site_path=None)
    draw_field(ax, fm.geometry, fm.markers)

    fig.text(0.03, 0.958, "C0 场地实测图纸 · 尺寸", fontsize=15, fontweight="bold",
             ha="left", va="top")
    fig.text(0.03, 0.918,
             "按官方 field_geometry.yaml 的真实比例画出（14×9 m）；右表「官方」列是地图给定值，"
             "请把卷尺量到的数值填在「实测」列。",
             fontsize=8.8, color="#555", ha="left", va="top")
    fig.text(0.03, 0.888,
             "图例：蓝色字母 = 要量的尺寸　｜　红点 = 标志点（T1–T6 为 T 形、L1–L5 为 L 形、"
             "X1–X2 为 X 形、P0 为点球点）　｜　橙色底 = 官方覆盖的 x∈[0,7] 半场",
             fontsize=8.8, color="#555", ha="left", va="top")
    fig.text(0.03, 0.858,
             "坐标约定：原点 = 场地中心（中线中点）　+ X 指向球门端　+ Y 面朝 +X 时的左侧　"
             "单位 m，地面 z = 0",
             fontsize=8.8, color="#777", ha="left", va="top")

    rows = [["代号", "是什么", "官方", "实测"]]
    for code, desc, val, _how in DIM_ROWS:
        rows.append([code, desc, f"{val:.2f}", ""])
    rows.append(["h", "相机镜头到地面高度", "—", ""])

    tbl = axt.table(cellText=rows[1:], colLabels=rows[0],
                    colWidths=[0.13, 0.52, 0.14, 0.21],
                    loc="upper center", cellLoc="left",
                    bbox=[0.0, 0.34, 1.0, 0.62])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9.0)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#bbb")
        cell.set_linewidth(0.6)
        if r == 0:
            cell.set_facecolor("#eef2f7")
            cell.set_text_props(fontweight="bold")
        if c == 3 and r > 0:
            cell.set_facecolor("#fffdf5")
    tbl.scale(1, 1.45)

    axt.text(0.0, 0.995, "① 尺寸表（填右列）", fontsize=11, fontweight="bold",
             ha="left", va="top", transform=axt.transAxes)
    tip = ("测量约定\n"
           "· 白线中心 → 白线中心，不量线宽\n"
           "· 卷尺贴地拉直，误差 1 cm 足够\n"
           "· 从场地中心往两边量最省事\n"
           "· 中圈量 X1↔X2（中圈与中线两交点）\n\n"
           "量完把字母和数值发我，例如：\n"
           "  L 10.02  W 6.48  G 2.05  C 2.15\n"
           "我就按实测尺度重建地图再解外参。")
    axt.text(0.0, 0.27, tip, fontsize=9, ha="left", va="top", transform=axt.transAxes,
             linespacing=1.55,
             bbox=dict(boxstyle="round,pad=0.5", fc="#f7f7f7", ec="#ccc", lw=0.8))

    fig.savefig(map_png, dpi=200)
    pdf_pages.savefig(fig)
    plt.close(fig)


def page2(points_png, pdf_pages):
    fig = plt.figure(figsize=(11.69, 8.27))
    ax = fig.add_axes([0.05, 0.06, 0.62, 0.80])
    ax.axis("off")
    ax2 = fig.add_axes([0.69, 0.06, 0.29, 0.80])
    ax2.axis("off")

    fm = build_field_map(ROOT / "config" / "field.yaml",
                         ROOT / "config" / "field_geometry.yaml",
                         mirror=False, site_path=None)
    rows = [["代号", "位置（场地中心系）", "官方 x", "官方 y", "实测 x", "实测 y"]]
    for m in fm.markers:
        code = LETTER.get(m.name or "", m.name)
        rows.append([code, MEANING.get(code, ""), f"{m.x:+.3f}", f"{m.y:+.3f}", "", ""])

    tbl = ax.table(cellText=rows[1:], colLabels=rows[0],
                   colWidths=[0.10, 0.40, 0.12, 0.12, 0.13, 0.13],
                   cellLoc="left", bbox=[0.0, 0.02, 1.0, 0.86])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.6)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#bbb")
        cell.set_linewidth(0.6)
        if r == 0:
            cell.set_facecolor("#eef2f7")
            cell.set_text_props(fontweight="bold")
        elif c >= 4:
            cell.set_facecolor("#fffdf5")
    tbl.scale(1, 1.42)

    fig.text(0.05, 0.955, "C0 场地实测图纸 · 点位（可选，第二轮）", fontsize=15,
             fontweight="bold", ha="left", va="top")
    fig.text(0.05, 0.918,
             "只有在「尺寸表已经交了、但长宽比对不上」时才需要这一页：逐点量到场地中心的 x、y。",
             fontsize=9, color="#555", ha="left", va="top")

    check = (
        "现场核对（打勾）\n"
        "☐ 有边线、端线、中线\n"
        "☐ 有中圈，且中线上能看到两个交点\n"
        "☐ 有罚球区（球门前的长方形）\n"
        "☐ 有球门区（更小的那个长方形）\n"
        "☐ 有球门（两门柱）\n"
        "☐ 中圈圆心处有点球点标记吗？\n"
        "   有 → 量它到球门线的距离\n"
        "   无 → 记「无」，与 config/field_site.yaml 一致\n"
        "☐ 场上有 T 形、L 形、X 形标志吗？\n"
        "   分别在哪几个字母的位置上\n\n"
        "坐标系约定\n"
        "· 原点 = 场地中心（中线中点）\n"
        "· +X 指向有球门的那一端\n"
        "· +Y 面朝 +X 时的左侧\n"
        "· 单位米，z 向上，地面 z=0"
    )
    ax2.text(0.0, 1.0, check, fontsize=9.5, ha="left", va="top",
             transform=ax2.transAxes, linespacing=1.7,
             bbox=dict(boxstyle="round,pad=0.55", fc="#f7f7f7", ec="#ccc", lw=0.8))

    fig.savefig(points_png, dpi=200)
    pdf_pages.savefig(fig)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="生成 C0 场地实测图纸")
    ap.add_argument("--out", default=str(OUTPUT_DIR / "field_survey"),
                    help="输出前缀（不带扩展名）")
    args = ap.parse_args()

    prefix = Path(args.out)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    map_png = prefix.with_name(prefix.name + "_map.png")
    points_png = prefix.with_name(prefix.name + "_points.png")
    pdf = prefix.with_name(prefix.name + "_A4.pdf")

    from matplotlib.backends.backend_pdf import PdfPages
    with PdfPages(str(pdf)) as pdf_pages:
        page1(map_png, pdf_pages)
        page2(points_png, pdf_pages)

    for p in (pdf, map_png, points_png):
        print(f"输出: {p.relative_to(ROOT)}  ({p.stat().st_size:,} B)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
