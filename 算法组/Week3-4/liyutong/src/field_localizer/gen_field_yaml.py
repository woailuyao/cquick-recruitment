# -*- coding: utf-8 -*-
"""
按《算法考核细则》的坐标系规范生成任务 2 需要的场地地图：

    field.yaml                  标志点地图（地图匹配用）
    config/field_geometry.yaml  场地几何参数
    _probe/field_topview.png    俯视图（与细则示意图同朝向，肉眼看对不对）

细则规范（见「场地坐标系示意图」）：
    原点 O      = 场地中心
    +X          = 指向唯一球门
    +Y          = 面向 +X 时的左侧
    +Z          = 垂直地面向上，地面 Z = 0
    长度单位    = 米

用法：python gen_field_yaml.py
"""
import os
from collections import Counter

# ============================================================
# 实测值（8 x 6 m 迷你场地）——单位和含义都写清楚，改这里后重跑
# ============================================================
F = dict(
    length_m=8.0,          # 端线到端线（内沿），沿 X 轴
    width_m=6.0,           # 边线到边线（内沿），沿 Y 轴
    line_width_m=0.05,     # 线宽 5 cm
    center_circle_r_m=0.75,
    penalty_depth_m=1.0,   # 罚球区从端线向内
    penalty_width_m=3.8,   # 罚球区宽度（以中线对称）
    goal_width_m=2.8,      # 球门内宽（两根门柱内侧）
    goal_height_m=2.0,     # 球门内高（横梁下沿到地面）
)

PENALTY_BOTH_ENDS = True   # 远端（-X，无球门那侧）是否也有罚球区框，看俯视图确认
INCLUDE_CIRCLE_X = True    # 中圈与中线交点是否算 X 类十字（几何上是，看模型标注口径）
CENTER_DOT_EXISTS = False  # 实测：场地中心（点球点）地面上没有画任何东西，只有中圈。
                           # 所以 P 类无法检测、无法标注，地图里只作为"几何基准点"保留，
                           # 不写进 markers（否则匹配时永远等不到 P 观测，白占一个类别）

# 派生量
HX = F["length_m"] / 2.0          # 端线 x = ±HX
HY = F["width_m"] / 2.0           # 边线 y = ±HY
HPY = F["penalty_width_m"] / 2.0  # 罚球区半宽
PX_GOAL = HX - F["penalty_depth_m"]    # +X 端罚球区前沿 x
PX_FAR = -HX + F["penalty_depth_m"]    # -X 端罚球区前沿 x


def build_markers():
    """返回标志点列表。坐标单位米，原点=场地中心，+X 朝球门，+Y 朝左。"""
    ms = []

    def add(t, x, y, note):
        ms.append(dict(id=len(ms) + 1, type=t, x=round(x, 3), y=round(y, 3), note=note))

    # ---- L：两线相交且都终止 ----
    add("L", +HX, +HY, "球门端/左边线 角")
    add("L", +HX, -HY, "球门端/右边线 角")
    add("L", -HX, +HY, "远端/左边线 角")
    add("L", -HX, -HY, "远端/右边线 角")
    add("L", PX_GOAL, +HPY, "球门端罚球区 左前角")
    add("L", PX_GOAL, -HPY, "球门端罚球区 右前角")
    if PENALTY_BOTH_ENDS:
        add("L", PX_FAR, +HPY, "远端罚球区 左前角")
        add("L", PX_FAR, -HPY, "远端罚球区 右前角")

    # ---- T：一条线终止在另一条线上 ----
    add("T", 0.0, +HY, "中线/左边线")
    add("T", 0.0, -HY, "中线/右边线")
    add("T", +HX, +HPY, "球门端罚球区边线/端线")
    add("T", +HX, -HPY, "球门端罚球区边线/端线")
    if PENALTY_BOTH_ENDS:
        add("T", -HX, +HPY, "远端罚球区边线/端线")
        add("T", -HX, -HPY, "远端罚球区边线/端线")

    # ---- X：两线交叉 ----
    if INCLUDE_CIRCLE_X:
        r = F["center_circle_r_m"]
        add("X", 0.0, +r, "中圈/中线 交点上")
        add("X", 0.0, -r, "中圈/中线 交点下")

    return ms


def build_anchors():
    """不可检测、但几何上有用的基准点"""
    if CENTER_DOT_EXISTS:
        return []
    return [dict(id=1, type="P", x=0.0, y=0.0,
                 note="场地中心 = 细则规定的点球点；实测地面无实体标记，检测器不输出")]


def write_geom():
    os.makedirs("config", exist_ok=True)
    p = os.path.join("config", "field_geometry.yaml")
    with open(p, "w", encoding="utf-8") as f:
        f.write("# 场地几何参数（卷尺实测）\n")
        f.write("# 坐标系：原点 = 场地中心；+X 指向球门；+Y 面向 +X 时的左侧；+Z 向上\n")
        f.write("#         地面 Z = 0，单位米，右手系（X × Y = Z）\n\n")
        f.write("coordinate_frame:\n")
        f.write("  origin: field_center\n")
        f.write("  x_axis: toward_goal\n")
        f.write("  y_axis: left_of_plus_x\n")
        f.write("  z_axis: up\n")
        f.write("  handedness: right\n")
        f.write("  unit: m\n\n")
        f.write("field:\n")
        for k in ("length_m", "width_m", "line_width_m",
                  "center_circle_r_m", "penalty_depth_m", "penalty_width_m"):
            f.write(f"  {k}: {F[k]}\n")
        f.write("\n# 端线 / 边线坐标（方便代码里直接引用）\n")
        f.write("bounds:\n")
        f.write(f"  endline_x: [{round(-HX, 3)}, {round(+HX, 3)}]\n")
        f.write(f"  sideline_y: [{round(-HY, 3)}, {round(+HY, 3)}]\n")
        f.write("  penalty_front_x: "
                f"[{round(PX_FAR, 3)}, {round(PX_GOAL, 3)}]\n")
        f.write("  penalty_side_y: "
                f"[{round(-HPY, 3)}, {round(+HPY, 3)}]\n")
        f.write("\n# 唯一球门在 +X 端（细则：+X 指向唯一球门）\n")
        f.write("goal:\n")
        f.write(f"  center_x: {round(+HX, 3)}\n")
        f.write("  center_y: 0.0\n")
        f.write("  width_m: %s    # 待量\n" % ("null" if F["goal_width_m"] is None else F["goal_width_m"]))
        f.write("  height_m: %s   # 待量\n" % ("null" if F["goal_height_m"] is None else F["goal_height_m"]))
    return p


def write_field(markers, anchors=()):
    p = "field.yaml"
    with open(p, "w", encoding="utf-8") as f:
        f.write("# ============================================================\n")
        f.write("# 场地标志点地图 —— 任务 2「地图匹配」\n")
        f.write("# 坐标系严格按《算法考核细则》场地坐标系示意图：\n")
        f.write("#   原点 O = 场地中心\n")
        f.write("#   +X     = 指向唯一球门\n")
        f.write("#   +Y     = 面向 +X 时的左侧\n")
        f.write("#   +Z     = 垂直地面向上（地面 Z = 0）\n")
        f.write("#   单位   = 米，右手系\n")
        f.write("# ============================================================\n")
        f.write("# 类别约定：\n")
        f.write("#   L = 线角（两线相交且都到此终止，如场地角、罚球区前角）\n")
        f.write("#   T = 丁字口（一条线终止在另一条延续线上，如中线与边线交点）\n")
        f.write("#   X = 十字交叉（两线交叉，本场地只有中圈与中线的交点）\n")
        f.write("#   P = 点球点（细则规定在场地中心，即原点 O）\n")
        f.write(f"# 共 {len(markers)} 个标志点\n\n")

        f.write("frame:\n")
        f.write("  origin: field_center\n")
        f.write("  x_axis: toward_goal\n")
        f.write("  y_axis: left_of_plus_x\n")
        f.write("  z_axis: up\n")
        f.write("  handedness: right\n")
        f.write("  unit: m\n\n")

        f.write("field:\n")
        for k in ("length_m", "width_m", "line_width_m",
                  "center_circle_r_m", "penalty_depth_m", "penalty_width_m"):
            f.write(f"  {k}: {F[k]}\n")
        f.write("\n")

        f.write("# 唯一球门位于 +X 端；场地标线两端近似对称，\n")
        f.write("# 因此仅靠标线无法唯一确定 yaw，需靠球门消歧（0 或 180 deg）\n")
        f.write("goal:\n")
        f.write(f"  x: {round(+HX, 3)}\n")
        f.write("  y: 0.0\n")
        f.write("  width_m: %s\n" % ("null" if F["goal_width_m"] is None else F["goal_width_m"]))
        f.write("  height_m: %s\n\n" % ("null" if F["goal_height_m"] is None else F["goal_height_m"]))

        f.write("markers:\n")
        for mk in markers:
            body = ("  - {id: %d, type: %s, x: %.3f, y: %.3f}"
                    % (mk["id"], mk["type"], mk["x"], mk["y"]))
            pad = " " * max(1, 46 - len(body))
            f.write(body + pad + "# " + mk["note"] + "\n")

        f.write("\n# --- 待人工确认 ---\n")
        f.write("# 1) length_m=8.0 vs 实测「中线到端线 2.0 m」互相矛盾：\n")
        f.write("#    原点在场地中心时，中线到端线必然 = length/2 = 4.0 m。\n")
        f.write("#    请再拉一次卷尺：中间那条横线 → 端线，应读到 4.0 m。\n")
        f.write("#    若确实只有 2.0 m，则 length_m 应为 4.0，本文件全部 x 值要重算。\n")
        f.write("# 2) 点球点按细则在场地中心，但实测地面无实体标记 → 归入 anchors，不参与检测。\n")
        f.write("# 3) goal.width_m / height_m 待量。\n")
    return p


def draw(markers):
    try:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import font_manager, rcParams
        for cand in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
            if os.path.exists(cand):
                font_manager.fontManager.addfont(cand)
                rcParams["font.sans-serif"] = [font_manager.FontProperties(fname=cand).get_name()]
                break
        rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Circle
    except Exception as e:
        print("画图跳过（不影响 yaml）：", e)
        return

    fig, ax = plt.subplots(figsize=(8, 10))
    # 细则示意图朝向：+X 向上（球门在上），+Y 向左（横轴反向）
    ax.add_patch(Rectangle((-HY, -HX), F["width_m"], F["length_m"],
                           fill=True, fc="#2e7d32", ec="white", lw=2, alpha=0.25))
    ax.plot([-HY, HY], [0, 0], "w-", lw=3)                       # 中线
    ax.add_patch(Circle((0, 0), F["center_circle_r_m"], fill=False, ec="w", lw=2))
    for xf, xe in ((PX_GOAL, +HX), (PX_FAR, -HX)):
        if xf == PX_FAR and not PENALTY_BOTH_ENDS:
            continue
        ax.plot([-HPY, -HPY, HPY, HPY], [xe, xf, xf, xe], "w-", lw=2)
    # 球门（画在场地外，用深色避免白底上看不见）
    gw = F["goal_width_m"] or 1.5
    gx = +HX + 0.45
    ax.plot([-gw / 2, -gw / 2, gw / 2, gw / 2],
            [+HX, gx, gx, +HX], "-", color="#37474f", lw=3, zorder=6)
    ax.plot([-gw / 2, gw / 2], [gx, gx], "-", color="#37474f", lw=1.2,
            ls=":", zorder=6)
    ax.annotate(f"唯一球门（+X 端，内宽 {gw} m）", (0, gx + 0.28),
                ha="center", fontsize=10, color="#37474f", zorder=7)

    style = {"L": ("o", "#4fc3f7"), "T": ("s", "#a5d6a7"),
             "X": ("^", "#ef5350"), "P": ("*", "#ffb74d")}
    for mk in markers:
        m_s, c = style[mk["type"]]
        ax.plot(mk["y"], mk["x"], m_s, color=c, ms=10, mec="k", mew=0.6, zorder=5)
        ax.annotate(str(mk["id"]), (mk["y"], mk["x"]), textcoords="offset points",
                    xytext=(7, -3), fontsize=8, color="k",
                    bbox=dict(fc="w", ec="none", alpha=0.6, pad=0.6))
    for t, (m_s, c) in style.items():
        ax.plot([], [], m_s, color=c, ms=9, mec="k", label=t)
    ax.legend(loc="lower right", fontsize=9, title="标志类型")

    # 坐标轴示意
    ax.annotate("", xy=(0, +HX + 0.3), xytext=(0, +HX - 1.2),
                arrowprops=dict(arrowstyle="-|>", color="#d32f2f", lw=2.5))
    ax.annotate("+X 指向球门", (0.08, +HX - 1.0), color="#d32f2f", fontsize=11)
    ax.annotate("", xy=(+HY + 1.0, 0), xytext=(+HY - 0.6, 0),
                arrowprops=dict(arrowstyle="-|>", color="#1976d2", lw=2.5))
    ax.annotate("+Y 左侧", (+HY + 0.3, 0.18), color="#1976d2", fontsize=11)

    ax.set_xlim(+HY + 1.3, -HY - 1.3)   # 反向：+Y 朝左
    ax.set_ylim(-HX - 0.9, +HX + 1.1)
    ax.set_aspect("equal")
    ax.set_xlabel("y (m)  ←  +Y 朝左")
    ax.set_ylabel("x (m)  ↑  +X 朝球门")
    ax.set_title(f"场地俯视图 {F['length_m']}x{F['width_m']} m，{len(markers)} 个标志点\n"
                 f"原点=场地中心，+X→球门，+Y→面向+X的左侧", fontsize=11)
    ax.grid(alpha=0.25)
    os.makedirs("_probe", exist_ok=True)
    fig.tight_layout()
    fig.savefig("_probe/field_topview.png", dpi=130)
    print("俯视图：_probe/field_topview.png")


def main():
    markers = build_markers()
    print("坐标系：原点=场地中心，+X→球门，+Y→面向+X的左侧，+Z 向上（右手系）")
    print(f"端线 x = ±{HX:.2f} m，边线 y = ±{HY:.2f} m")
    print(f"罚球区前沿 x = {PX_FAR:.2f} / {PX_GOAL:.2f}，侧边 y = ±{HPY:.2f}")
    print(write_geom())
    anchors = build_anchors()
    print(write_field(markers, anchors))
    print(f"共 {len(markers)} 个可检测标志点，类型分布 {dict(Counter(m['type'] for m in markers))}")
    for a in anchors:
        print(f"  [anchor] {a['type']} ({a['x']:+.2f}, {a['y']:+.2f}) {a['note']}")
    for m in markers:
        print(f"  {m['id']:>2} {m['type']}  ({m['x']:+.2f}, {m['y']:+.2f})  {m['note']}")
    draw(markers)


if __name__ == "__main__":
    main()
