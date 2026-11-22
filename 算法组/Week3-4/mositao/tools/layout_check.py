# -*- coding: utf-8 -*-
"""机位可行性检查：给定相机高度、俯角、站位与朝向，算出这一帧能看到几个标志点。

为什么需要这个工具
    任务二在 0.73 m 低机位上反复失败，根因不是检测模型，而是**视角几何**：
    内参 fx=1677（水平视场 41.8°、垂直 24.3°）配 12° 俯角时，相机的垂直覆盖面只有
    二十几度。而 0.73 m 高、相机贴在球门后面时，地面可见区间要从 0.5 m 一路铺到 5.9 m，
    需要 48° 的垂直覆盖 —— 物理上塞不进画面。于是整个球场被压进画面顶部一条窄带里：
    6~8 m 的地面只占几十个像素，还会叠在球门网这种同样是白的背景上。

    同一套内参、同一套外参，换成 1.375 m 桌面机位就能跑通（离线自检 5~6 点、残差
    0.09~0.15 m），正是因为那时地面跨度只需 22.6°，刚好落在 24.3° 的视场里。

    所以先算清楚「这个机位理论上能看到什么」，再去调检测器，否则是在跟几何较劲。

用法
    python tools\\layout_check.py
    python tools\\layout_check.py --height 0.73 --pitch 11.99 --x -2.0 --y 1.4 --yaw 0
    python tools\\layout_check.py --sweep      # 扫一遍站位，给出可行的机位表
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import load_yaml                                  # noqa: E402
from src.common.paths import CALIB_DIR                                     # noqa: E402
from src.loc.field_map import FieldMap                                      # noqa: E402
from tools.realtime_field import build_field_map                            # noqa: E402


def load_field() -> FieldMap:
    return build_field_map(ROOT / "config/field.yaml",
                           ROOT / "config/field_geometry.yaml",
                           site_path=ROOT / "config/field_site.yaml")


def intrinsics():
    d = load_yaml(CALIB_DIR / "camera_intrinsics.yaml")["camera_intrinsics"]
    return float(d["fx"]), float(d["fy"]), int(d["width"]), int(d["height"])


def analyse(fm, H, pitch_deg, cx, cy, yaw_deg, fx, fy, W, Hpx):
    """返回每个标志点的可见性与反投影精度。"""
    hfov = 2 * math.degrees(math.atan(W / 2 / fx))
    vfov = 2 * math.degrees(math.atan(Hpx / 2 / fy))
    psi = math.radians(yaw_deg)
    c, s = math.cos(psi), math.sin(psi)

    rows = []
    for m in fm.markers:
        dx, dy = m.x - cx, m.y - cy
        # 载体系：x 沿光轴水平投影，y 向左
        bx = c * dx + s * dy
        by = -s * dx + c * dy
        d_planar = math.hypot(bx, by)
        if bx <= 0.15:
            rows.append((m, "背后", None, None, None))
            continue
        # 垂直方向的俯角（相机看向该点需要的下俯角）
        dep = math.degrees(math.atan2(H, d_planar))
        # 像素坐标：以光轴为中心，pitch 之下为画面下方
        theta_v = dep - pitch_deg                     # 相对光轴的垂直偏角（正=下方）
        u = W / 2 + fx * (by / bx)
        v = Hpx / 2 + fy * math.tan(math.radians(theta_v))
        in_u = 0 <= u < W
        in_v = 0 <= v < Hpx
        # 反投影精度：d(距离)/d(像素)
        theta = math.radians(dep)
        m_per_px = (H / (math.sin(theta) ** 2)) * (math.radians(vfov) / Hpx)
        tag = "可见" if (in_u and in_v) else ("出画面" if bx > 0.15 else "背后")
        rows.append((m, tag, (u, v), m_per_px, dep))
    return rows, hfov, vfov


def report(fm, H, pitch, cx, cy, yaw, fx, fy, W, Hpx, quiet=False):
    rows, hfov, vfov = analyse(fm, H, pitch, cx, cy, yaw, fx, fy, W, Hpx)
    vis = [r for r in rows if r[1] == "可见"]
    behind = sum(1 for r in rows if r[1] == "背后")
    if not quiet:
        print(f"机位 h={H:.2f} m  pitch={pitch:+.1f}°  站位=({cx:+.2f},{cy:+.2f})  yaw={yaw:+.1f}°")
        print(f"视场 水平 {hfov:.1f}°  垂直 {vfov:.1f}°")
    if not vis:
        if not quiet:
            print("  ✗ 画面里没有任何标志点")
        return 0, 999.0, behind
    worst = max(r[3] for r in vis)
    if not quiet:
        print(f"  可见标志点 {len(vis)} 个（{behind} 个在相机背后）")
        print(f"  {'标志':10s} {'类型':10s} {'像素':>16s} {'地面距离':>9s} {'精度':>9s}")
        for m, tag, uv, mpp, dep in sorted(vis, key=lambda r: r[2][1]):
            d = math.hypot(m.x - cx, m.y - cy)
            print(f"  {m.name.replace('@site',''):10s} {m.type:10s} "
                  f"({uv[0]:6.1f},{uv[1]:6.1f}) {d:8.2f} m {mpp*100:7.2f} cm/px")
        bad = [r for r in vis if r[3] > 0.03]
        if bad:
            print(f"  ⚠ 其中 {len(bad)} 个点反投影精度差于 3 cm/px（"
                  f"{', '.join(r[0].name.replace('@site','') for r in bad)}）")
    return len(vis), worst, behind


def main() -> int:
    ap = argparse.ArgumentParser(description="任务二机位可行性检查")
    ap.add_argument("--height", type=float, default=0.73)
    ap.add_argument("--pitch", type=float, default=11.99)
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--sweep", action="store_true", help="扫站位，找能看最多标志点的机位")
    ap.add_argument("--sweep-pitch", action="store_true",
                    help="同时扫站位与俯角，找「可见够多 + 离得够近 + 不压在画面顶部」的机位")
    ap.add_argument("--y-min", type=float, default=280.0,
                    help="要求可见标志点的画面 y 不小于此值（躲开球门网/墙面背景）")
    a = ap.parse_args()

    fm = load_field()
    fx, fy, W, Hpx = intrinsics()
    print(f"地图 {len(fm.markers)} 点｜内参 fx={fx:.1f} fy={fy:.1f} 分辨率 {W}x{Hpx}\n")

    if a.sweep_pitch:
        print(f"扫站位 × 俯角（高度 {a.height:.2f} m 固定，朝向 +X）：")
        print("  判据：可见 ≥5 点、最差点 ≤3.0 cm/px、且全部落在画面 y ≥ "
              f"{a.y_min:.0f}（避开球门网与墙）")
        rows = []
        for pitch in np.arange(4.0, 20.1, 1.0):
            for cx in np.arange(-3.6, 3.7, 0.2):
                for cy in np.arange(-2.8, 2.9, 0.2):
                    rr, _hf, _vf = analyse(fm, a.height, float(pitch), float(cx),
                                           float(cy), a.yaw, fx, fy, W, Hpx)
                    vis = [r for r in rr if r[1] == "可见"]
                    if len(vis) < 5:
                        continue
                    worst = max(r[3] for r in vis)
                    ymin = min(r[2][1] for r in vis)
                    if worst > 0.030 or ymin < a.y_min:
                        continue
                    near = min(math.hypot(m.x - cx, m.y - cy) for m, _t, _u, _p, _d in vis)
                    rows.append((len(vis), worst, ymin, near, float(pitch), float(cx), float(cy)))
        if not rows:
            print("  ✗ 没有组合同时满足三条判据")
            print("  → 说明在当前高度下，低俯角与「标志点离得近」不可兼得")
            return 1

        # 先要可见点多、再要精度好、再要离得近
        rows.sort(key=lambda z: (-z[0], z[1], -z[3]))
        print(f"\n  可行组合 {len(rows)} 个，前 12 名：")
        print(f"  {'站位':>16s} {'俯角':>6s} {'可见':>5s} {'最差点':>9s} "
              f"{'最上沿y':>8s} {'最近点':>8s}")
        seen = set()
        shown = 0
        for n, worst, ymin, near, pitch, cx, cy in rows:
            key = (round(pitch), round(cx, 1), round(cy, 1))
            if key in seen:
                continue
            seen.add(key)
            print(f"  ({cx:+.1f},{cy:+.1f}) {pitch:5.1f}° {n:5d} "
                  f"{worst*100:8.2f}cm {ymin:8.0f} {near:7.2f}m")
            shown += 1
            if shown >= 12:
                break
        print(f"\n  用 python tools\\layout_check.py --pitch <P> --x <X> --y <Y> 看逐点明细。")
        return 0

    if a.sweep:
        print("扫站位（高度固定 %.2f m、俯角 %.2f°，朝向 +X）：" % (a.height, a.pitch))
        print(f"{'站位':>16s} {'可见':>5s} {'最差精度':>10s} {'背后':>5s}")
        cands = []
        for cx in np.arange(-3.4, 3.5, 0.2):
            for cy in np.arange(-2.8, 2.9, 0.2):
                n, worst, beh = report(fm, a.height, a.pitch, float(cx), float(cy),
                                       a.yaw, fx, fy, W, Hpx, quiet=True)
                if n >= 4:
                    cands.append((n, worst, cx, cy))
        if not cands:
            print("  ✗ 扫遍场地也没有能同时看到 4 个以上标志点的站位")
            print("  → 说明问题不在站位，而在高度或俯角")
            return 1
        # 先按「可见最多」再按「精度最好」排序
        cands.sort(key=lambda z: (-z[0], z[1]))
        for n, worst, cx, cy in cands[:15]:
            flag = "✓" if worst <= 0.03 else "⚠"
            print(f"  {flag} ({cx:+.1f},{cy:+.1f}) {n:5d} {worst*100:9.2f} cm/px")
        print(f"\n共 {len(cands)} 个站位可选，上表按可见点数与精度排序。")
        print("用 python tools\\layout_check.py --x <X> --y <Y> 看某个站位的逐点明细。")
        return 0

    n, worst, beh = report(fm, a.height, a.pitch, a.x, a.y, a.yaw, fx, fy, W, Hpx)
    if n < 3:
        print(f"\n✗ 只有 {n} 个可见标志点，少于定位所需的下限 3 个")
        return 1
    print(f"\n✓ 可见 {n} 个标志点，最差反投影精度 {worst*100:.2f} cm/px")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
