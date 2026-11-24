#!/usr/bin/env python3

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

COLORS = {
    "LCross": "#d62728",
    "TCross": "#1f77b4",
    "XCross": "#2ca02c",
    "PenaltyPoint": "#9467bd",
}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field.yaml"))
    ap.add_argument("--out", default=os.path.join(ROOT, "config", "field_preview.png"))
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    with open(args.field, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    markers = data.get("markers") or []
    if not markers:
        print("[错误] field.yaml 里没有 markers，先用 build_field_map.py add 添加")
        return 1

    fig, ax = plt.subplots(figsize=(7, 6))
    handles = {}

    size = data.get("field_size") or {}
    length = float(size.get("length_m", 0) or 0)
    width = float(size.get("width_m", 0) or 0)
    for m in markers:
        t = m.get("type", "?")
        x, y = float(m["x"]), float(m["y"])
        color = COLORS.get(t, "#333333")
        ax.scatter([x], [y], s=60, color=color, zorder=3)
        ax.annotate(f"{t}\n({x:.2f},{y:.2f})", (x, y), textcoords="offset points",
                    xytext=(6, 8), fontsize=7, ha="left")
        handles[t] = color
        print(f"{t:<14} x={x:8.3f}  y={y:8.3f}")

    if length > 0 and width > 0:
        ax.add_patch(plt.Rectangle((0, -width / 2), length, width,
                                   fill=False, linestyle="--", linewidth=1, color="#999999"))
        geom_path = os.path.join(os.path.dirname(os.path.abspath(args.field)), "field_geometry.yaml")
        if os.path.exists(geom_path):
            try:
                geom = yaml.safe_load(open(geom_path, encoding="utf-8")) or {}
                cc = geom.get("center_circle") or {}
                r = float(cc.get("radius_m", 0) or 0)
                if r > 0:
                    ax.add_patch(plt.Circle((0, 0), r, fill=False, linestyle=":",
                                            linewidth=1, color="#666666"))
            except Exception:
                pass
        ax.annotate("GOAL", (length, 0), textcoords="offset points",
                    xytext=(8, 0), fontsize=8, color="#d62728", va="center")
        ax.set_xlim(-0.6, length + 0.9)
        ax.set_ylim(-width / 2 - 0.6, width / 2 + 0.6)

    ax.axhline(0, color="#cccccc", linewidth=0.8)
    ax.axvline(0, color="#cccccc", linewidth=0.8)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"field map: {data.get('map_version', 'unnamed')}  (Y up, CCW positive)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    for t, c in handles.items():
        ax.scatter([], [], color=c, label=t)
    if handles:
        ax.legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"已保存 {args.out}")
    if args.show:
        plt.show()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
