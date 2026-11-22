"""地图体检：尺寸 / 间距 / 对称性 / 可定位性。

回答三个问题：
    1. 这份 field.yaml 的尺度对不对（能不能和卷尺量出来的对上）
    2. 这张图有没有对称歧义（180 度、镜像）
    3. 站在这张图的哪些位置，算法能唯一定位、哪些会 ambiguous

支持两种格式：
    - 本仓库格式：  markers: [{id, type, x, y}]
    - 官方扁平格式：TCross1: [5.955, 2.930]

    python scripts/check_map.py                     # 查本仓库 config/field.yaml
    python scripts/check_map.py --field <path>      # 查任意地图
    python scripts/check_map.py --export out.yaml   # 转成本仓库格式
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))


from field_localize.field_map import FieldMap                      # noqa: E402
from field_localize.pose_match import Locator, Observation         # noqa: E402


def stats(fmap):
    xs = [m.x for m in fmap.markers]
    ys = [m.y for m in fmap.markers]
    print(f"markers : {len(fmap.markers)}")
    print(f"x range : {min(xs):+.3f} .. {max(xs):+.3f}   span {max(xs)-min(xs):.3f} m")
    print(f"y range : {min(ys):+.3f} .. {max(ys):+.3f}   span {max(ys)-min(ys):.3f} m")
    for t in fmap.types_present():
        print(f"  {t:<13} x{len(fmap.by_type(t))}")

    d = sorted(fmap.pairwise_distances(), key=lambda p: p[2])
    print(f"\nclosest marker pairs (a <-> b : distance):")
    for a, b, dist in d[:5]:
        flag = "  <-- too close, easy to confuse" if dist < 0.5 else ""
        print(f"  {a:<10} {b:<10} {dist:6.3f} m{flag}")

    print("\nreference distances to verify with a tape measure:")
    nn = {}
    for m in fmap.markers:
        cand = [(math.hypot(m.x - o.x, m.y - o.y), o)
                for o in fmap.markers if o.id != m.id]
        if not cand:
            continue
        dist, other = min(cand, key=lambda t: t[0])
        key = tuple(sorted([m.id, other.id]))
        nn[key] = dist
    for (a, b), dist in sorted(nn.items(), key=lambda kv: kv[1])[:8]:
        print(f"  {a:<10} <-> {b:<10} {dist:6.3f} m"
              f"  ({fmap.get(a).type} - {fmap.get(b).type})")


def symmetry(fmap, tol=0.10):
    """检查 180 度旋转对称 / 关于中心轴的镜像对称。"""
    xs = [m.x for m in fmap.markers]
    ys = [m.y for m in fmap.markers]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    print(f"\nmap centre: ({cx:+.3f}, {cy:+.3f})")

    transforms = {
        "rotate 180 deg": lambda x, y: (2 * cx - x, 2 * cy - y),
        "mirror about vertical axis": lambda x, y: (2 * cx - x, y),
        "mirror about horizontal axis": lambda x, y: (x, 2 * cy - y),
    }
    results = {}
    for name, fn in transforms.items():
        unmatched = []
        for m in fmap.markers:
            px, py = fn(m.x, m.y)
            same = [o for o in fmap.by_type(m.type)
                    if math.hypot(o.x - px, o.y - py) <= tol]
            if not same:
                unmatched.append(m.id)
        results[name] = unmatched
        if unmatched:
            print(f"  {name:<28} NOT symmetric  (breaks: {', '.join(unmatched[:3])}"
                  f"{' ...' if len(unmatched) > 3 else ''})")
        else:
            print(f"  {name:<28} SYMMETRIC  -> poses cannot be told apart")
    return results


def locatable(fmap, n_pose=60, noise=0.0, seed=7, fov_deg=180.0, max_range=12.0):
    """随机撒位姿，看算法能不能唯一解出来。fov_deg<180 模拟真实视野。"""
    rng = random.Random(seed)
    xs = [m.x for m in fmap.markers]
    ys = [m.y for m in fmap.markers]
    loc = Locator(fmap)
    counts = {}
    worst = []
    half = math.radians(fov_deg) / 2.0
    for _ in range(n_pose):
        tx = rng.uniform(min(xs), max(xs))
        ty = rng.uniform(min(ys), max(ys))
        tyaw = rng.uniform(-180, 180)
        th = math.radians(tyaw)
        c, s = math.cos(th), math.sin(th)
        obs = []
        for m in fmap.markers:
            dx, dy = m.x - tx, m.y - ty
            bx = c * dx + s * dy
            by = -s * dx + c * dy
            r = math.hypot(bx, by)
            if r > max_range:
                continue
            if fov_deg < 180 and abs(math.atan2(by, bx)) > half:
                continue
            if noise:
                bx += rng.gauss(0, noise)
                by += rng.gauss(0, noise)
            obs.append(Observation(m.type, bx, by, 0.9, r))
        r = loc.locate(obs)
        counts[r.status] = counts.get(r.status, 0) + 1
        if r.status != "ok":
            worst.append((round(tx, 2), round(ty, 2), round(tyaw, 1),
                          len(obs), r.status))
    total = sum(counts.values())
    print(f"\nlocatability test ({total} random poses, noise={noise*100:.0f} cm, "
          f"fov={fov_deg:.0f} deg, range<{max_range:.0f} m):")
    for k, v in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"  {k:<22} {v:3d}  ({v/total*100:5.1f}%)")
    if worst:
        print(f"  examples of non-ok poses (x, y, yaw, #obs, status): {worst[:5]}")
    return counts


def export(fmap, path):
    data = {
        "version": f"{fmap.version}-converted",
        "frame": "field_map",
        "unit": "m",
        "origin_note": "converted from official flat field.yaml; axes unchanged",
        "min_inliers": 3,
        "markers": [{"id": m.id, "type": m.type, "x": m.x, "y": m.y}
                    for m in fmap.markers],
    }
    Path(path).write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
                          encoding="utf-8")
    print(f"\nwritten {path}  ({len(fmap.markers)} markers)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", default=str(ROOT / "config" / "field.yaml"))
    ap.add_argument("--export", default=None)
    ap.add_argument("--poses", type=int, default=60)
    ap.add_argument("--noise", type=float, default=0.0, help="observation noise in metres")
    ap.add_argument("--fov", type=float, default=180.0, help="camera horizontal FOV, deg")
    ap.add_argument("--range", type=float, default=12.0, help="max visible range, m")
    args = ap.parse_args()

    fmap = FieldMap.load(args.field)
    print("=" * 66)
    print(fmap.summary())
    print("=" * 66)
    stats(fmap)
    symmetry(fmap)
    locatable(fmap, n_pose=args.poses, noise=args.noise,
              fov_deg=args.fov, max_range=args.range)
    if args.export:
        export(fmap, args.export)


if __name__ == "__main__":
    main()