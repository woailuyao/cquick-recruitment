#!/usr/bin/env python3

import argparse
import csv
import math

def load_csv(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))

def yaw_error(a, b):
    d = abs((a - b) % 360.0)
    return min(d, 360.0 - d)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--truth", required=True)
    ap.add_argument("--pred", required=True)
    ap.add_argument("--max-dist", type=float, default=0.5)
    ap.add_argument("--max-yaw", type=float, default=10.0)
    args = ap.parse_args()

    truth = {r["name"]: r for r in load_csv(args.truth)}
    pred = {r["name"]: r for r in load_csv(args.pred)}

    dists, yaws = [], []
    print(f"{'name':<8}{'dist(m)':>10}{'yaw_err(deg)':>14}  status")
    for name, t in truth.items():
        p = pred.get(name)
        if p is None:
            print(f"{name:<8}{'--':>10}{'--':>14}  缺预测")
            continue
        d = math.hypot(float(p["x"]) - float(t["x"]), float(p["y"]) - float(t["y"]))
        ye = yaw_error(float(p["yaw_deg"]), float(t["yaw_deg"]))
        dists.append(d)
        yaws.append(ye)
        print(f"{name:<8}{d:10.3f}{ye:14.2f}  {p.get('status', '')}")

    if not dists:
        print("没有可评估的样本")
        return 1

    print()
    print(f"样本 {len(dists)} 个")
    print(f"平面误差  均值 {sum(dists)/len(dists):.3f} m，最大 {max(dists):.3f} m")
    print(f"航向误差  均值 {sum(yaws)/len(yaws):.2f}°，最大 {max(yaws):.2f}°")
    ok = sum(1 for d, y in zip(dists, yaws) if d <= args.max_dist and y <= args.max_yaw)
    print(f"达标（≤{args.max_dist} m 且 ≤{args.max_yaw}°）：{ok}/{len(dists)}")
    return 0 if ok == len(dists) else 1

if __name__ == "__main__":
    raise SystemExit(main())
