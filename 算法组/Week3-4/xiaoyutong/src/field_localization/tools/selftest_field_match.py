"""自测：地图匹配到底准不准（不需要相机、不需要场地）。

做法：在场地里随机取 500 个位姿 → 用地图算出"理论上能看到哪些标志"→
     给观测加噪声、丢掉一部分、再塞几个假的 → 交给匹配算法 → 和真值比。

用法：
    python tools/selftest_field_match.py
"""

import math
import os
import sys

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
from vision2.field_match import FieldMap, bounds_from_geometry, match, visible_observations  # noqa: E402
from vision2.yaml_mini import load_yaml  # noqa: E402

setup_console()

FIELD = os.path.join(ROOT, "config", "field.yaml")
GEOM = os.path.join(ROOT, "config", "field_geometry.yaml")
BOUNDS = bounds_from_geometry(load_yaml(GEOM))


def angle_err(a, b):
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)


def run(trials=400, noise=0.10, drop=0.2, outliers=1, seed=0):
    fmap = FieldMap.load(FIELD)
    rng = np.random.default_rng(seed)
    pos_err, yaw_err = [], []
    status = {}
    for _ in range(trials):
        x = rng.uniform(0.6, 6.4)
        y = rng.uniform(-4.0, 4.1)
        yaw = rng.uniform(-180, 180)
        truth = visible_observations(fmap, x, y, yaw, max_range=6.5, half_fov_deg=42)
        obs = []
        for o in truth:
            if rng.random() < drop:
                continue
            obs.append({
                "type": o["type"],
                "x": o["x"] + rng.normal(0, noise),
                "y": o["y"] + rng.normal(0, noise),
            })
        for _ in range(outliers):
            obs.append({"type": rng.choice(["T", "L", "X"]),
                        "x": rng.uniform(0.5, 6.5), "y": rng.uniform(-4, 4)})
        if len(obs) < 3:
            status["观测太少"] = status.get("观测太少", 0) + 1
            continue
        res = match(obs, fmap, gating=0.8, bounds=BOUNDS)
        status[res["status"]] = status.get(res["status"], 0) + 1
        if res["valid"]:
            pos_err.append(math.hypot(res["x"] - x, res["y"] - y))
            yaw_err.append(angle_err(res["yaw_deg"], yaw))
    return fmap, status, np.array(pos_err), np.array(yaw_err)


def main():
    print("== 地图匹配自测 ==")
    fmap = FieldMap.load(FIELD)
    print("地图特征：%d 个  %s" % (len(fmap), {t: len(fmap.of(t)) for t in ("T", "L", "X", "P", "C")}))
    print("")

    for noise, drop, outliers in ((0.05, 0.0, 0), (0.10, 0.2, 1), (0.20, 0.3, 2)):
        fmap, status, pe, ye = run(noise=noise, drop=drop, outliers=outliers, seed=3)
        n = len(pe)
        print("噪声 %.2f m，丢 %.0f%% 观测，%d 个假点：有效解 %d/400" % (noise, drop * 100, outliers, n))
        print("    状态分布：%s" % status)
        if n:
            print("    位置误差：中位 %.3f m，90%% %.3f m，最大 %.3f m"
                  % (np.median(pe), np.percentile(pe, 90), pe.max()))
            print("    航向误差：中位 %.2f°，90%% %.2f°，最大 %.2f°"
                  % (np.median(ye), np.percentile(ye, 90), ye.max()))
        print("")

    print("判读标准：")
    print("  位置 90% 分位 < 0.30 m、航向 90% 分位 < 5° → 匹配算法没问题（考核线是 0.5 m / 10°）")
    print("  如果 valid 比例太低，说明 gating / 阈值太严；太高且误差大，说明阈值太松")
    return 0


if __name__ == "__main__":
    sys.exit(main())
