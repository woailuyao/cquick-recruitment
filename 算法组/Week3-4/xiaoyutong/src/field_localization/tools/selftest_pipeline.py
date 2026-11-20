"""端到端自测：投影 + 匹配 + 时序滤波 是不是自洽。

做法（完全不依赖检测器）：
  1. 随便选一个"真值位姿"（相机在地图里的 x/y/yaw）
  2. 用地图算出这个位姿理论上能看到哪些标志，把它们投到像素上 —— 这就是"完美检测"
  3. 给像素加一点噪声、丢掉一部分
  4. 让主程序把像素反投影回地面、再和地图匹配
  5. 比较解出来的位姿和真值

如果这一步过不了，说明投影/坐标定义有 bug，跟检测器没关系。

用法：
    python tools/selftest_pipeline.py
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
from vision2.field_match import FieldMap, visible_observations  # noqa: E402
from vision2.geometry import rotation_from_height_pitch_roll  # noqa: E402
from vision2.yaml_mini import load_yaml  # noqa: E402

setup_console()

import localize_app as app  # noqa: E402


class Args:
    video = ""
    detector = "classical"
    weights = ""
    conf = 0.4


def main():
    print("== 端到端自测（投影 → 匹配）==")
    intr = load_yaml(app.INTR)
    K = np.asarray(intr["camera_matrix"], float).reshape(3, 3)
    dist = np.asarray(intr["dist_coeffs"], float).reshape(-1, 1)

    R, t = rotation_from_height_pitch_roll(0.85, 20.0, 0.0)
    geom = load_yaml(app.GEOM)
    fmap = FieldMap.load(app.FIELD)

    from vision2.geometry import GroundProjector

    proj = GroundProjector(K, dist, R, t, width=1280, height=720)

    rng = np.random.default_rng(0)
    errs, yerrs, n_ok, n_try = [], [], 0, 200
    for _ in range(n_try):
        x = rng.uniform(1.0, 6.0)
        y = rng.uniform(-3.5, 3.6)
        yaw = rng.uniform(-40, 40)  # 朝球门方向看

        # 真值位姿下能看到哪些标志（载体坐标系）
        vis = visible_observations(fmap, x, y, yaw, max_range=6.0, half_fov_deg=35)
        dets = []
        for o in vis:
            if rng.random() < 0.25:
                continue
            uv = proj.ground_to_pixel(o["x"], o["y"])
            if uv is None:
                continue
            u, v = uv
            if not (10 < u < 1270 and 10 < v < 710):
                continue
            dets.append({"type": o["type"], "u": u + rng.normal(0, 1.5),
                         "v": v + rng.normal(0, 1.5), "conf": 0.9})
        if len(dets) < 3:
            continue

        loc = app.Localizer(Args())
        res, obs, used = loc.process(np.zeros((720, 1280, 3), np.uint8), dets)
        if not res["valid"]:
            continue
        n_ok += 1
        errs.append(math.hypot(res["x"] - x, res["y"] - y))
        yerrs.append(abs((res["yaw_deg"] - yaw + 180) % 360 - 180))

    print("  有效解 %d / %d（其余是标志不够或判为歧义，属正常）" % (n_ok, n_try))
    if errs:
        e = np.array(errs)
        ye = np.array(yerrs)
        print("  位置误差：中位 %.3f m，90%% %.3f m，最大 %.3f m" % (np.median(e), np.percentile(e, 90), e.max()))
        print("  航向误差：中位 %.2f°，90%% %.2f°，最大 %.2f°" % (np.median(ye), np.percentile(ye, 90), ye.max()))
        ok = np.percentile(e, 90) < 0.35 and np.percentile(ye, 90) < 6
        print("")
        print("  结论：%s" % ("通过 —— 像素→地面→地图 这条链路是自洽的" if ok
                              else "不通过 —— 检查外参方向/坐标定义"))
        return 0 if ok else 1
    print("  [失败] 没有有效解，先检查外参和地图是否读进来了")
    return 1


if __name__ == "__main__":
    sys.exit(main())
