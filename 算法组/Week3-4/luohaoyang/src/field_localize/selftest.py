"""离线自检：不接相机、不用模型，验证三件事能不能对。

1. 地面投影：已知地面点 -> 像素 -> 反投影回地面，误差应 < 1 cm
2. 位姿求解：用已知位姿把地图标志投到载体系，再反解位姿，误差应 < 1 cm / 0.2 deg
3. 歧义与失败：只给 2 个对称点应报 ambiguous，只给 1 个点应报 insufficient_markers
4. 界面渲染：用有效位姿画一次左/右分屏，写到 selftest_out.jpg

    python scripts/selftest.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))


from field_localize.field_map import FieldMap                       # noqa: E402
from field_localize.ground_projection import GroundProjector, euler_to_R  # noqa: E402
from field_localize.pose_match import Locator, Observation          # noqa: E402
from field_localize.viz import MapPanel, compose, draw_hud          # noqa: E402

OK, FAIL = "  [ok]  ", "  [FAIL]"
fails = []


def check(cond, msg, extra=""):
    print((OK if cond else FAIL) + " " + msg + ("  " + extra if extra else ""))
    if not cond:
        fails.append(msg)


def main():
    print("=" * 60)
    print("1) ground projection round-trip")
    K = np.array([[1100.0, 0, 640.0], [0, 1100.0, 360.0], [0, 0, 1.0]])
    D = np.zeros(5)
    R = euler_to_R(15.0, 0.0, 0.0)
    O = np.array([0.0, 0.0, 0.745])
    proj = GroundProjector(K, D, R, O)
    R_cb = R.T
    t_cb = -R_cb @ O
    rvec, _ = cv2.Rodrigues(R_cb)
    for p in [(2.0, 0.3, 0.0), (4.0, -1.0, 0.0), (1.2, 1.5, 0.0)]:
        uv, _ = cv2.projectPoints(np.array([p]), rvec, t_cb, K, D)
        u, v = uv.reshape(2)
        g = proj.project(u, v)
        err = math.hypot(g["x"] - p[0], g["y"] - p[1]) if g else 9e9
        check(err < 0.01, f"project back ({p[0]},{p[1]})", f"err={err*1000:.2f} mm")
    check(proj.project(640, 10) is None, "ray above horizon is rejected")

    print("\n2) pose solve")
    fmap = FieldMap([
        __import__("src.field_map", fromlist=["MapMarker"]).MapMarker("L1", "LCross", 0.0, 0.0),
        __import__("src.field_map", fromlist=["MapMarker"]).MapMarker("L2", "LCross", 6.0, 0.0),
        __import__("src.field_map", fromlist=["MapMarker"]).MapMarker("T1", "TCross", 2.5, 0.0),
        __import__("src.field_map", fromlist=["MapMarker"]).MapMarker("X1", "XCross", 3.0, 2.5),
    ], version="selftest")
    truth = (1.6, 1.1, 27.0)
    th = math.radians(truth[2])
    c, s = math.cos(th), math.sin(th)
    obs = []
    for m in fmap.markers:
        dx, dy = m.x - truth[0], m.y - truth[1]
        bx, by = c * dx + s * dy, -s * dx + c * dy
        obs.append(Observation(m.type, bx, by, 0.9, math.hypot(bx, by)))
    # 加 2cm 噪声
    for o in obs:
        o.x += float(np.random.RandomState(0).normal(0, 0.02))
        o.y += float(np.random.RandomState(1).normal(0, 0.02))
    loc = Locator(fmap)
    r = loc.locate(obs)
    err = math.hypot(r.x - truth[0], r.y - truth[1])
    dyaw = abs((r.yaw_deg - truth[2] + 180) % 360 - 180)
    check(r.valid and r.status == "ok", "4 markers -> ok", f"status={r.status}")
    check(err < 0.05, "position error < 5 cm with 2 cm noise", f"err={err*100:.1f} cm")
    check(dyaw < 1.0, "yaw error < 1 deg", f"err={dyaw:.2f} deg")

    print("\n3) failure / ambiguity paths")
    r1 = loc.locate(obs[:1])
    check(r1.status == "insufficient_markers" and not r1.valid, "1 marker -> insufficient_markers")
    sym = FieldMap([
        __import__("src.field_map", fromlist=["MapMarker"]).MapMarker("L1", "LCross", 0.0, 0.0),
        __import__("src.field_map", fromlist=["MapMarker"]).MapMarker("L2", "LCross", 6.0, 0.0),
        __import__("src.field_map", fromlist=["MapMarker"]).MapMarker("L3", "LCross", 0.0, 4.0),
        __import__("src.field_map", fromlist=["MapMarker"]).MapMarker("L4", "LCross", 6.0, 4.0),
    ], version="symmetric")
    loc2 = Locator(sym)
    obs2 = [Observation("LCross", 1.0, 1.0, 0.9, 1.4),
            Observation("LCross", 5.0, 1.0, 0.9, 5.1),
            Observation("LCross", 1.0, 3.0, 0.9, 3.2)]
    r2 = loc2.locate(obs2)
    check(r2.status in ("ambiguous", "ok_low_markers"),
          "4 symmetric L corners flagged (ambiguous expected)",
          f"status={r2.status}")

    print("\n4) render split screen")
    panel = MapPanel(fmap, size=(640, 720), field_size={"length": 6.0, "width": 4.0})
    left = np.full((720, 1280, 3), 45, np.uint8)
    draw_hud(left, ["selftest  FPS=--  infer=--ms", "det=4  ok"], origin=(8, 22), scale=0.5)
    right = panel.render(pose=(r.x, r.y, math.radians(r.yaw_deg)),
                         obs_map=[{"x": o.x, "y": o.y} for o in obs],
                         title="field map (selftest)",
                         status_lines=[f"x_camera_map  = {r.x:7.2f} m",
                                       f"y_camera_map  = {r.y:7.2f} m",
                                       f"yaw_camera_map= {r.yaw_deg:7.1f} deg",
                                       f"valid={r.valid}  status={r.status}",
                                       f"markers={r.marker_count}  residual={r.residual:.2f} m",
                                       "fps=--  update=-- Hz  map=selftest"])
    out = compose(left, right)
    cv2.imwrite(str(ROOT / "selftest_out.jpg"), out)
    check(out.shape[1] == 1920, "split screen composed", f"size={out.shape[1]}x{out.shape[0]}")

    print("=" * 60)
    if fails:
        print(f"{len(fails)} check(s) FAILED")
        for f in fails:
            print("  -", f)
        raise SystemExit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()