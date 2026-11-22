# -*- coding: utf-8 -*-
"""单帧定位：点选结果 -> 去畸变 -> 地面反投影 -> 地图匹配 -> 位姿。

用法：
    python locate.py clicks/xxx.json --camera ../config/camera.yaml --field ../config/field.yaml
    python locate.py clicks/  --out ../out        # 批量处理整个目录
"""
import argparse
import glob
import json
import os

import cv2
from imgio import imread, project_root
import numpy as np

from geometry import GroundProjector, solve_pose_with_matching
from viz import MapView, flat_markers, load_camera, load_field


def undistort_pixels(pts_uv, K, dist):
    """像素 -> 去畸变后的像素（仍用同一个 K 投影）。"""
    if not pts_uv:
        return []
    pts = np.array(pts_uv, dtype=np.float32).reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts, K, dist, P=K).reshape(-1, 2)
    return [tuple(map(float, p)) for p in und]


def run_one(clicks_path, cam, markers_map, bounds, out_dir=None, save=True):
    with open(clicks_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    img_path = data["image"]
    points = data["points"]

    proj = GroundProjector(
        cam["K"], cam["height_m"], cam["pitch_deg"], cam["roll_deg"], cam["yaw_deg"],
        max_range_m=float(cam["proj"].get("max_range_m", 6.0)),
        min_range_m=float(cam["proj"].get("min_range_m", 0.3)),
    )

    img = imread(img_path)
    print(f"\n=== {os.path.basename(clicks_path)} -> {os.path.basename(img_path)} ===")
    print(f"  视场地面距离: {proj.ground_band(cam['width'], cam['height'])[0]:.2f} ~ "
          f"{proj.ground_band(cam['width'], cam['height'])[1]:.2f} m")

    uv = [(p["u"], p["v"]) for p in points]
    uv_und = undistort_pixels(uv, cam["K"], cam["dist"])

    observations = []
    for p, (u, v), (ui, vi) in zip(points, uv, uv_und):
        g = proj.ground_from_pixel(ui, vi)
        if g is None:
            print(f"  [{p['type']}] 像素({u:.1f},{v:.1f}) -> 在地面视场之外，判为无效")
            continue
        X, Y, d = g
        print(f"  [{p['type']}] 像素({u:.1f},{v:.1f}) 去畸变({ui:.1f},{vi:.1f}) "
              f"-> 载体系 X={X:+.3f} Y={Y:+.3f} m (距离 {d:.2f} m)")
        observations.append({"type": p["type"], "x": X, "y": Y, "u": u, "v": v})

    res = solve_pose_with_matching(
        observations, flat_markers(markers_map),
        max_residual_m=float(cam["loc"].get("max_residual_m", 0.6)),
        max_ambiguity_margin=float(cam["loc"].get("ambiguity_margin_m", 0.20)),
        max_ambiguity_pose=float(cam["loc"].get("ambiguity_pose_m", 0.5)),
        max_ambiguity_yaw=float(cam["loc"].get("ambiguity_yaw_deg", 15.0)),
        max_range_m=float(cam["proj"].get("max_range_m", 6.0)),
    )

    extra = ""
    if "second_residual" in res and res["second_residual"] is not None:
        extra = (f"  [次优解 rms={res['second_residual']:.3f} m, "
                 f"相距 {res.get('second_dx', 0):.2f} m / {res.get('second_dyaw', 0):.1f} deg]")
    print(f"  --> status={res['status']}  valid={res['valid']}  "
          f"markers={res['marker_count']}{extra}")
    if res["valid"]:
        print(f"      相机地图位姿: x={res['x']:.3f} m  y={res['y']:.3f} m  "
              f"yaw={res['yaw_deg']:.1f} deg  residual={res['residual']:.3f} m")
        for a in res["assignments"]:
            o, m = a["obs"], a["map"]
            print(f"      {a['type']}: 观测({o['x']:+.2f},{o['y']:+.2f}) "
                  f"<-> 地图({m['x']:.2f},{m['y']:.2f})")

    if save and out_dir:
        os.makedirs(out_dir, exist_ok=True)
        view = MapView(bounds, size=(720, int(cam["height"])))
        right = view.render(markers_map, res if res["valid"] else None)
        left = img.copy()
        for o in observations:
            cv2.circle(left, (int(o["u"]), int(o["v"])), 8, (0, 0, 255), 2)
            cv2.putText(left, f"{o['type']} ({o['x']:+.2f},{o['y']:+.2f})",
                        (int(o["u"]) + 10, int(o["v"]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(left, f"{res['status']}  markers={res['marker_count']}",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        right = cv2.resize(right, (left.shape[1], left.shape[0]))
        combo = np.hstack([left, right])
        stem = os.path.splitext(os.path.basename(clicks_path))[0]
        out_path = os.path.join(out_dir, f"{stem}_locate.png")
        cv2.imwrite(out_path, combo)
        print(f"  [SAVE] {out_path}")

    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clicks", help="clicks json 或目录")
    ap.add_argument("--camera", default="../config/camera.yaml")
    ap.add_argument("--field", default="../config/field.yaml")
    ap.add_argument("--out", default="../out")
    args = ap.parse_args()

    cam = load_camera(args.camera)
    markers_map, bounds = load_field(args.field, cam.get("field_frame"))

    if os.path.isdir(args.clicks):
        files = sorted(glob.glob(os.path.join(args.clicks, "*.json")))
    else:
        files = [args.clicks]
    if not files:
        print("[ERROR] 没找到 clicks json")
        return
    for f in files:
        try:
            run_one(f, cam, markers_map, bounds, args.out)
        except Exception as e:  # 单张失败不影响批量
            print(f"[ERROR] {f}: {e}")


if __name__ == "__main__":
    main()
