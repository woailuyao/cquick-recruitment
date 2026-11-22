"""Verify extrinsics against measured ground points WITHOUT refitting.

Use it two ways:

  1) hold-out check: fit extrinsics on 6 points, then run this on 3 other points
  2) sanity check:    run this on the same points (should look good, but it proves nothing)

    python scripts/verify_extrinsics.py ^
        --points calib_ground/points_holdout.yaml ^
        --intrinsics ..\calibration\camera_intrinsics.yaml ^
        --extrinsics ..\calibration\camera_extrinsics.yaml

Reports, per point:
    pixel error  : how well the 3D point reprojects onto the image
    ground error : distance between the tape-measured (x,y) and what the pipeline
                   would recover from that pixel  <- this is what actually matters
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.ground_projection import GroundProjector  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--points", default="calib_ground/points.yaml")
    ap.add_argument("--intrinsics", default=str(ROOT.parent / "calibration" / "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics", default=str(ROOT.parent / "calibration" / "camera_extrinsics.yaml"))
    ap.add_argument("--max-ground-err", type=float, default=0.05,
                    help="pass threshold for ground error at <=3 m, metres")
    args = ap.parse_args()

    with open(args.intrinsics, "r", encoding="utf-8") as f:
        intr = yaml.safe_load(f)
    K = np.array(intr["camera_matrix"], dtype=np.float64)
    D = np.array(intr["distortion_coefficients"], dtype=np.float64).reshape(-1, 1)

    with open(args.extrinsics, "r", encoding="utf-8") as f:
        extr = yaml.safe_load(f)
    if "R_base_camera" in extr and "camera_origin_base_m" in extr:
        R_bc = np.array(extr["R_base_camera"], dtype=np.float64).reshape(3, 3)
        O = np.array(extr["camera_origin_base_m"], dtype=np.float64).reshape(3)
        src = extr.get("method", "solvePnP")
    else:
        # 退回到欧拉角外参（手填的那份），只做量级检查
        from src.ground_projection import euler_to_R
        pitch = float(extr.get("pitch_deg", 0.0))
        roll = float(extr.get("roll_deg", 0.0))
        yaw = float(extr.get("yaw_deg", 0.0))
        h = float(extr.get("camera_height_m", 0.0))
        O = np.array([float(extr.get("translation_x_m", 0.0)),
                      float(extr.get("translation_y_m", 0.0)),
                      float(extr.get("translation_z_m", h))])
        R_bc = euler_to_R(pitch, roll, yaw)
        src = f"euler height={h} pitch={pitch} roll={roll} yaw={yaw} (hand-filled?)"
        print("! extrinsics has no R_base_camera; falling back to the euler values")
    R_cb = R_bc.T

    proj = GroundProjector(K, D, R_bc, O)
    print(f"intrinsics : {args.intrinsics}")
    print(f"extrinsics : {args.extrinsics}  (height {O[2]:.3f} m, {src})")
    print(f"points     : {args.points}\n")

    with open(args.points, "r", encoding="utf-8") as f:
        spec = yaml.safe_load(f) or {}
    rows = []
    for view in spec.get("views", []):
        for p in view["points"]:
            rows.append((view.get("image", "?"), p))
    if not rows:
        raise SystemExit("no points in file")

    print(f"{'#':>2} {'truth x,y (m)':>18} {'pixel':>7} {'ground err':>10} "
          f"{'truth r':>8} {'proj r':>7} {'range err':>10}")
    px_err, gd_err, rg_err = [], [], []
    for i, (img, p) in enumerate(rows, 1):
        Xb = np.array([p["x"], p["y"], 0.0])
        Xc = R_cb @ (Xb - O)
        uv, _ = cv2.projectPoints(Xc.reshape(1, 3), np.zeros(3), np.zeros(3), K, D)
        u_p, v_p = uv.reshape(2)
        e_px = math.hypot(u_p - p["u"], v_p - p["v"])

        back = proj.project(p["u"], p["v"])
        if back is None:
            print(f"{i:2d} ({p['x']:+6.2f},{p['y']:+6.2f})  {e_px:7.2f}px       "
                  f"no intersection")
            px_err.append(e_px)
            continue
        e_gd = math.hypot(back["x"] - p["x"], back["y"] - p["y"])
        r_truth = math.hypot(p["x"], p["y"])
        e_rg = back["range_m"] - r_truth
        px_err.append(e_px)
        gd_err.append(e_gd)
        rg_err.append(e_rg)
        print(f"{i:2d} ({p['x']:+6.2f},{p['y']:+6.2f})  {e_px:7.2f}px "
              f"{e_gd*100:8.1f} cm {r_truth:7.2f} m {back['range_m']:6.2f} m "
              f"{e_rg*100:+9.1f} cm")

    px_err = np.array(px_err)
    gd_err = np.array(gd_err) if gd_err else np.array([9.9])
    rg_err = np.array(rg_err) if rg_err else np.array([9.9])
    print(f"\npixel error  : mean {px_err.mean():.2f}  max {px_err.max():.2f} px")
    print(f"ground error : mean {gd_err.mean()*100:.1f}  max {gd_err.max()*100:.1f} cm  "
          f"({len(gd_err)} viable points)")
    print(f"range error  : mean {rg_err.mean()*100:+.1f}  max |{np.abs(rg_err).max()*100:.1f}| cm"
          f"   (projected minus measured distance)")
    ok = gd_err.mean() <= args.max_ground_err
    print(f"verdict      : {'PASS' if ok else 'FAIL'} "
          f"(threshold {args.max_ground_err*100:.0f} cm mean ground error)")
    if not ok:
        print("\nwhat usually causes this:")
        print("  * extrinsics fitted with an old/bad intrinsics file")
        print("  * camera moved / bumped after calibration")
        print("  * points clicked on the wrong line intersection")
        print("  * ground not flat where the points were measured")


if __name__ == "__main__":
    main()