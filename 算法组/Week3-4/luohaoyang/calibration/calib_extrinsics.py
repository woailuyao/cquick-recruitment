"""Camera extrinsics: measured ground points + solvePnP.

Frames (must stay consistent):
    base   : x forward (optical axis projected on the ground), y left, z up, ground z=0
    camera : OpenCV, x right, y down, z forward

Step 1 - click ground points (camera must be mounted and locked):

    python scripts/calib_extrinsics.py --image calib_ground/cam_ground_01.jpg ^
        --expect calib_ground/points_01.txt --save calib_ground/points.yaml

    points_01.txt holds one "x y" per line (metres, base frame), in click order.

Step 2 - solve:

    python scripts/calib_extrinsics.py --points calib_ground/points.yaml ^
        --intrinsics ../calibration/camera_intrinsics.yaml ^
        --out ../calibration/camera_extrinsics.yaml

Use >= 6 points spread over the whole view. RMS < 1.5 px is good, < 3 px is the
bare minimum for localization.
"""

from __future__ import annotations

import argparse
import datetime
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INTR = ROOT.parent / "calibration" / "camera_intrinsics.yaml"
DEFAULT_EXTR = ROOT.parent / "calibration" / "camera_extrinsics.yaml"


def load_intrinsics(path):
    with open(path, "r", encoding="utf-8") as f:
        y = yaml.safe_load(f)
    K = np.array(y["camera_matrix"], dtype=np.float64)
    D = np.array(y["distortion_coefficients"], dtype=np.float64).reshape(-1, 1)
    return K, D, y


# ----------------------------------------------------------------------
def capture(args):
    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"cannot read image: {args.image}")
    coords = []
    with open(args.expect, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            coords.append((float(parts[0]), float(parts[1])))
    print(f"click these {len(coords)} points in order:")
    for i, (x, y) in enumerate(coords):
        print(f"  {i+1:2d}. x={x:+.3f}  y={y:+.3f}")

    H, W = img.shape[:2]
    scale = min(1.0, 1400.0 / W)
    disp = cv2.resize(img, (int(W * scale), int(H * scale))) if scale < 1 else img.copy()
    picks = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(picks) < len(coords):
            picks.append((x / scale, y / scale))
            k = len(picks) - 1
            print(f"  point {k+1}: pixel=({x/scale:.1f},{y/scale:.1f}) "
                  f"-> base=({coords[k][0]:+.3f},{coords[k][1]:+.3f})")

    win = "click ground points"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)
    while True:
        canvas = disp.copy()
        for i, (px, py) in enumerate(picks):
            p = (int(px * scale), int(py * scale))
            cv2.circle(canvas, p, 5, (0, 0, 255), -1)
            cv2.putText(canvas, f"{i+1}:({coords[i][0]:.2f},{coords[i][1]:.2f})",
                        (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 255), 1)
        cv2.putText(canvas, f"clicked {len(picks)}/{len(coords)}  s=save  r=reset  q=quit",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
        cv2.imshow(win, canvas)
        k = cv2.waitKey(20) & 0xFF
        if k == ord("r"):
            picks.clear()
            print("  cleared")
        elif k == ord("s"):
            if len(picks) != len(coords):
                print(f"  {len(coords)-len(picks)} points to go")
                continue
            break
        elif k in (27, ord("q")):
            cv2.destroyAllWindows()
            raise SystemExit("cancelled, nothing saved")
    cv2.destroyAllWindows()

    views = []
    save_path = Path(args.save)
    if save_path.exists():
        with open(save_path, "r", encoding="utf-8") as f:
            old = yaml.safe_load(f) or {}
        views = old.get("views", [])
        print(f"appending as view #{len(views)+1}")
    pts = [{"u": round(u, 2), "v": round(v, 2), "x": x, "y": y}
           for (u, v), (x, y) in zip(picks, coords)]
    views.append({"image": str(Path(args.image).name), "points": pts})
    out = {"ok": True, "base_frame": "x forward / y left / z up, ground z=0",
           "views": views}
    save_path.parent.mkdir(parents=True, exist_ok=True)
    save_path.write_text(yaml.safe_dump(out, sort_keys=False, allow_unicode=True),
                         encoding="utf-8")
    print(f"saved {len(pts)} points to {args.save}")
    print("tip: take a second image from another spot, append with the same --save")


# ----------------------------------------------------------------------
def euler_from_R(R):
    R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    rel = R @ R0.T
    sp = max(-1.0, min(1.0, -rel[2, 0]))
    pitch = math.degrees(math.asin(sp))
    roll = math.degrees(math.atan2(rel[2, 1], rel[2, 2]))
    yaw = math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
    return pitch, roll, yaw


def solve(args):
    with open(args.points, "r", encoding="utf-8") as f:
        spec = yaml.safe_load(f) or {}
    views = spec.get("views", [])
    if not views:
        raise SystemExit("no views in points file")

    K, D, _ = load_intrinsics(args.intrinsics)
    base_pts, img_pts, names = [], [], []
    for vi, view in enumerate(views):
        img_path = Path(args.points).parent / view["image"]
        if img_path.exists():
            img = cv2.imread(str(img_path))
            if img is not None and img.shape[1] != int(K[0, 2] * 2):
                print(f"  [warn] {view['image']} is {img.shape[1]}x{img.shape[0]}, "
                      f"check it matches the intrinsics resolution")
        for p in view["points"]:
            base_pts.append([p["x"], p["y"], 0.0])
            img_pts.append([p["u"], p["v"]])
            names.append(f"v{vi+1}p{len(base_pts)}")

    base_pts = np.array(base_pts, dtype=np.float64)
    img_pts = np.array(img_pts, dtype=np.float64)
    if len(base_pts) < 6:
        raise SystemExit(f"only {len(base_pts)} points, need >= 6 (8-12 is better)")

    ok, rvec, tvec = cv2.solvePnP(base_pts, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(base_pts, img_pts, K, D)

    R_cam_base, _ = cv2.Rodrigues(rvec)          # X_cam = R * X_base + t
    t_cam_base = tvec.reshape(3)
    R_base_cam = R_cam_base.T
    O_base = -R_base_cam @ t_cam_base            # camera centre in base frame

    proj, _ = cv2.projectPoints(base_pts, rvec, tvec, K, D)
    err = np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)
    rms = float(math.sqrt(float(np.mean(err ** 2))))

    print("\nper-point reprojection error:")
    for n, e, b, p in zip(names, err, base_pts, img_pts):
        print(f"  {n:<8} base=({b[0]:+.2f},{b[1]:+.2f})  "
              f"pixel=({p[0]:6.1f},{p[1]:6.1f})  err={e:5.2f}px")
    print(f"\nRMS = {rms:.3f} px   points = {len(base_pts)}")
    if rms > 3.0:
        print("! RMS > 3px: extrinsics unusable - wrong clicks, bad intrinsics, "
              "or the camera moved")
    elif rms > 1.5:
        print("! RMS is high, add/redo points; target < 1.5px")

    pitch, roll, yaw = euler_from_R(R_base_cam)
    out = {
        "camera_frame": "camera (OpenCV: x right, y down, z forward)",
        "base_frame": "base_link (x forward, y left, z up, ground z=0)",
        "method": "solvePnP on measured ground points (IPPE)",
        "R_base_camera": [float(v) for v in R_base_cam.reshape(-1)],
        "camera_origin_base_m": [float(v) for v in O_base],
        "camera_height_m": float(O_base[2]),
        "pitch_deg": float(pitch),
        "roll_deg": float(roll),
        "yaw_deg": float(yaw),
        "translation_x_m": float(O_base[0]),
        "translation_y_m": float(O_base[1]),
        "translation_z_m": float(O_base[2]),
        "reprojection_rms_px": rms,
        "n_points": int(len(base_pts)),
        "date": datetime.date.today().isoformat(),
        "measurement_note": f"solvePnP on {len(base_pts)} ground points, "
                            f"RMS {rms:.2f}px; positive pitch means lens tilted down",
    }
    outpath = Path(args.out)
    outpath.parent.mkdir(parents=True, exist_ok=True)
    outpath.write_text(yaml.safe_dump(out, sort_keys=False, allow_unicode=True),
                       encoding="utf-8")
    print(f"\ncamera origin (base): {np.round(O_base,3).tolist()} m")
    print(f"equivalent angles: pitch_down={pitch:.2f} roll={roll:.2f} yaw={yaw:.2f} deg")
    print(f"written: {outpath}")
    if abs(yaw) > 5:
        print("! |yaw| > 5 deg: optical axis is not aligned with base +X. It is fine"
              " because R is stored, but document it in the README")


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=None, help="capture mode: calibration image")
    ap.add_argument("--expect", default=None, help="capture mode: x y list in click order")
    ap.add_argument("--save", default="calib_ground/points.yaml")
    ap.add_argument("--points", default=None, help="solve mode: points.yaml")
    ap.add_argument("--intrinsics", default=str(DEFAULT_INTR))
    ap.add_argument("--out", default=str(DEFAULT_EXTR))
    args = ap.parse_args()

    if args.image and args.expect:
        capture(args)
    elif args.points:
        solve(args)
    else:
        ap.print_help()
        raise SystemExit("\nneed (--image + --expect) or --points")


if __name__ == "__main__":
    main()