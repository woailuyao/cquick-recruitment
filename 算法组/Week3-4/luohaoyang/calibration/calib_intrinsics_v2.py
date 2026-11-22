"""Camera intrinsics calibration with a physical sanity check.

Why: the old result had fx/fy = 1.175 and k3 = 13.4, which is a degenerate fit.
This script tries several constraint sets and validates the result.

Usage:
    python scripts/calib_intrinsics_v2.py --images calibration/intrinsics/p_frames_selected
    python scripts/calib_intrinsics_v2.py --report ../calibration/camera_intrinsics.yaml
    python scripts/calib_intrinsics_v2.py --images ... --drop-worst 0.15
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
DEFAULT_OUT = ROOT.parent / "calibration" / "camera_intrinsics.yaml"


def detect_corners(paths, cols, rows):
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    data, size = [], None
    for p in paths:
        img = cv2.imread(str(p))
        if img is None:
            print(f"  [skip] cannot read {p.name}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        blur = cv2.Laplacian(gray, cv2.CV_64F).var()
        ok, corners = False, None
        if hasattr(cv2, "findChessboardCornersSB"):
            ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), None)
        if not ok:
            ok, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
            if ok:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
        if not ok:
            print(f"  [skip] board not found: {p.name}")
            continue
        data.append({"name": p.name, "corners": corners, "objp": objp.copy(),
                     "blur": float(blur)})
    return data, size


def view_error(objp, corners, rvec, tvec, K, D):
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
    e = np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
    return float(e.mean())


def calibrate(data, size, flags, guess=None):
    op = [d["objp"] for d in data]
    ip = [d["corners"] for d in data]
    if guess is None:
        rms, K, D, rv, tv = cv2.calibrateCamera(op, ip, size, None, None, flags=flags)
    else:
        rms, K, D, rv, tv = cv2.calibrateCamera(
            op, ip, size, guess.copy(), np.zeros((5, 1)), flags=flags)
    errs = [view_error(d["objp"], d["corners"], rv[i], tv[i], K, D)
            for i, d in enumerate(data)]
    return {"rms": float(rms), "K": K, "D": D, "rvecs": rv, "tvecs": tv,
            "errs": errs, "flags": flags}


def quality(res, size):
    fx, fy = res["K"][0, 0], res["K"][1, 1]
    cx, cy = res["K"][0, 2], res["K"][1, 2]
    ratio = fx / fy if fy else 999
    return {
        "fx": fx, "fy": fy, "cx": cx, "cy": cy, "fx_fy": ratio,
        "rms": res["rms"],
        "ok": (0.98 <= ratio <= 1.02 and res["rms"] < 0.8
               and abs(cx - size[0] / 2) < 0.12 * size[0]
               and abs(cy - size[1] / 2) < 0.12 * size[1]),
    }


def print_advice(q, size):
    print("\nHow to fix:")
    if not (0.98 <= q["fx_fy"] <= 1.02):
        print(f"  * fx/fy={q['fx_fy']:.3f} is far from 1: mount the board on a rigid"
              " plate (acrylic/MDF), not paper on cardboard; tilt it 20-45 deg"
              " around BOTH axes while shooting")
    if abs(q["cx"] - size[0] / 2) > 0.12 * size[0] or abs(q["cy"] - size[1] / 2) > 0.12 * size[1]:
        print(f"  * principal point ({q['cx']:.0f},{q['cy']:.0f}) is far from image"
              " center: make sure the board reaches the image corners")
    if q["rms"] > 0.8:
        print(f"  * RMS={q['rms']:.2f}px is large: sharper printing / bigger squares,"
              " avoid defocus and motion blur")
    print("  * target: fx/fy in [0.99,1.01], cx=640+-20, cy=360+-20, RMS<0.5px")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default=str(ROOT.parent / "calibration" / "intrinsics" / "p_frames_selected"))
    ap.add_argument("--cols", type=int, default=8)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square-size", type=float, default=1.0,
                    help="checker square size (mm or m); K does not depend on it")
    ap.add_argument("--output", default=str(DEFAULT_OUT))
    ap.add_argument("--drop-worst", type=float, default=0.0,
                    help="drop this fraction of worst views and refit, e.g. 0.15")
    ap.add_argument("--fix-k3", action="store_true",
                    help="force k3=0 (4-coefficient model; usually fits identically "
                         "and stays monotonic across the whole image)")
    ap.add_argument("--report", default=None, help="only audit an existing yaml")
    args = ap.parse_args()

    if args.report:
        with open(args.report, "r", encoding="utf-8") as f:
            y = yaml.safe_load(f)
        K = np.array(y["camera_matrix"], dtype=np.float64)
        D = np.array(y["distortion_coefficients"], dtype=np.float64)
        size = (int(y.get("image_width", 1280)), int(y.get("image_height", 720)))
        res = {"rms": float(y.get("rms_reprojection_error_px", 99)), "K": K, "D": D}
        q = quality(res, size)
        print(f"file: {args.report}")
        print(f"fx={q['fx']:.2f} fy={q['fy']:.2f} fx/fy={q['fx_fy']:.4f} "
              f"cx={q['cx']:.1f} cy={q['cy']:.1f} rms={q['rms']:.3f}")
        print("distortion:", np.round(D, 4).tolist())
        print("verdict:", "PASS" if q["ok"] else "FAIL")
        if not q["ok"]:
            print_advice(q, size)
        return

    img_dir = Path(args.images)
    paths = sorted(p for p in img_dir.iterdir()
                   if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"})
    if not paths:
        raise SystemExit(f"no images in {img_dir}")
    print(f"reading {len(paths)} images from {img_dir}")
    data, size = detect_corners(paths, args.cols, args.rows)
    if len(data) < 6:
        raise SystemExit(f"too few usable views: {len(data)}")
    print(f"usable views: {len(data)}, image {size[0]}x{size[1]}")

    base = calibrate(data, size, cv2.CALIB_FIX_K3 if args.fix_k3 else 0)
    tilts = []
    for rvec in base["rvecs"]:
        R, _ = cv2.Rodrigues(rvec)
        tilts.append(math.degrees(math.acos(min(1.0, abs(R[2, 2])))))
    print(f"board tilt: min={min(tilts):.1f} max={max(tilts):.1f} "
          f"median={np.median(tilts):.1f} deg; views below 15 deg: "
          f"{sum(t < 15 for t in tilts)}")

    results = [("A unconstrained", base)]
    qA = quality(base, size)
    if not qA["ok"]:
        f0 = 0.5 * (base["K"][0, 0] + base["K"][1, 1])
        guess = np.array([[f0, 0, base["K"][0, 2]],
                          [0, f0, base["K"][1, 2]],
                          [0, 0, 1]], dtype=np.float64)
        try:
            results.append(("B fx=fy", calibrate(
                data, size,
                cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO,
                guess=guess)))
        except cv2.error as e:
            print("  B failed:", e)
        try:
            results.append(("C fx=fy, k3=0", calibrate(
                data, size,
                cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_K3,
                guess=guess)))
        except cv2.error as e:
            print("  C failed:", e)

    print("\ncandidates:")
    best = None
    for name, r in results:
        q = quality(r, size)
        flag = "OK  " if q["ok"] else "    "
        print(f" {flag}{name:<18} fx={q['fx']:8.2f} fy={q['fy']:8.2f} "
              f"fx/fy={q['fx_fy']:.3f} cx={q['cx']:6.1f} cy={q['cy']:6.1f} "
              f"rms={q['rms']:.3f}")
        if q["ok"] and (best is None or q["rms"] < best[2]["rms"]):
            best = (name, r, q)

    if best is None:
        print("\nno candidate passed the sanity check.")
        print_advice(qA, size)
        print("\nsaving the unconstrained result anyway (NOT recommended for localization).")
        best = ("A unconstrained", base, qA)

    name, res, q = best
    if args.drop_worst > 0:
        order = np.argsort(res["errs"])
        keep = max(6, int(len(data) * (1 - args.drop_worst)))
        picked = [data[i] for i in sorted(order[:keep])]
        print(f"\nre-fitting without the {len(data)-keep} worst views ({keep} left)")
        res2 = calibrate(picked, size, res["flags"],
                         guess=res["K"] if res["flags"] else None)
        q2 = quality(res2, size)
        if q2["ok"] and q2["rms"] <= q["rms"] + 0.05:
            name, res, q = name + " + drop worst", res2, q2
            data = picked

    out = Path(args.output)
    if not q["ok"]:
        # 不合格就不要覆盖正在用的内参，另存一份 _FAIL 方便对比
        out = out.with_name(out.stem + "_FAIL" + out.suffix)
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "image_width": int(size[0]),
        "image_height": int(size[1]),
        "camera_matrix": [[float(x) for x in row] for row in res["K"]],
        "distortion_coefficients": [float(x) for x in res["D"].ravel()],
        "distortion_model": "plumb_bob",
        "rms_reprojection_error_px": float(res["rms"]),
        "fx_fy_ratio": float(q["fx_fy"]),
        "pattern_cols": args.cols,
        "pattern_rows": args.rows,
        "square_size": args.square_size,
        "calibration_strategy": name,
        "quality_pass": bool(q["ok"]),
        "date": datetime.date.today().isoformat(),
        "used_images": [d["name"] for d in data],
        "notes": "generated by calib_intrinsics_v2.py; K is independent of square_size",
    }
    out.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
                   encoding="utf-8")
    print(f"\nverdict: {'PASS' if q['ok'] else 'FAIL'}   strategy={name}")
    print(f"fx={q['fx']:.2f} fy={q['fy']:.2f} fx/fy={q['fx_fy']:.4f} "
          f"cx={q['cx']:.1f} cy={q['cy']:.1f} rms={q['rms']:.3f}")
    print(f"written: {out}")
    if not q["ok"]:
        print_advice(q, size)


if __name__ == "__main__":
    main()