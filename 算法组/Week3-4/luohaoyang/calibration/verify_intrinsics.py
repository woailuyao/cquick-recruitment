"""Cross-validate camera intrinsics on frames that did NOT take part in calibration.

Calibration RMS only proves "it fits the frames it was fitted on".
The real test: take unseen frames, run solvePnP with the given K/D and check
whether the board reprojects accurately.

    python scripts/verify_intrinsics.py ^
        --intrinsics ..\calibration\camera_intrinsics_qp.yaml ^
        --compare ..\calibration\camera_intrinsics_oldFAIL.yaml ^
        --video F:\video\qp.avi --cols 9 --rows 6

Verdict: mean held-out reprojection RMS
    <= 0.8 px  PASS
    <= 1.5 px  MARGINAL (reshoot with a bigger board)
    >  1.5 px  FAIL
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
IMG_SUFFIX = {".jpg", ".jpeg", ".png", ".bmp"}


def load_intrinsics(path):
    with open(path, "r", encoding="utf-8") as f:
        y = yaml.safe_load(f)
    K = np.array(y["camera_matrix"], dtype=np.float64)
    D = np.array(y["distortion_coefficients"], dtype=np.float64).reshape(-1, 1)
    used = set()
    for name in y.get("used_images", []) or []:
        m = re.search(r"_(\d{4,6})\.(?:jpg|jpeg|png|bmp)$", str(name), re.I)
        if m:
            used.add(int(m.group(1)))
    return K, D, y, used


def board_object_points(cols, rows):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp


def measure(gray, objp, K, D, cols, rows):
    ok, c = cv2.findChessboardCornersSB(gray, (cols, rows), None)
    if not ok:
        ok, c = cv2.findChessboardCorners(gray, (cols, rows), None)
        if ok:
            c = cv2.cornerSubPix(
                gray, c, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    if not ok:
        return None
    okp, rvec, tvec = cv2.solvePnP(objp, c, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not okp:
        okp, rvec, tvec = cv2.solvePnP(objp, c, K, D)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
    err = np.linalg.norm(proj.reshape(-1, 2) - c.reshape(-1, 2), axis=1)
    pts = c.reshape(-1, 2)
    area = ((pts[:, 0].max() - pts[:, 0].min()) *
            (pts[:, 1].max() - pts[:, 1].min())) / (gray.shape[1] * gray.shape[0])
    return {"rms": float(np.sqrt(np.mean(err ** 2))),
            "mean": float(err.mean()), "max": float(err.max()),
            "area": float(area), "corners": pts}


def describe(tag, K, D):
    fx, fy = K[0, 0], K[1, 1]
    print(f"{tag}: fx={fx:.2f} fy={fy:.2f} fx/fy={fx/fy:.4f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}  dist={np.round(D.ravel(),4).tolist()}")
    print(f"      HFOV={2*math.degrees(math.atan(640/fx)):.1f} deg  "
          f"VFOV={2*math.degrees(math.atan(360/fy)):.1f} deg")


def verdict_of(mean_rms):
    if mean_rms <= 0.8:
        return "PASS"
    if mean_rms <= 1.5:
        return "MARGINAL"
    return "FAIL"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--intrinsics", default=str(ROOT.parent / "calibration" / "camera_intrinsics_qp.yaml"))
    ap.add_argument("--compare", default=None, help="second intrinsics file for an A/B comparison")
    ap.add_argument("--video", default=r"F:\video\qp.avi")
    ap.add_argument("--images", default=None, help="directory of held-out frames instead of --video")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--gap", type=int, default=60, help="min frame distance from any calibration frame")
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--limit", type=int, default=40)
    args = ap.parse_args()

    K, D, meta, used = load_intrinsics(args.intrinsics)
    print(f"file       : {args.intrinsics}")
    describe("candidate  ", K, D)
    print(f"calibration used {len(used)} frames")

    sets = [(args.intrinsics, K, D)]
    if args.compare:
        K2, D2, m2, _ = load_intrinsics(args.compare)
        print(f"baseline   : {args.compare}")
        describe("baseline   ", K2, D2)
        sets.append((args.compare, K2, D2))

    # ---- gather held-out frames ----
    frames = []
    if args.video:
        if not used:
            print("! cannot parse frame indices from used_images; "
                  "held-out frames may overlap the calibration set")
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            raise SystemExit(f"cannot open {args.video}")
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        used_arr = np.array(sorted(used)) if used else np.array([-10 ** 9])
        for i in range(0, n, args.step):
            if np.min(np.abs(used_arr - i)) < args.gap:
                continue
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, frame = cap.read()
            if not ok:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if cv2.findChessboardCornersSB(gray, (args.cols, args.rows), None)[0]:
                frames.append((i, gray))
            if len(frames) >= args.limit:
                break
        cap.release()
    else:
        for p in sorted(q for q in Path(args.images).iterdir() if q.suffix.lower() in IMG_SUFFIX):
            gray = cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2GRAY)
            frames.append((p.name, gray))

    if not frames:
        raise SystemExit("no held-out frame with a detectable board "
                         "(check --cols/--rows and the video path)")
    print(f"\nheld-out frames: {len(frames)}  (at least {args.gap} frames away "
          f"from any calibration frame)")

    objp = board_object_points(args.cols, args.rows)
    summary = []
    for path, Kx, Dx in sets:
        rows = []
        for key, gray in frames:
            r = measure(gray, objp, Kx, Dx, args.cols, args.rows)
            if r is not None:
                rows.append((key, r))
        if not rows:
            print(f"  {path}: no valid measurement")
            continue
        arr = np.array([r["rms"] for _, r in rows])
        mx = np.array([r["max"] for _, r in rows])
        ar = np.array([r["area"] for _, r in rows])
        summary.append((path, arr, mx, ar))
        print(f"\n=== {path} ===")
        print(f"  reprojection RMS : mean {arr.mean():.3f}  median {np.median(arr):.3f}  "
              f"p90 {np.percentile(arr,90):.3f}  max {arr.max():.3f} px")
        print(f"  worst corner     : mean {mx.mean():.3f}  max {mx.max():.3f} px")
        print(f"  board area       : mean {ar.mean():.3f}  max {ar.max():.3f}")
        print(f"  frames < 1.0 px  : {(arr<1.0).sum()}/{len(arr)} "
              f"({(arr<1.0).mean()*100:.0f}%)")
        print(f"  verdict          : {verdict_of(arr.mean())}")

    if len(summary) == 2:
        a, b = summary[0], summary[1]
        print("\n=== A/B ===")
        print(f"  candidate mean RMS {a[1].mean():.3f} px vs baseline {b[1].mean():.3f} px")
        ratio = a[1].mean() / max(b[1].mean(), 1e-9)
        print(f"  ratio {ratio:.2f}x "
              f"({'candidate better' if ratio < 1 else 'candidate WORSE'})")

    print("\nNext step (needs the lab):")
    print("  1) after re-calibrating extrinsics, tape two marks at 2 m and 3 m")
    print("  2) run run_localize.py and compare projected (x,y) with the tape")
    print("  3) < 5 cm error at 3 m means intrinsics + extrinsics are both right")
    print("  4) error growing smoothly with distance is fine; a jump means a bad K/D")


if __name__ == "__main__":
    main()