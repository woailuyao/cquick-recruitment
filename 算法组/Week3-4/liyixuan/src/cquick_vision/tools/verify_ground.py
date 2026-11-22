#!/usr/bin/env python3
"""标定验证：鼠标点地面上的点，程序给出载体系米制坐标与地面距离。

用卷尺量出相机正下方到该点的真实距离，两者应对得上：
  2 m 处误差 < 5 cm，5 m 处误差 < 15 cm。超过就重标外参。

用法：
  python tools/verify_ground.py --camera 0
  鼠标左键=取点   c=清空   q=退出
"""
from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402
import numpy as np                  # noqa: E402

from cvlib.camera import open_capture                        # noqa: E402
from cvlib.config import load_extrinsics, load_intrinsics  # noqa: E402
from cvlib.geom import ray_to_ground                     # noqa: E402

POINTS = []          # [(u, v, x, y, D, why)]


def project(u, v, intr, extr):
    K, dist = intr["K"], intr["dist"]
    pts = np.asarray([[[float(u), float(v)]]], np.float64)
    nx, ny = cv2.undistortPoints(pts, K, dist, P=None).reshape(-1, 2)[0]
    d_c = np.array([float(nx), float(ny), 1.0])
    d_c /= np.linalg.norm(d_c)
    d_b = extr["R"] @ d_c
    P, why = ray_to_ground(d_b, extr["t"], max_range=30.0)
    if P is None:
        return None, None, why
    D = math.hypot(float(P[0]) - float(extr["t"][0]), float(P[1]) - float(extr["t"][1]))
    return (float(P[0]), float(P[1])), D, None


def on_mouse(event, x, y, flags, param):
    intr, extr = param
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    P, D, why = project(x, y, intr, extr)
    if P is None:
        POINTS.append((x, y, float("nan"), float("nan"), float("nan"), why or "invalid"))
        print(f"[verify] ({x},{y}) 无效: {why}")
        return
    POINTS.append((x, y, P[0], P[1], D, None))
    print(f"[verify] pixel=({x},{y})  base=({P[0]:.3f},{P[1]:.3f}) m  "
          f"ground_dist={D:.3f} m   卷尺实测=____ m")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="0")
    ap.add_argument("--video", default=None)
    ap.add_argument("--intrinsics", default=os.path.join(
        ROOT, "calibration", "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics", default=os.path.join(
        ROOT, "calibration", "camera_extrinsics.yaml"))
    args = ap.parse_args()

    intr = load_intrinsics(args.intrinsics)
    extr = load_extrinsics(args.extrinsics)
    print(f"[verify] h={extr['height']:.3f}m pitch={extr['pitch_deg']:.2f} "
          f"roll={extr['roll_deg']:.2f} fy={intr['fy']:.2f}")

    src = args.video if args.video else args.camera
    cap = open_capture(src, intr["width"], intr["height"])
    if cap is None:
        print("[error] 打不开视频源")
        return 2

    win = "verify_ground (left click to sample)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse, (intr, extr))

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        vis = frame.copy()
        for (u, v, x, y, D, why) in POINTS:
            if why:
                cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 13, 2)
                continue
            cv2.drawMarker(vis, (int(u), int(v)), (0, 255, 0), cv2.MARKER_CROSS, 15, 2)
            txt = f"({x:.2f},{y:.2f}) D={D:.2f}m"
            cv2.putText(vis, txt, (int(u) + 8, int(v) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(vis, "left click: sample ground point | c: clear | q: quit",
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.imshow(win, vis)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("c"):
            POINTS.clear()
            print("[verify] 清空")

    cap.release()
    cv2.destroyAllWindows()

    valid = [p for p in POINTS if not p[5]]
    if valid:
        print("\n[verify] 采样汇总（把 ground_dist 与卷尺实测对照）")
        for (u, v, x, y, D, _) in valid:
            print(f"  pixel=({u:4d},{v:4d})  base=({x:6.3f},{y:6.3f})  D={D:6.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
