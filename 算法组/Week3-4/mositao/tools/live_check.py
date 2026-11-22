# -*- coding: utf-8 -*-
"""现场一键诊断：抓一帧，分别用两个权重检测，把「像素 -> 地面 -> 最近地图点」全列出来。

为什么需要它：
    现场换一个方位就定位不出来时，只靠终端里的 marker_count 分不清是
    「检测器没出框」「出了框但类别判错」「类别对但位置反投影偏差过大」还是
    「外参失效」。这个工具把四个环节的数据摊开，一眼看出断在哪。

用法（项目根目录）：
    run.bat tools\\live_check.py
    run.bat tools\\live_check.py --conf 0.15 --far-crop
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import CALIB_DIR, OUTPUT_DIR                                # noqa: E402
from src.loc.camera_model import CameraModel, Extrinsics, pixels_to_ground         # noqa: E402
from src.loc.matcher import MatchParams, ObservedMarker, locate                    # noqa: E402
from tools.realtime_field import build_field_map, dedup_dets                      # noqa: E402

CLASSES = ["LCross", "TCross", "XCross"]


def grab(index: int, n: int = 12):
    cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        return None
    best = None
    for _ in range(n):
        ok, fr = cap.read()
        if not ok or fr is None or fr.shape[1] != 1280:
            continue
        s = cv2.Laplacian(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
        if best is None or s > best[0]:
            best = (s, fr)
    cap.release()
    return None if best is None else best[1]


def detect(model, frame, conf, far_crop, far_ratio=0.34, far_scale=2.0):
    from ultralytics import YOLO  # noqa: F401
    dets = []

    def add(res, s, y_off=0):
        for xyxy, cf, cl in zip(res.boxes.xyxy.cpu().numpy(),
                                res.boxes.conf.cpu().numpy(),
                                res.boxes.cls.cpu().numpy()):
            name = model.names[int(cl)]
            if name not in CLASSES:
                continue
            box = xyxy / s
            box[[1, 3]] += y_off
            dets.append({"type": name, "conf": float(cf), "box": box,
                         "u": float((box[0] + box[2]) / 2),
                         "v": float((box[1] + box[3]) / 2)})

    add(model.predict(frame, conf=conf, imgsz=640, verbose=False)[0], 1.0)
    if far_crop:
        y2 = int(frame.shape[0] * far_ratio)
        big = cv2.resize(frame[:y2], None, fx=far_scale, fy=far_scale,
                         interpolation=cv2.INTER_CUBIC)
        add(model.predict(big, conf=conf, imgsz=640, verbose=False)[0], far_scale)
    return dedup_dets(dets, radius=35.0)


def main() -> int:
    ap = argparse.ArgumentParser(description="现场一键诊断")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--far-crop", action="store_true")
    ap.add_argument("--start-half", choices=["auto", "neg", "pos"], default="auto",
                    help="相机在场地哪一端：neg=x<0，pos=x>0，auto=不限制（会判 ambiguous）")
    args = ap.parse_args()

    frame = grab(args.index)
    if frame is None:
        print(f"打不开相机 index={args.index}")
        return 1
    sharp = cv2.Laplacian(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    print(f"帧 {frame.shape[1]}x{frame.shape[0]}  清晰度 {sharp:.0f}")

    cam = CameraModel.from_yaml(CALIB_DIR / "camera_intrinsics.yaml")
    ext = Extrinsics.from_yaml(CALIB_DIR / "camera_extrinsics.yaml")
    fm = build_field_map(ROOT / "config/field.yaml", ROOT / "config/field_geometry.yaml",
                         mirror=True, site_path=ROOT / "config/field_site.yaml")
    print(f"外参 h={ext.t[2]:.2f} m（t 应只含高度）")

    out_dir = OUTPUT_DIR / "live_check"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%H%M%S")

    from ultralytics import YOLO
    params = MatchParams()
    x_sign = {"auto": 0, "neg": -1, "pos": 1}[args.start_half]

    for wname in ("best_marker_v7.pt", "best_marker_v11.pt"):
        model = YOLO(str(ROOT / "weights" / wname))
        dets = detect(model, frame, args.conf, args.far_crop)
        print(f"\n=== {wname} @conf {args.conf}"
              f"{' + far-crop' if args.far_crop else ''} → {len(dets)} 个检测 ===")
        if not dets:
            print("  （没有检测）")
            continue

        uv = np.array([[d["u"], d["v"]] for d in dets], float)
        pts, okm = pixels_to_ground(cam, ext, uv, max_range=12.0)
        obs, obs_det = [], []
        for i, d in enumerate(dets):
            if not okm[i]:
                continue
            obs.append(ObservedMarker(type=d["type"], x=float(pts[i, 0]),
                                      y=float(pts[i, 1]), confidence=d["conf"]))
            obs_det.append(i)
        cnt = {t: sum(1 for o in obs if o.type == t) for t in CLASSES}
        print(f"  反投影成功 {len(obs)} 个｜类别 "
              + "，".join(f"{t}×{v}" for t, v in cnt.items()))

        result = locate(obs, fm, params, x_sign=x_sign)
        print(f"  定位：valid={result.valid} status={result.status} "
              f"markers={result.marker_count} "
              f"x={result.x:+.3f} y={result.y:+.3f} yaw={result.yaw_deg:+.2f}° "
              f"residual={result.residual:.3f} m")
        for k, m in enumerate(result.matches):
            print(f"    #{k + 1} {m['type']:7s} obs#{m['obs_index']} "
                  f"map({m['map_x']:+.2f},{m['map_y']:+.2f}) d={m['residual_m']:.2f} m")

        hit = {int(m["obs_index"]) for m in result.matches}
        vis = frame.copy()
        for oi, det_i in enumerate(obs_det):
            d = dets[det_i]
            used = oi in hit
            col = (80, 255, 120) if used else (110, 110, 110)
            cv2.rectangle(vis, (int(d["box"][0]), int(d["box"][1])),
                          (int(d["box"][2]), int(d["box"][3])), col, 2 if used else 1)
            cv2.putText(vis, d["type"][:1] + f"{d['conf']:.2f}",
                        (int(d["box"][0]), max(int(d["box"][1]) - 4, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        cv2.putText(vis, f"{wname}  valid={result.valid} status={result.status} "
                         f"markers={result.marker_count}",
                    (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (80, 255, 120) if result.valid else (0, 0, 255), 1)

        p = out_dir / f"{wname.split('_')[-1].split('.')[0]}_{stamp}.png"
        cv2.imwrite(str(p), vis)
        print(f"  标注图：{p.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
