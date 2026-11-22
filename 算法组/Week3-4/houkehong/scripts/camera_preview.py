# -*- coding: utf-8 -*-
"""
相机视角预览

两种用法：

    # 抓一张快照存到 snapshots/（不用弹窗，方便远程看）
    python scripts/camera_preview.py --camera 1 --snapshot

    # 打开实时预览窗口（按 q 或 ESC 退出，空格键存图）
    python scripts/camera_preview.py --camera 1 --live

    # 同时叠加上"估算的地面距离圈"（基于 config 里的假定外参，仅供判断取景，
    # 不是标定结果，不要拿来当测量值用）
    python scripts/camera_preview.py --camera 1 --live --ground

画面上的标记：
    白色十字   画面中心（= 相机光轴方向）
    三分线     构图参考
    绿色弧线   水平视场角
    地面距离圈 估算的 1/2/3/5 m 落点（依赖外参，见上面说明）
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
OUT = os.path.join(ROOT, "snapshots")


def open_cam(index, width, height):
    for name, flag in (("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)):
        cap = cv2.VideoCapture(index, flag)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 别让残留手动曝光把帧率拖死
        ok = False
        for _ in range(12):
            ok, f = cap.read()
            if ok:
                break
        if ok:
            print(f"[相机] index={index} backend={name}")
            return cap, name
        cap.release()
    return None, None


def measure_fps(cap, n=40):
    t0 = time.perf_counter()
    got = 0
    for _ in range(n):
        ok, _ = cap.read()
        if not ok:
            break
        got += 1
    dt = time.perf_counter() - t0
    return got / dt if dt else 0.0


def draw_overlay(img, fps, backend, index, draw_ground):
    h, w = img.shape[:2]
    out = img.copy()

    # 三分线
    for i in (1, 2):
        cv2.line(out, (w * i // 3, 0), (w * i // 3, h), (90, 90, 90), 1, cv2.LINE_AA)
        cv2.line(out, (0, h * i // 3), (w, h * i // 3), (90, 90, 90), 1, cv2.LINE_AA)

    # 中心十字 + 视场角标注
    cx, cy = w // 2, h // 2
    cv2.drawMarker(out, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 40, 2)
    cv2.putText(out, f"{w}x{h}", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(out, f"{w}x{h}", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (255, 255, 255), 2, cv2.LINE_AA)
    info = f"idx={index} {backend}  {fps:.1f} fps"
    cv2.putText(out, info, (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(out, info, (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 255), 2, cv2.LINE_AA)

    # 地面距离圈：用 config 里的假定外参推算，仅供取景判断
    if draw_ground:
        try:
            from src.config import load_extrinsics, load_camera

            cam = load_camera(os.path.join(ROOT, "config", "camera.yaml"))
            ex = load_extrinsics(os.path.join(ROOT, "config", "camera.yaml"))
            for d in (1.0, 2.0, 3.0, 5.0):
                # 载体系下的地面点 (d, 0, 0) -> 相机系 -> 像素
                p_b = np.array([[d, 0.0, 0.0]])
                p_c = (ex.R.T @ (p_b - ex.t).T).T
                if p_c[0, 2] <= 0:
                    continue
                u = cam.fx * p_c[0, 0] / p_c[0, 2] + cam.cx
                v = cam.fy * p_c[0, 1] / p_c[0, 2] + cam.cy
                if 0 <= v < h:
                    cv2.line(out, (0, int(v)), (w, int(v)), (60, 200, 60), 1, cv2.LINE_AA)
                    cv2.putText(out, f"{d:.0f} m", (w - 70, int(v) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 200, 60), 2, cv2.LINE_AA)
            hfov = 2 * math.degrees(math.atan(cam.cx / cam.fx))
            cv2.putText(out, f"hFOV~{hfov:.0f}deg (assumed intrinsics)",
                        (12, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (60, 200, 60), 2, cv2.LINE_AA)
        except Exception as e:
            cv2.putText(out, f"ground overlay skipped: {e}", (12, h - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=1)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--live", action="store_true", help="弹出实时窗口")
    ap.add_argument("--snapshot", action="store_true", help="只存一张图")
    ap.add_argument("--ground", action="store_true", help="叠加地面距离圈（依赖假定外参）")
    ap.add_argument("--out", default=os.path.join(OUT, "camera_view.jpg"))
    args = ap.parse_args()

    cap, backend = open_cam(args.camera, args.width, args.height)
    if cap is None:
        print(f"[错误] 打不开相机 index={args.camera}")
        return 1

    os.makedirs(OUT, exist_ok=True)

    if args.snapshot or not args.live:
        fps = measure_fps(cap)
        ok, frame = cap.read()
        if not ok:
            print("[错误] 取帧失败")
            cap.release()
            return 1
        vis = draw_overlay(frame, fps, backend, args.camera, args.ground)
        cv2.imwrite(args.out, vis)
        clean = os.path.splitext(args.out)[0] + "_clean.jpg"
        cv2.imwrite(clean, frame)
        print(f"[OK] 带标注: {args.out}")
        print(f"[OK] 原图  : {clean}")
        print(f"     分辨率 {frame.shape[1]}x{frame.shape[0]}  亮度 {np.mean(frame):.1f}  fps {fps:.1f}")
        cap.release()
        return 0

    print("实时预览中：空格存图，q 或 ESC 退出")
    fps = 0.0
    t_last = time.perf_counter()
    cnt = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            cnt += 1
            now = time.perf_counter()
            if now - t_last >= 0.5:
                fps = cnt / (now - t_last)
                cnt = 0
                t_last = now
            vis = draw_overlay(frame, fps, backend, args.camera, args.ground)
            cv2.imshow("camera preview", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == 32:
                p = os.path.join(OUT, time.strftime("shot_%H%M%S.jpg"))
                cv2.imwrite(p, frame)
                print("已保存", p)
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
