# -*- coding: utf-8 -*-
"""
相机接入自检（对应考核细则 2.2）

作用：
  1. 枚举当前可用的摄像头设备号，逐个尝试 DirectShow / MSMF 后端能否真正打开；
  2. 强制请求 1280x720 / 30fps，并读回相机实际协商到的分辨率与 CAP_PROP_FPS；
  3. 按细则要求用"连续取帧实际耗时"测量真实采集帧率（CAP_PROP_FPS 不可信）；
  4. 检查画面是否全黑 / 全白（防止打开了一个空设备）；
  5. 存一张快照到 snapshots/ 便于人工确认拍的到底是哪个相机。

用法（Windows）：
  D:\\vision_ws\\k1_soccer_vision\\.venv\\Scripts\\python.exe scripts\\camera_check.py
  ...\\python.exe scripts\\camera_check.py --max-index 5 --frames 90 --save
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

# 顺序很重要：**MSMF 必须排在 DSHOW 前面**。
# 本机实测：同一台相机用 DSHOW 只能跑到 ~10 fps，换 MSMF 立刻 30 fps，
# 而且换 MJPG / 改分辨率 / 改请求帧率都救不了 DSHOW。
BACKENDS = [
    ("MSMF", getattr(cv2, "CAP_MSMF", None)),
    ("ANY", cv2.CAP_ANY),
    ("DSHOW", getattr(cv2, "CAP_DSHOW", None)),
]


def probe(index: int, backend_name: str, backend_flag: int, frames: int):
    """尝试打开 (index, backend)，成功返回结果字典。"""
    if backend_flag is None:
        return None
    cap = cv2.VideoCapture(index, backend_flag)
    if not cap.isOpened():
        cap.release()
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    # 只在个别后端支持时生效，失败也不影响
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_reported = float(cap.get(cv2.CAP_PROP_FPS))

    # 连续取帧测真实帧率：丢掉前几帧等自动曝光稳定
    for _ in range(5):
        cap.read()
    t0 = time.perf_counter()
    got = 0
    last = None
    for _ in range(frames):
        ok, f = cap.read()
        if not ok or f is None:
            break
        last = f
        got += 1
    dt = time.perf_counter() - t0
    measured_fps = got / dt if dt > 0 else 0.0

    mean_brightness = float(np.mean(last)) if last is not None else -1.0
    result = {
        "index": index,
        "backend": backend_name,
        "width": w,
        "height": h,
        "fps_reported": fps_reported,
        "fps_measured": measured_fps,
        "frames_read": got,
        "brightness": mean_brightness,
        "frame": last,
    }
    cap.release()
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-index", type=int, default=4, help="扫描 0..N-1 号设备")
    ap.add_argument("--frames", type=int, default=60, help="测帧率时读取的帧数")
    ap.add_argument("--save", action="store_true", help="保存快照到 snapshots/")
    ap.add_argument("--only", type=int, default=None, help="只测试指定设备号")
    args = ap.parse_args()

    print("=" * 72)
    print("cv2:", cv2.__version__)
    print("=" * 72)

    indices = [args.only] if args.only is not None else range(args.max_index + 1)
    found = []
    for idx in indices:
        for name, flag in BACKENDS:
            r = probe(idx, name, flag, args.frames)
            if r is None:
                continue
            print(
                f"[OK] index={idx} backend={name:5s} "
                f"{r['width']}x{r['height']} "
                f"reported_fps={r['fps_reported']:.1f} "
                f"measured_fps={r['fps_measured']:.1f} "
                f"frames={r['frames_read']} brightness={r['brightness']:.1f}"
            )
            found.append(r)

            if args.save and r["frame"] is not None:
                outdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "snapshots")
                os.makedirs(outdir, exist_ok=True)
                path = os.path.join(outdir, f"cam{idx}_{name}.jpg")
                cv2.imwrite(path, r["frame"])
                print(f"      snapshot -> {path}")
            break  # 同一 index 用一个能成功的后端就够

    print("-" * 72)
    if not found:
        print("[FAIL] 没有任何设备能被打开。")
        print("       排查：设备是否插好 / 是否被其它程序占用 / 隐私权限是否允许桌面应用访问相机。")
        return 1

    print(f"[DONE] 可用设备 {len(found)} 个")
    for r in found:
        warn = []
        if (r["width"], r["height"]) != (1280, 720):
            warn.append("分辨率不是 1280x720")
        if r["fps_measured"] < 25:
            warn.append("实测帧率明显低于 30fps")
        if r["brightness"] < 5:
            warn.append("画面几乎全黑")
        if warn:
            print(f"  index={r['index']} 注意: " + "; ".join(warn))
    return 0


if __name__ == "__main__":
    sys.exit(main())
