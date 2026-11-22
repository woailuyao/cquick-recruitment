#!/usr/bin/env python3
"""测相机真实帧率：对比 MJPG 开/关，找出能跑满 30fps 的组合。

只测不改任何文件，跑完会告诉你该用哪个配置。

用法：
  python tools/fps_check.py --camera 1
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402


def fourcc_str(v) -> str:
    v = int(v or 0)
    if v <= 0:
        return "????"
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def measure(cap, seconds: float = 2.0):
    for _ in range(5):                     # 丢掉启动抖动
        cap.read()
    n = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        ok, _ = cap.read()
        if not ok:
            break
        n += 1
    return n / max(time.perf_counter() - t0, 1e-6), n


def try_combo(index, w, h, fps, fourcc, backend_name, backend):
    if backend is None:
        cap = cv2.VideoCapture(index)
    else:
        cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        cap.release()
        return None
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    gw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    gh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real, n = measure(cap, 2.0)
    cc = fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))
    cap.release()
    return {"backend": backend_name, "fourcc_set": fourcc or "不变",
            "fourcc_actual": cc, "size": f"{gw}x{gh}", "fps": real, "n": n}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="1")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()

    if not str(args.camera).isdigit():
        print("[error] 只支持数字设备号")
        return 2
    idx = int(args.camera)

    backends = []
    if os.name == "nt":
        if hasattr(cv2, "CAP_DSHOW"):
            backends.append(("DSHOW", cv2.CAP_DSHOW))
        if hasattr(cv2, "CAP_MSMF"):
            backends.append(("MSMF", cv2.CAP_MSMF))
    else:
        if hasattr(cv2, "CAP_V4L2"):
            backends.append(("V4L2", cv2.CAP_V4L2))
    backends.append(("默认", None))

    print(f"=== 测量 设备 {idx}  {args.width}x{args.height}  请求 {args.fps}fps ===\n")
    results = []
    for name, api in backends:
        for fourcc in ("MJPG", None):
            r = try_combo(idx, args.width, args.height, args.fps, fourcc, name, api)
            if r is None:
                print(f"  {name:6s} fourcc={fourcc or '不变':5s} -> 打不开")
                continue
            print(f"  {name:6s} fourcc设={fourcc or '不变':5s} "
                  f"实际fourcc={r['fourcc_actual']:5s} {r['size']:9s} "
                  f"-> {r['fps']:5.1f} fps ({r['n']} 帧)")
            results.append(r)

    if not results:
        print("\n[error] 全部组合都打不开。检查 USB、驱动、是否被别的程序占用")
        return 3

    best = max(results, key=lambda r: r["fps"])
    print(f"\n=== 最佳组合 ===")
    print(f"  backend={best['backend']}  设 fourcc={best['fourcc_set']}  "
          f"实际={best['fourcc_actual']}  {best['size']}  {best['fps']:.1f} fps")

    if best["fps"] >= 25:
        print("  [OK] 能跑满 30fps 档位，录制和演示都够用")
    elif best["fps"] >= 12:
        print("  [warn] 只有一半帧率。718p 下这是很多 UVC 相机的上限；\n"
              "         如果 MJPG 那一行明显比另一行高，就用 MJPG（本工程已默认开启）。\n"
              "         录屏时如实记录实测帧率即可，不影响评分")
    else:
        print("  [warn] 帧率很低，优先换 USB 3.0 蓝口、别用扩展坞、光线调亮")

    print(f"\n  分辨率必须保持 {args.width}x{args.height}，不要为了帧率改分辨率——"
          f"内参是按这个分辨率标的，改了就要重新标定")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
