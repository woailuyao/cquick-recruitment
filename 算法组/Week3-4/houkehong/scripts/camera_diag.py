# -*- coding: utf-8 -*-
"""
相机深入诊断：判断内置相机到底是"没画面"还是"画面被冻结"

对比连续帧的差异：
  * 每帧都不一样  -> 相机在工作，之前看到的灰图可能是某一帧的偶然
  * 帧与帧完全相同 -> 静态占位图，说明驱动/隐私层在拦截

同时尝试多种分辨率与后端，确认是分辨率不匹配还是整个设备被拦。
"""

import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "snapshots")
os.makedirs(OUT, exist_ok=True)

# MSMF 排最前：实测 DSHOW 只能 10 fps，MSMF 才能 30 fps
BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]
RESOLUTIONS = [(1280, 720), (640, 480), (320, 240)]


def test(index, backend_name, flag, w, h, nframes=8):
    cap = cv2.VideoCapture(index, flag)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)

    frames = []
    for _ in range(12):          # 丢弃前几帧等曝光稳定
        ok, f = cap.read()
        if not ok or f is None:
            break
        frames.append(f)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()

    if len(frames) < 2:
        return {"ok": False, "reason": "read_failed", "nego": (aw, ah)}

    t0 = time.perf_counter()
    # 相邻帧差异
    diffs = []
    for a, b in zip(frames[:-1], frames[1:]):
        if a.shape != b.shape:
            diffs.append(255.0)
            continue
        diffs.append(float(np.mean(np.abs(a.astype(np.int16) - b.astype(np.int16)))))
    last = frames[-1]
    return {
        "ok": True,
        "nego": (aw, ah),
        "shape": last.shape,
        "mean": float(np.mean(last)),
        "std": float(np.std(last)),
        "unique_colors": int(len(np.unique(last.reshape(-1, 3), axis=0))),
        "frame_diffs": [round(d, 4) for d in diffs],
        "max_diff": round(max(diffs), 4),
        "frozen": max(diffs) < 0.01,
        "sample": last,
    }


def main():
    print("=" * 78)
    print("相机深入诊断  cv2:", cv2.__version__)
    print("=" * 78)
    results = []
    for idx in range(3):
        for name, flag in BACKENDS:
            for w, h in RESOLUTIONS:
                r = test(idx, name, flag, w, h)
                if r is None:
                    continue
                tag = f"idx={idx} {name:5s} 请求{w}x{h}"
                if not r["ok"]:
                    print(f"{tag:34s} 打开但取帧失败 协商={r['nego']}")
                    continue
                print(f"{tag:34s} 协商={r['nego']} shape={r['shape']} "
                      f"mean={r['mean']:.1f} std={r['std']:.1f} "
                      f"颜色数={r['unique_colors']} 最大帧差={r['max_diff']}"
                      f"{'  << 画面冻结(静态占位图)' if r['frozen'] else ''}")
                path = os.path.join(OUT, f"diag_idx{idx}_{name}_{w}x{h}.jpg")
                cv2.imwrite(path, r["sample"])
                results.append((idx, name, w, h, r, path))
            break  # 同一后端测完所有分辨率就够了

    print("-" * 78)
    if not results:
        print("没有任何可用组合。")
        return 1

    live = [t for t in results if not t[4]["frozen"]]
    if live:
        print(f"[结论] 有 {len(live)} 个组合能出**动态画面**（相机真的在工作）：")
        for idx, name, w, h, r, path in live:
            print(f"   idx={idx} {name} {w}x{h}  -> {path}")
    else:
        print("[结论] 所有组合的画面都是**冻结的静态图**，说明相机没有真正输出视频流。")
        print("       这不是分辨率或后端问题，而是设备层被拦截（隐私/驱动/硬件开关）。")
        for idx, name, w, h, r, path in results[:1]:
            print(f"       参考快照: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
