"""相机深度诊断：定位"能开但帧率极低"的原因。

按顺序验证四件事：
  1. 单帧耗时分布（是匀速慢，还是前几帧慢后面正常）
  2. 画面亮度（是否黑帧 / 是否暗光导致长曝光）
  3. 像素格式（MJPG 压缩流 vs YUY2 原始流）
  4. 手动曝光 / 分辨率组合

用法：
  python tools/diag_camera.py --index 2
  python tools/diag_camera.py --index 2 --save-dir logs
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def read_timed(cap, n):
    """连续读 n 帧，返回每帧耗时（秒）与最后一帧。"""
    times, frame = [], None
    for _ in range(n):
        t = time.time()
        ok, f = cap.read()
        times.append(time.time() - t)
        if ok:
            frame = f
    return times, frame


def report(tag, times, frame):
    if not times:
        print(f"  {tag}: 无数据")
        return
    fps = len(times) / sum(times) if sum(times) > 0 else 0
    print(f"  {tag}: 平均 {fps:.1f} fps | 单帧 min {min(times)*1000:.0f}ms "
          f"max {max(times)*1000:.0f}ms")
    if frame is not None:
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print(f"        画面 {frame.shape[1]}x{frame.shape[0]} 亮度 mean={g.mean():.1f} "
              f"max={g.max()} {'<-- 疑似黑帧' if g.max() < 10 else ''}")


def open_cam(index, api, w, h, fourcc=None):
    cap = cv2.VideoCapture(index, api)
    if not cap.isOpened():
        return None
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=2)
    ap.add_argument("--save-dir", default="logs")
    args = ap.parse_args()

    api = cv2.CAP_DSHOW
    print(f"诊断 index={args.index} 后端=DSHOW\n")

    # --- 1. 默认打开，看单帧耗时分布与亮度 ---
    cap = open_cam(args.index, api, 1280, 720)
    if cap is None:
        raise SystemExit(f"打不开 index={args.index}")
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    print(f"[默认] 协商 {aw}x{ah} | fourcc={''.join(chr((fourcc>>8*i)&0xFF) for i in range(4))} "
          f"| 报告FPS={cap.get(cv2.CAP_PROP_FPS):.1f}")
    times, frame = read_timed(cap, 8)
    report("前 8 帧", times, frame)
    if frame is not None:
        Path(ROOT / args.save_dir).mkdir(parents=True, exist_ok=True)
        p = ROOT / args.save_dir / f"diag_default_{args.index}.jpg"
        cv2.imwrite(str(p), frame)
        print(f"        样例图 -> {p.relative_to(ROOT)}")

    # --- 2. 尝试手动曝光（DSHOW: 负数表示手动，越小曝光越短）---
    print("\n[手动曝光]")
    for val in (-6, -5, -4):
        ok = cap.set(cv2.CAP_PROP_EXPOSURE, val)
        times, frame = read_timed(cap, 8)
        fps = len(times) / sum(times) if sum(times) > 0 else 0
        print(f"  exposure={val} set={ok} -> {fps:.1f} fps", end="")
        if frame is not None:
            print(f" 亮度={cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean():.1f}")
        else:
            print()
    cap.release()

    # --- 3. MJPG 格式 + 各分辨率 ---
    print("\n[MJPG + 分辨率]")
    for (w, h) in [(640, 480), (1280, 720), (1920, 1080)]:
        cap = open_cam(args.index, api, w, h, fourcc="MJPG")
        if cap is None:
            print(f"  {w}x{h}: 打不开")
            continue
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fc = int(cap.get(cv2.CAP_PROP_FOURCC))
        times, frame = read_timed(cap, 20)
        fps = len(times) / sum(times) if sum(times) > 0 else 0
        tag = "".join(chr((fc >> 8 * i) & 0xFF) for i in range(4))
        bright = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean() if frame is not None else -1
        print(f"  请求 {w}x{h} -> 实际 {aw}x{ah} [{tag}] {fps:.1f} fps 亮度 {bright:.1f}")
        cap.release()

    # --- 4. YUY2 原始流对照 ---
    print("\n[YUY2 原始流]")
    cap = open_cam(args.index, api, 1280, 720, fourcc="YUY2")
    if cap is not None:
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        times, _ = read_timed(cap, 20)
        fps = len(times) / sum(times) if sum(times) > 0 else 0
        print(f"  1280x720 -> 实际 {aw}x{ah} {fps:.1f} fps")
        cap.release()


if __name__ == "__main__":
    main()
