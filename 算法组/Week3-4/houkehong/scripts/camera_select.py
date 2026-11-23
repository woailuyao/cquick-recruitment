# -*- coding: utf-8 -*-
"""
相机设备号选择与固定

为什么需要这个工具：
    Windows 下 OpenCV 的设备号（0/1/2...）**不保证稳定**。
    插拔 USB、换口、或者别的程序占用，都可能让外接相机从 idx=1 变成 idx=0。
    而 OpenCV 不提供"按设备名打开"的接口，所以要人工确认一次，
    把结果写进 config/camera.yaml，之后 run.py 默认就用这个号。

用法：

    # 1) 扫描所有设备，每个存一张带编号的快照，看哪个是外接相机
    python scripts/camera_select.py --max-index 3

    # 2) 确认后用外接相机的编号固定下来（会写进 config/camera.yaml）
    python scripts/camera_select.py --set 1

    # 3) 只看不写
    python scripts/camera_select.py --max-index 3 --no-save
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(ROOT, "config", "camera.yaml")
OUT = os.path.join(ROOT, "snapshots")


def try_open(index: int):
    """优先 MSMF（实测 DSHOW 只有 10 fps），失败再退回默认后端。"""
    for name, flag in (("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)):
        cap = cv2.VideoCapture(index, flag)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 别让残留的手动曝光把帧率拖死
        ok = False
        frame = None
        for _ in range(12):
            ok, frame = cap.read()
            if ok and frame is not None:
                break
        if not ok or frame is None:
            cap.release()
            continue
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return cap, name, aw, ah, frame
    return None, None, 0, 0, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-index", type=int, default=3)
    ap.add_argument("--set", type=int, default=None, help="把该编号写入 config/camera.yaml")
    ap.add_argument("--no-save", action="store_true")
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    print("=" * 70)
    print("扫描相机设备（MSMF 优先）")
    print("=" * 70)
    print(f"{'idx':>4} {'backend':<7} {'分辨率':>10} {'亮度':>7}  快照")
    print("-" * 70)

    found = []
    for idx in range(args.max_index + 1):
        cap, name, aw, ah, frame = try_open(idx)
        if cap is None:
            continue
        mean = float(np.mean(frame))
        path = ""
        if not args.no_save:
            vis = frame.copy()
            tag = f"idx={idx}  {name}  {aw}x{ah}"
            cv2.rectangle(vis, (0, 0), (vis.shape[1], 46), (0, 0, 0), -1)
            cv2.putText(vis, tag, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 255), 2, cv2.LINE_AA)
            path = os.path.join(OUT, f"camera_idx{idx}.jpg")
            cv2.imwrite(path, vis)
        print(f"{idx:>4} {name:<7} {f'{aw}x{ah}':>10} {mean:>7.1f}  {path}")
        found.append(idx)
        cap.release()

    print("-" * 70)
    if not found:
        print("没有找到任何可用相机。")
        print("  排查：USB 是否插好 / 别的程序是否占用 / Windows 隐私设置是否允许桌面应用访问相机。")
        return 1

    print(f"可用设备号: {found}")
    if len(found) > 1:
        print("多个设备时，请打开 snapshots/camera_idx*.jpg 确认哪个是外接相机。")
        print("选好后运行：  python scripts/camera_select.py --set <编号>")
    else:
        print(f"只有一个设备，建议固定为：  python scripts/camera_select.py --set {found[0]}")

    if args.set is not None:
        if args.set not in found:
            print(f"\n[警告] 你指定的 {args.set} 不在本次可用列表 {found} 里，仍然写入。")
        if not os.path.isfile(CONFIG):
            print(f"[错误] 找不到 {CONFIG}")
            return 2
        import yaml
        with open(CONFIG, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        cam = data.setdefault("camera", {})
        cam["device_index"] = int(args.set)
        with open(CONFIG, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        print(f"\n已写入 {CONFIG}: camera.device_index = {args.set}")
        print("之后 run.py 不带 --camera 就会默认用这个号。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
