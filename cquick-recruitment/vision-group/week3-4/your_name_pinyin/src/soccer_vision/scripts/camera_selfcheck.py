#!/usr/bin/env python3
"""相机自检：验证设备节点、分辨率、连续取帧、实际帧率。

用法：
    python camera_selfcheck.py --device /dev/video0 --width 1280 --height 720

考核要求（细则 2.2）：
  - 能否打开设备；
  - 实际输出分辨率是否为 1280x720；
  - 能否连续取帧；
  - 实际采集帧率是否接近 30fps（用连续取帧实际耗时统计，不能用 CAP_PROP_FPS）。
"""

import argparse
import time

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--frames", type=int, default=150)
    args = ap.parse_args()

    print(f"[1] 打开设备 {args.device} ...")
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print("[FAIL] 无法打开设备，请用 `v4l2-ctl --list-devices` 确认节点")
        return 1
    print("     打开成功")

    # 强制 MJPG 编码（否则部分摄像头默认 YUYV 只能到 10fps，MJPG 可达 30fps）
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[2] 实际分辨率: {real_w}x{real_h}")
    if (real_w, real_h) != (args.width, args.height):
        print(f"[WARN] 与目标 {args.width}x{args.height} 不一致")

    print(f"[3] 连续取帧 {args.frames} 帧，统计实际帧率 ...")
    t0 = time.time()
    count = 0
    while count < args.frames:
        ok, _ = cap.read()
        if not ok:
            print("[FAIL] 取帧失败")
            return 1
        count += 1
    dt = time.time() - t0
    fps = count / dt
    print(f"     实际帧率: {fps:.2f} fps（耗时 {dt:.2f}s / {count} 帧）")
    print(f"     标称 CAP_PROP_FPS: {cap.get(cv2.CAP_PROP_FPS):.2f}（仅供参考，非真实帧率）")

    cap.release()
    print("[OK] 相机自检完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
