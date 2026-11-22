#!/usr/bin/env python3

import argparse
import time

import cv2

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="0",
                    help="本地相机填编号（0 = /dev/video0）；也可以填网络流地址 http://ip:port/video")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--seconds", type=float, default=5.0)
    return ap.parse_args()

def main():
    args = parse_args()
    if str(args.device).startswith("http"):
        cap = cv2.VideoCapture(str(args.device), cv2.CAP_FFMPEG)
    else:
        cap = cv2.VideoCapture(int(args.device), cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[错误] 打不开 {args.device}，检查设备节点/网络流是否可用")
        return 1

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"设备      : {args.device if str(args.device).startswith('http') else '/dev/video' + str(args.device)}")
    print(f"分辨率    : {w}x{h}（目标 {args.width}x{args.height}）")
    print(f"驱动报帧率: {cap.get(cv2.CAP_PROP_FPS):.1f}（不一定等于真实帧率）")

    frames = 0
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[错误] 取帧失败，可能被其他程序占用或设备断开")
            break
        frames += 1

    dt = time.time() - t0
    if frames > 0:
        print(f"实测帧率  : {frames / dt:.1f} fps（{frames} 帧 / {dt:.2f} 秒）")
    cap.release()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
