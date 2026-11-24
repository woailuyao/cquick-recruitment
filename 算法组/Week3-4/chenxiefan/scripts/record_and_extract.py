#!/usr/bin/env python3

import argparse
import os
import time

import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW_DIR = os.path.join(ROOT, "data", "raw")
FRAME_DIR = os.path.join(ROOT, "data", "frames")

def parse_args():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)

    rec = sub.add_parser("record")
    rec.add_argument("--device", default="0",
                     help="本地相机编号（0 = /dev/video0）或网络流 http://ip:port/video")
    rec.add_argument("--index", type=int, default=1, help="视频编号，例如 1 -> video1.mp4")
    rec.add_argument("--seconds", type=float, default=60.0)
    rec.add_argument("--width", type=int, default=1280)
    rec.add_argument("--height", type=int, default=720)
    rec.add_argument("--fps", type=float, default=30.0)
    rec.add_argument("--no-show", action="store_true", help="不弹预览窗口（SSH/无显示环境用）")

    ext = sub.add_parser("extract")
    ext.add_argument("--video", required=True)
    ext.add_argument("--every", type=int, default=10, help="每 N 帧抽一张")
    return ap.parse_args()

def record(args):
    os.makedirs(RAW_DIR, exist_ok=True)
    path = os.path.join(RAW_DIR, f"video{args.index}.mp4")

    if str(args.device).startswith("http"):
        cap = cv2.VideoCapture(str(args.device), cv2.CAP_FFMPEG)
    else:
        cap = cv2.VideoCapture(int(args.device), cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[错误] 打不开 {args.device}")
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
    print(f"录制到 {path}（{w}x{h}），按 q 结束")

    frames = 0
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        ok, frame = cap.read()
        if not ok:
            print("[错误] 取帧失败")
            break
        writer.write(frame)
        frames += 1
        if not args.no_show:
            cv2.imshow("record", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    writer.release()
    if not args.no_show:
        cv2.destroyAllWindows()
    print(f"完成：{frames} 帧，{frames / max(time.time() - t0, 1e-6):.1f} fps")
    return 0

def extract(args):
    name = os.path.splitext(os.path.basename(args.video))[0]
    out_dir = os.path.join(FRAME_DIR, name)
    os.makedirs(out_dir, exist_ok=True)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[错误] 打不开视频 {args.video}")
        return 1

    idx = 0
    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every == 0:
            cv2.imwrite(os.path.join(out_dir, f"{name}_{idx:06d}.jpg"), frame)
            saved += 1
        idx += 1

    cap.release()
    print(f"抽帧完成：{saved} 张，输出到 {out_dir}")
    return 0

def main():
    args = parse_args()
    return record(args) if args.mode == "record" else extract(args)

if __name__ == "__main__":
    raise SystemExit(main())
