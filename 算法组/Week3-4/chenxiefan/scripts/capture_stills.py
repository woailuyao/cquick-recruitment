#!/usr/bin/env python3

import argparse
import os
import time

import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="http://192.168.56.1:8080/video")
    ap.add_argument("--out", default=os.path.join(ROOT, "data", "calib"))
    ap.add_argument("--prefix", default="still")
    ap.add_argument("--warmup", type=int, default=10, help="先丢掉多少帧")
    return ap.parse_args()

def main():
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)

    if str(args.source).startswith("http"):
        cap = cv2.VideoCapture(args.source, cv2.CAP_FFMPEG)
    else:
        cap = cv2.VideoCapture(int(args.source), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    if not cap.isOpened():
        print(f"[错误] 打不开 {args.source}")
        return 1

    for _ in range(args.warmup):
        cap.read()

    existing = [f for f in os.listdir(args.out) if f.startswith(args.prefix) and f.endswith(".jpg")]
    idx = len(existing)
    print(f"输出目录：{args.out}（已有 {idx} 张 {args.prefix}_*.jpg）")
    print("空格=存一张，q/ESC=退出")

    saved = 0
    flash_until = 0.0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[错误] 取帧失败")
            break

        view = frame.copy()
        cv2.putText(view, f"saved={saved}  SPACE=save  q=quit", (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if time.time() < flash_until:
            cv2.putText(view, "SAVED", (12, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

        cv2.imshow("capture (SPACE=save, q=quit)", view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            idx += 1
            path = os.path.join(args.out, f"{args.prefix}_{idx:03d}.jpg")
            cv2.imwrite(path, frame)
            saved += 1
            flash_until = time.time() + 0.4
            print(f"已保存 {path}")
        elif key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"本次保存 {saved} 张")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
