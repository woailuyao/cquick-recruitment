# -*- coding: utf-8 -*-
"""抓一帧干净的相机原图，用于离线诊断检测器、手点标志像素。

不推理、不画框、不拼接，输出就是相机原始画面（默认 1280x720）。

    python grab_frame.py --source 1 --out D:\\vision_data\\diag\\frame.jpg

用途：
  1) 拿这一帧跑 yolo detect predict conf=0.01，看模型到底能不能看见标志；
  2) 如果检测器实在不行，用这一帧手点 4 个标志的像素坐标做 PnP。
"""

import argparse
import os
import time

import cv2


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="1", help="摄像头索引")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--warmup", type=int, default=25,
                    help="丢弃前 N 帧，等自动曝光/白平衡稳定")
    ap.add_argument("--out", default=os.path.join("snapshots", "frame.jpg"))
    return ap.parse_args()


def open_cam(idx, w, h):
    """按 MSMF->MSMF->DSHOW 的顺序试，只有真能读到帧才接受。"""
    for name, api, set_res in (("MSMF", cv2.CAP_MSMF, True),
                               ("MSMF", cv2.CAP_MSMF, False),
                               ("DSHOW", cv2.CAP_DSHOW, True)):
        cap = cv2.VideoCapture(idx, api)
        if not cap.isOpened():
            cap.release()
            time.sleep(0.8)
            continue
        if set_res:
            try:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            except Exception:
                cap.release()
                time.sleep(0.8)
                continue
        ok_n = 0
        for _ in range(12):
            try:
                ok, _ = cap.read()
            except Exception:
                # OpenCV 5 流启动失败时会抛 Mat 断言，而不是返回 False
                ok = False
            ok_n += 1 if ok else 0
        if ok_n < 6:
            cap.release()
            time.sleep(0.8)
            continue
        gw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        gh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"采集后端 {name}  实际分辨率 {gw}x{gh}")
        return cap
    raise SystemExit(f"摄像头 {idx} 打不开或读不到帧。换个索引试试（1 -> 0），"
                     "并确认没有别的程序占用摄像头。")


def main():
    args = parse_args()
    if args.source.isdigit():
        cap = open_cam(int(args.source), args.width, args.height)
    else:
        if not os.path.exists(args.source):
            raise SystemExit(f"找不到输入: {args.source}")
        cap = cv2.VideoCapture(args.source)
        if not cap.isOpened():
            raise SystemExit(f"打不开输入源: {args.source}")

    frame = None
    for _ in range(max(1, args.warmup)):
        try:
            ok, f = cap.read()
        except Exception:
            ok, f = False, None
        if ok:
            frame = f
    cap.release()

    if frame is None:
        raise SystemExit("读帧失败，没拿到任何一帧")

    h, w = frame.shape[:2]
    print(f"帧尺寸 {w}x{h}")
    if (w, h) != (args.width, args.height):
        print(f"警告: 不是 {args.width}x{args.height}，内参不匹配，定位会偏")

    out = os.path.abspath(args.out)
    d = os.path.dirname(out)
    if d:
        os.makedirs(d, exist_ok=True)
    cv2.imwrite(out, frame)
    print("已保存", out)


if __name__ == "__main__":
    main()
