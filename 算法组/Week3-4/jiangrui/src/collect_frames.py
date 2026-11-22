# -*- coding: utf-8 -*-
"""补拍训练数据：连续抽帧保存，给标注和重训用。

为什么要补拍：
    现有 marker 模型只见过“近距离、同一机位”的画面，换到 66 cm / 13.3°
    或更远的机位就基本检不出。这类外观域差只能靠补充数据解决，调参无法弥补。

用法：
    python collect_frames.py --source 1 --out D:\\vision_data\\marker_dataset2\\images\\raw

    按键：
      s  立刻存一帧
      a  开始/停止自动抽帧（默认每 0.5 s 一帧）
      q  退出

建议的拍摄路线（每个机位走一遍，边走边开自动抽帧）：
    1) 相机高度 0.6 m / 0.9 m / 1.2 m 各一遍
    2) 每个高度下，离标志 1.5 m / 2.5 m / 4 m / 5.5 m 各停留几秒
    3) 正对标志、斜 30°、斜 60° 三个朝向
    4) 顺带拍几段“画面里没有标志”的空场，抑制误检
"""

import argparse
import os
import time

import cv2


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="1")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--out", default=r"D:\vision_data\marker_dataset2\images\raw")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="自动抽帧间隔，秒")
    return ap.parse_args()


def open_cam(idx, w, h):
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
                ok = False
            ok_n += 1 if ok else 0
        if ok_n < 6:
            cap.release()
            time.sleep(0.8)
            continue
        print(f"采集后端 {name}  "
              f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
              f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        return cap
    raise SystemExit(f"摄像头 {idx} 打不开或读不到帧")


def main():
    args = parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)

    if args.source.isdigit():
        cap = open_cam(int(args.source), args.width, args.height)
    else:
        cap = cv2.VideoCapture(args.source)
        if not cap.isOpened():
            raise SystemExit(f"打不开输入: {args.source}")

    print(f"保存到 {out}")
    print("s 存一帧   a 自动抽帧   q 退出")
    print("建议每 5 帧挑 1 帧标注即可，相邻帧几乎一样")

    auto = False
    last_save = 0.0
    saved = 0
    t0 = time.time()
    prev_t = t0
    fps = 0.0

    while True:
        try:
            ok, frame = cap.read()
        except Exception:
            ok = False
        if not ok:
            print("读帧失败，退出")
            break

        now = time.time()
        if auto and now - last_save >= args.interval:
            p = os.path.join(out, time.strftime("mk_%Y%m%d_%H%M%S_") + f"{saved:04d}.jpg")
            cv2.imwrite(p, frame)
            saved += 1
            last_save = now

        fps = 0.9 * fps + 0.1 * (1.0 / max(1e-3, now - prev_t))
        prev_t = now

        hud = frame.copy()
        cv2.putText(hud, f"saved {saved}   auto {'ON' if auto else 'off'}"
                         f"   {frame.shape[1]}x{frame.shape[0]}   {fps:.0f} fps",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if auto else (200, 200, 200), 2)
        cv2.putText(hud, "s save   a auto   q quit", (12, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        cv2.imshow("collect", hud)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("s"):
            p = os.path.join(out, time.strftime("mk_%Y%m%d_%H%M%S_") + f"{saved:04d}.jpg")
            cv2.imwrite(p, frame)
            saved += 1
            print("saved", p)
        if k == ord("a"):
            auto = not auto
            print("auto", "ON" if auto else "off")

    cap.release()
    cv2.destroyAllWindows()
    print(f"共保存 {saved} 帧，用时 {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()