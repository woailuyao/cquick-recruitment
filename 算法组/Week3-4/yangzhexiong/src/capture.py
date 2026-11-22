"""采集原始视频片段（任务 1 的“本人实验室实拍数据”）。

用法：
    python capture.py clip_01 --secs 30
    python capture.py clip_02 --secs 25 --note "球在滚动，实验室东侧"

按 q 可提前结束。视频以 MJPG 编码原样存盘，不重新编码，抽帧时不会二次失真。
每次采集都会往 data/raw/capture_log.csv 追加一行（设备、分辨率、实测帧率、日期），
写 README 时直接抄这张表。
"""
import argparse
import csv
import datetime
import os
import time

import cv2

import camera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="片段名，例如 clip_01")
    ap.add_argument("--secs", type=float, default=30, help="录制时长，秒")
    ap.add_argument("--out", default="data/raw")
    ap.add_argument("--note", default="", help="这段视频拍的是什么，会写进 log")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    cap, frame, info = camera.auto_open()
    real_fps = camera.measure_fps(cap, 20) or info["fps"] or 30.0

    h, w = frame.shape[:2]
    path = os.path.join(args.out, f"{args.name}.avi")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), real_fps, (w, h))
    if not writer.isOpened():
        raise RuntimeError(f"VideoWriter 打不开：{path}")

    print(f"录制 {args.name}.avi  {w}x{h} @ {real_fps} fps  时长约 {args.secs:.0f}s")
    print("按 q 提前结束；录制中尽量让球出现在画面不同位置/远近。")

    t0 = time.time()
    frames = 0
    interrupted = False
    last_print = t0
    while True:
        elapsed = time.time() - t0
        if elapsed >= args.secs:
            break
        ok, frame = cap.read()
        if not ok:
            print("取帧失败，已写入到目前为止的内容。")
            interrupted = True
            break
        writer.write(frame)
        frames += 1

        if not args.no_preview:
            canvas = frame.copy()
            left = max(0.0, args.secs - elapsed)
            cv2.putText(canvas, f"REC {elapsed:5.1f}s / {args.secs:.0f}s  REC LEFT {left:4.1f}s",
                        (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(canvas, f"{w}x{h} {real_fps}fps  frames={frames}",
                        (12, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("capture", canvas)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                interrupted = True
                break
        if time.time() - last_print > 5:
            last_print = time.time()
            print(f"  {elapsed:5.1f}s  已写 {frames} 帧")

    writer.release()
    cap.release()
    cv2.destroyAllWindows()

    dur = frames / real_fps if real_fps else 0
    print(f"完成：{path}  共 {frames} 帧，约 {dur:.1f}s"
          + ("（提前结束）" if interrupted else ""))

    log = os.path.join(args.out, "capture_log.csv")
    new = not os.path.exists(log)
    with open(log, "a", newline="", encoding="utf-8-sig") as f:
        wr = csv.writer(f)
        if new:
            wr.writerow(["clip", "date", "resolution", "fourcc", "measured_fps",
                         "frames", "seconds", "node", "note"])
        wr.writerow([args.name, datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                     f"{w}x{h}", info["fourcc"], real_fps, frames, f"{dur:.1f}",
                     "V4L2", args.note])
    print(f"已记录到 {log}")


if __name__ == "__main__":
    main()
