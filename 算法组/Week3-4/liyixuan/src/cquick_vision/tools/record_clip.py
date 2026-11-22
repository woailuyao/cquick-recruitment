#!/usr/bin/env python3
"""录相机视频：手动开始/停止，按实测帧率写文件（不会出现二倍速）。

操作：
  空格 = 开始 / 停止录制
  q    = 退出（退出前会校验文件）
  r    = 重录（覆盖当前文件）

用法：
  python tools\\record_clip.py --camera 1 --out C:\\...\\raw\\ball_v01.avi
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402

from cvlib.camera import open_capture  # noqa: E402


def measure_fps(cap, seconds: float = 2.0) -> float:
    """先空跑几秒，量出相机真实的出帧速度。"""
    n = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        ok, _ = cap.read()
        if not ok:
            break
        n += 1
    dt = max(time.perf_counter() - t0, 1e-6)
    return n / dt


def probe_cameras() -> None:
    print("\n[probe] 正在测试设备号 0~4 ...")
    works = []
    for i in range(5):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                works.append((i, frame.shape))
        cap.release()
    if works:
        for i, shape in works:
            print(f"  设备 {i} 可用，分辨率 {shape[1]}x{shape[0]}")
        print(f"\n  把 --camera 改成 {works[0][0]} 再跑一次")
    else:
        print("  0~4 全都打不开。检查 USB 是否插紧、有没有被别的程序占用")


def hud(frame, lines, color=(0, 255, 0)):
    vis = frame.copy()
    y = 30
    for txt in lines:
        cv2.putText(vis, txt, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(vis, txt, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    color, 2, cv2.LINE_AA)
        y += 30
    return vis


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="1")
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--max-seconds", type=float, default=0.0,
                    help="安全上限，0 表示不限（默认不限，靠你手动停）")
    args = ap.parse_args()

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if os.path.exists(out):
        print(f"[warn] 已存在同名文件，录制后会覆盖：{out}")

    cap = open_capture(args.camera, args.width, args.height)
    if cap is None:
        print(f"[error] 打不开相机 {args.camera}")
        probe_cameras()
        return 2

    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    driver_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
    print(f"[rec] 相机 {args.camera} 实际 {got_w}x{got_h}，先测 2 秒真实帧率 ...")
    measured = measure_fps(cap, 2.0)
    print(f"[rec] 预热实测 {measured:.1f} fps，驱动报告 {driver_fps:.1f} fps")

    # 预热那两秒镜头刚打开、自动曝光还在收敛，测出来往往偏低。
    # 只要实测不低于驱动值的 75%，就用驱动值写文件，播放速度才准。
    fps = measured
    if 5.0 <= driver_fps <= 120.0 and measured >= 0.75 * driver_fps:
        fps = driver_fps
        print(f"[rec] 采用驱动帧率 {fps:.1f} fps 写文件（这样播放速度正确）")
    else:
        print(f"[rec] 采用实测帧率 {fps:.1f} fps 写文件")
    if fps < 20:
        print("[warn] 帧率偏低（<20）。换 USB 3.0 蓝口、别用扩展坞、把灯打开")
    fps = max(5.0, min(60.0, round(fps, 2)))

    win = "record_clip  |  SPACE=start/stop   q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 540)

    writer = None
    recording = False
    n_frames = 0
    t_start = 0.0
    total_saved = 0
    last_saved_frames = 0

    print("\n>>> 先调整好画面（球的位置、相机角度），然后按空格开始录制 <<<")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[error] 读帧失败，相机可能被拔了")
            break

        if recording:
            writer.write(frame)
            n_frames += 1
            el = time.time() - t_start
            vis = hud(frame, [
                f"REC  {el:5.1f}s   frames={n_frames}   {fps:.1f}fps",
                "SPACE = stop",
            ], (0, 0, 255))
            cv2.circle(vis, (got_w - 40, 40), 12, (0, 0, 255), -1)
            if args.max_seconds and el >= args.max_seconds:
                recording = False
                writer.release()
                writer = None
                last_saved_frames = n_frames
                total_saved = n_frames
                print(f"[rec] 到上限 {args.max_seconds:.0f}s，自动停止")
        else:
            if total_saved:
                vis = hud(frame, [
                    f"SAVED  {total_saved} frames  ({out})",
                    "SPACE = record again (overwrite)   q = quit",
                ], (0, 255, 255))
            else:
                vis = hud(frame, [
                    f"READY   {got_w}x{got_h}  {fps:.1f}fps",
                    "SPACE = start recording   q = quit",
                ], (0, 255, 0))

        cv2.imshow(win, vis)
        key = cv2.waitKey(1) & 0xFF

        if key == 32:                       # 空格
            if not recording:
                writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"XVID"),
                                         fps, (got_w, got_h))
                if not writer.isOpened():
                    print("[error] 创建视频文件失败，检查输出路径")
                    break
                recording = True
                n_frames = 0
                total_saved = 0
                t_start = time.time()
                print(f"[rec] 开始录制 -> {out}（再按空格停止）")
            else:
                recording = False
                writer.release()
                writer = None
                total_saved = n_frames
                el = time.time() - t_start
                print(f"[rec] 停止：{n_frames} 帧 / {el:.1f} 秒 "
                      f"= {n_frames/max(el,1e-6):.1f} fps")
        elif key == ord("r"):
            if recording:
                recording = False
                writer.release()
                writer = None
            total_saved = 0
            n_frames = 0
            print("[rec] 已清空，可以重录")
        elif key == ord("q"):
            break

    if recording and writer is not None:
        writer.release()
        total_saved = n_frames
        print(f"[rec] 退出时仍在录制，已保存 {n_frames} 帧")
    cap.release()
    cv2.destroyAllWindows()
    _ = last_saved_frames

    if total_saved < 20:
        print(f"[warn] 只保存了 {total_saved} 帧，这段基本没用，重录一次")
        return 5

    # 闭环校验：重新读一遍刚写的文件
    chk = cv2.VideoCapture(out)
    ok = chk.isOpened()
    n_chk = int(chk.get(cv2.CAP_PROP_FRAME_COUNT))
    f_chk = chk.get(cv2.CAP_PROP_FPS)
    w_chk = int(chk.get(cv2.CAP_PROP_FRAME_WIDTH))
    h_chk = int(chk.get(cv2.CAP_PROP_FRAME_HEIGHT))
    chk.release()
    size_mb = os.path.getsize(out) / 1e6 if os.path.exists(out) else 0
    print(f"[rec] 校验：能读={ok} {w_chk}x{h_chk} 帧数={n_chk} "
          f"文件帧率={f_chk:.1f} 时长={n_chk/max(f_chk,1e-6):.1f}s 大小={size_mb:.1f}MB")
    print("[rec] OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
