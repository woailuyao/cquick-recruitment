#!/usr/bin/env python3
"""检查相机能不能用、真实分辨率是多少、真实帧率是多少。

这个脚本不需要标定文件，拿来确认"相机到底通不通"。

用法：
  python tools/check_camera.py              # 默认 /dev/video0
  python tools/check_camera.py --camera 2   # 换成 /dev/video2
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                          # noqa: E402
import numpy as np                  # noqa: E402


def v4l2_list() -> None:
    print("---- v4l2-ctl --list-devices ----")
    try:
        out = subprocess.run(["v4l2-ctl", "--list-devices"],
                             capture_output=True, text=True, timeout=5)
        print(out.stdout.strip() or out.stderr.strip())
    except FileNotFoundError:
        print("(没装 v4l2-ctl，可先 sudo apt install v4l-utils；不影响下面的检测)")
    except Exception as e:
        print("(v4l2-ctl 调用失败:", e, ")")


def fourcc_str(v: float) -> str:
    v = int(v)
    if v <= 0:
        return "????"
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def try_open(index: int, w: int, h: int, fps: int, mjpg: bool):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None
    if mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def measure_fps(cap, n: int = 120):
    """真实帧率：连续取 n 帧计时。"""
    for _ in range(5):                     # 丢掉前几帧，避免启动抖动
        cap.read()
    t0 = time.perf_counter()
    got = 0
    last = None
    for _ in range(n):
        ok, frame = cap.read()
        if not ok:
            break
        got += 1
        last = frame
    dt = time.perf_counter() - t0
    return (got / dt if dt > 0 else 0.0), got, last


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--frames", type=int, default=120)
    args = ap.parse_args()

    if not str(args.camera).isdigit():
        print("[error] 本脚本只支持 /dev/videoN 形式的数字索引")
        return 2
    index = int(args.camera)

    v4l2_list()
    print(f"\n---- 尝试打开 /dev/video{index} ----")

    results = []
    for mjpg in (True, False):
        cap = try_open(index, args.width, args.height, args.fps, mjpg)
        if cap is None:
            print(f"  MJPG={mjpg}: 打不开")
            continue
        got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_f = cap.get(cv2.CAP_PROP_FPS)
        got_cc = fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))
        real_fps, n, frame = measure_fps(cap, args.frames)
        cap.release()
        print(f"  MJPG={mjpg}: 报告 {got_w}x{got_h} fourcc={got_cc} "
              f"CAP_PROP_FPS={got_f:.1f} | 实测 {real_fps:.1f} fps ({n} 帧)")
        results.append((mjpg, got_w, got_h, got_cc, real_fps, frame))

    if not results:
        print("\n[error] 完全打不开相机。常见原因：\n"
              "  1) 设备号不对 -> ls /dev/video*\n"
              "  2) 没有权限 -> sudo usermod -aG video $USER 然后重新登录\n"
              "  3) 被别的程序占用 -> 关掉 cheese / 其他 python 进程")
        return 3

    best = max(results, key=lambda r: (r[4], r[1]))
    mjpg, w, h, cc, fps, frame = best

    print("\n---- 结论 ----")
    print(f"  推荐配置 : {'MJPG' if mjpg else 'YUYV(默认)'} {w}x{h} 实测 {fps:.1f} fps")
    if (w, h) != (args.width, args.height):
        print(f"  [WARN] 实际分辨率 {w}x{h} != 请求的 {args.width}x{args.height}，"
              "标定和推理都必须用实际分辨率")
    if fps < 25:
        print(f"  [WARN] 实测只有 {fps:.1f} fps。若已是 720p：\n"
              "     - 确认用的是 MJPG 而不是 YUYV\n"
              "     - 换 USB 3.0 口（蓝色），不要用 hub\n"
              "     - 相机可能本身最高只到该帧率")
    else:
        print("  [OK] 帧率达标")

    if frame is not None:
        os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
        path = os.path.join(ROOT, "logs", "camera_check.jpg")
        cv2.imwrite(path, frame)
        print(f"  已保存一帧到 {path}")

    # 有显示器就预览 3 秒
    if os.environ.get("DISPLAY"):
        cap = try_open(index, w, h, args.fps, mjpg)
        if cap is not None:
            print("  预览 3 秒（按 q 提前退出）...")
            t_end = time.time() + 3.0
            while time.time() < t_end:
                ok, f = cap.read()
                if not ok:
                    break
                cv2.imshow("camera_check (press q)", f)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            cap.release()
            cv2.destroyAllWindows()
    _ = np
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
