"""USB 端口 / 摄像头链路体检：逐个摄像头测实际分辨率与实际帧率。

用途：把相机分别插到笔记本不同 USB 口，各跑一次，对比 logs/port_test.csv，
      就能判断哪个口跑得动 1280x720。

用法：
  # 列出所有能打开的摄像头
  python tools/usb_port_test.py --list

  # 测左侧口（标签写 left）
  python tools/usb_port_test.py --port left --seconds 6

  # 测右侧口
  python tools/usb_port_test.py --port right --seconds 6
"""
import argparse
import csv
import datetime as dt
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
LOG_CSV = ROOT / "logs" / "port_test.csv"
FIELDS = ["时间", "端口标签", "index", "后端", "协商分辨率", "报告FPS",
          "实测FPS", "取帧失败数", "结论"]


def pick_backend(name):
    if name == "auto":
        return cv2.CAP_DSHOW if sys.platform.startswith("win") else getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)
    return {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF,
            "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)}[name]


def backend_name(api):
    return {cv2.CAP_DSHOW: "DSHOW", cv2.CAP_MSMF: "MSMF",
            getattr(cv2, "CAP_V4L2", -1): "V4L2"}.get(api, str(api))


def probe(index, api, width, height, seconds):
    cap = cv2.VideoCapture(index, api)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    rep_fps = cap.get(cv2.CAP_PROP_FPS)

    # 预热：丢掉前 10 帧，避开自动曝光/驱动初始化
    for _ in range(10):
        cap.read()

    frames = fails = 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        ok, frame = cap.read()
        if not ok:
            fails += 1
            if fails > 30:
                break
            time.sleep(0.01)
            continue
        frames += 1
    elapsed = time.time() - t0
    cap.release()

    fps = frames / elapsed if elapsed > 0 else 0.0
    if fps >= 25:
        verdict = "OK 可跑 720p30"
    elif fps >= 15:
        verdict = "偏低 可能带宽/供电不足"
    elif fps >= 5:
        verdict = "差 不建议用于验收"
    else:
        verdict = "失败 几乎无帧"
    if fails > 5:
        verdict += f"（取帧失败{fails}次）"

    return dict(aw=aw, ah=ah, rep_fps=rep_fps, fps=fps, fails=fails, verdict=verdict)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--port", default="", help="端口标签，如 left / right / rear")
    ap.add_argument("--max-index", type=int, default=6)
    ap.add_argument("--backend", default="auto", choices=["auto", "dshow", "msmf", "v4l2"])
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    api = pick_backend(args.backend)
    print(f"后端 {backend_name(api)} | 目标 {args.width}x{args.height}@30 | 每口测 {args.seconds:.0f}s\n")

    if args.list:
        print("可打开的摄像头：")
        for i in range(args.max_index):
            r = probe(i, api, args.width, args.height, 1.0)
            if r:
                print(f"  index={i}  协商 {r['aw']}x{r['ah']}  报告FPS={r['rep_fps']:.1f}  "
                      f"1秒实测 {r['fps']:.1f}")
        return

    LOG_CSV.parent.mkdir(parents=True, exist_ok=True)
    new_file = not LOG_CSV.exists()
    rows = []
    print(f"{'index':>5} {'协商分辨率':>12} {'报告FPS':>8} {'实测FPS':>8}  结论")
    for i in range(args.max_index):
        r = probe(i, api, args.width, args.height, args.seconds)
        if not r:
            continue
        print(f"{i:>5} {r['aw']:>7}x{r['ah']:<4} {r['rep_fps']:>8.1f} {r['fps']:>8.1f}  {r['verdict']}")
        rows.append([dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), args.port or "-",
                     i, backend_name(api), f"{r['aw']}x{r['ah']}",
                     f"{r['rep_fps']:.1f}", f"{r['fps']:.1f}", r["fails"], r["verdict"]])

    if not rows:
        sys.exit("\n没有能打开的摄像头：检查线是否插稳、换一个 USB 口再试")

    with open(LOG_CSV, "a", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(FIELDS)
        w.writerows(rows)
    print(f"\n已追加到 {LOG_CSV.relative_to(ROOT)}（端口标签：{args.port or '未填'}）")
    print("提示：换一个 USB 口再跑一次，对比实测FPS 即可判断端口能力。")


if __name__ == "__main__":
    main()
