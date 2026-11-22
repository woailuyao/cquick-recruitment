# -*- coding: utf-8 -*-
"""实时取景 + 按键抓单帧，用于机位对位与外参标定取样。

为什么需要它：
    capture_video.py 只能录整段视频，framing_check.py 抓帧后立刻跑 YOLO、不能反复挑；
    而摆机位这件事本质是**看一眼、挪一点、再抓一张**，需要能连续抓多张候选图。
    尤其是外参标定：棋盘格平铺在地面，要挑一张角点齐全、清晰度最高、反光最小的图，
    录视频再抽帧会浪费现场时间。

用法（项目根目录）：
    # 摆机位时对位：开窗看画面，g 切网格，s 存图
    run.bat tools\\grab_frame.py

    # 标定棋盘格：存到 data/extr/
    run.bat tools\\grab_frame.py --out-dir data\\extr --tag board

界面按键：
    s / 空格   保存当前帧
    g          切换 100 px 网格（对位时量像素用）
    q / Esc    退出
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass

BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]
W, H = 1280, 720
GRID = 100


def open_camera(index: int):
    for name, api in BACKENDS:
        cap = cv2.VideoCapture(index, api)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        return cap, name
    return None, ""


def sharpness(gray: np.ndarray) -> float:
    h, w = gray.shape[:2]
    return float(cv2.Laplacian(gray[int(h * .2):int(h * .8), int(w * .2):int(w * .8)],
                               cv2.CV_64F).var())


def draw_grid(img: np.ndarray) -> None:
    h, w = img.shape[:2]
    for x in range(GRID, w, GRID):
        cv2.line(img, (x, 0), (x, h), (0, 190, 190), 1, cv2.LINE_AA)
        cv2.putText(img, str(x), (x + 3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 190, 190), 1)
    for y in range(GRID, h, GRID):
        cv2.line(img, (0, y), (w, y), (0, 190, 190), 1, cv2.LINE_AA)
        cv2.putText(img, str(y), (3, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 190, 190), 1)
    # 光轴（画面中心）十字，方便对上 layout_check 预测的横向位置
    cx, cy = w // 2, h // 2
    cv2.line(img, (cx - 18, cy), (cx + 18, cy), (0, 120, 255), 1, cv2.LINE_AA)
    cv2.line(img, (cx, cy - 18), (cx, cy + 18), (0, 120, 255), 1, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description="实时取景抓单帧")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--out-dir", default="data/extr")
    ap.add_argument("--tag", default="grab", help="文件名前缀")
    ap.add_argument("--no-window", action="store_true",
                    help="不开窗，直接存一张就退出（脚本化用）")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    if not out_dir.is_absolute():
        out_dir = ROOT / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    cap, backend = open_camera(args.index)
    if cap is None:
        print(f"打不开相机 index={args.index}")
        return 1
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"相机 index={args.index} 后端={backend} 实际 {got_w}x{got_h}")
    if (got_w, got_h) != (W, H):
        print(f"警告：分辨率不是 {W}x{H}，内参按 {W}x{H} 标定，存下来的图不能用于标定")

    if args.no_window:
        best = None
        for _ in range(10):
            ok, fr = cap.read()
            if not ok or fr is None:
                continue
            s = sharpness(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY))
            if best is None or s > best[0]:
                best = (s, fr)
        cap.release()
        if best is None:
            print("抓不到帧")
            return 1
        p = out_dir / f"{args.tag}_{datetime.now():%Y%m%d_%H%M%S}.png"
        cv2.imwrite(str(p), best[1])
        print(f"已保存 {p.relative_to(ROOT)}  清晰度 {best[0]:.0f}")
        return 0

    print("按键：s/空格 存图　g 网格　q/Esc 退出")
    show_grid = True
    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("取帧失败，退出")
            break
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        s = sharpness(g)
        view = frame.copy()
        if show_grid:
            draw_grid(view)
        cv2.putText(view, f"sharp={s:.0f}  grid={'on' if show_grid else 'off'}  saved={saved}",
                    (14, H - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 180), 2)
        cv2.imshow("grab", view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("g"):
            show_grid = not show_grid
        if key in (ord("s"), 32):
            p = out_dir / f"{args.tag}_{datetime.now():%Y%m%d_%H%M%S}.png"
            cv2.imwrite(str(p), frame)          # 存原图，不存叠加网格的视图
            saved += 1
            print(f"[{saved}] 已保存 {p.relative_to(ROOT)}  清晰度 {s:.0f}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"共保存 {saved} 张到 {out_dir.relative_to(ROOT) if out_dir.is_relative_to(ROOT) else out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
