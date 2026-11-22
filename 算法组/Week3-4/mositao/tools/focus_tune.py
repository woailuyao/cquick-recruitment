# -*- coding: utf-8 -*-
"""对焦调校：实时显示画面清晰度，边拧镜头边找峰值。

背景：配发的 USB 相机是纯定焦模组（CAP_PROP_AUTOFOCUS 读回 -1），软件无法调焦，
只能物理旋转镜头。但「拧到怎样才算合焦」肉眼不可靠——用拉普拉斯方差当刻度量，
一边拧一边看数字爬升，最直观。

用法：
    # 图形界面（推荐）：把相机对着 3~10 m 外有纹理的目标，慢慢拧镜头，
    # 盯着「清晰度」与「历史最好」两个数字，爬到峰值后按 s 存一张存证
    python tools/focus_tune.py --index 1

    # 无窗口自检：只打印当前清晰度，适合确认「有没有得救」
    python tools/focus_tune.py --index 1 --probe --frames 60

判读标准（1280x720，中心 60% 区域）：
    < 50     严重失焦，几乎不可用
    50~150   偏软，标定误差会明显变大
    150~400  可用
    > 400    良好
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, rel                       # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass

BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]
ROI_RATIO = 0.6          # 只用画面中心 60% 算清晰度，避免边缘暗角干扰


def open_camera(index: int):
    for name, api in BACKENDS:
        cap = cv2.VideoCapture(index, api)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 30)
            return cap, name
        cap.release()
    return None, ""


def sharpness(gray: np.ndarray) -> float:
    """中心 ROI 的拉普拉斯方差。"""
    h, w = gray.shape[:2]
    ch, cw = int(h * ROI_RATIO), int(w * ROI_RATIO)
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    roi = gray[y0:y0 + ch, x0:x0 + cw]
    return float(cv2.Laplacian(roi, cv2.CV_64F).var())


def grade(value: float) -> tuple[str, tuple[int, int, int]]:
    if value < 50:
        return "严重失焦", (0, 0, 235)
    if value < 150:
        return "偏软", (0, 140, 255)
    if value < 400:
        return "可用", (0, 200, 0)
    return "良好", (0, 200, 0)


def probe(index: int, frames: int) -> int:
    cap, backend = open_camera(index)
    if cap is None:
        print(f"打不开相机 index={index}")
        return 1
    vals = []
    for _ in range(frames):
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        vals.append(sharpness(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
    cap.release()
    if not vals:
        print("取不到帧")
        return 1
    v = np.array(vals)
    tag, _ = grade(float(np.median(v)))
    print(f"后端 {backend}｜{len(vals)} 帧｜中位 {np.median(v):.1f}｜最大 {v.max():.1f}｜"
          f"最小 {v.min():.1f}｜判读 {tag}")
    if np.median(v) < 150:
        print("建议：把相机对准 3~10 m 外有纹理的目标（场地线、墙角、球网），再慢慢拧镜头观察数值爬升")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="对焦调校")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--probe", action="store_true", help="无窗口，只打印清晰度")
    ap.add_argument("--frames", type=int, default=60, help="--probe 时的帧数")
    args = ap.parse_args()

    if args.probe:
        return probe(args.index, args.frames)

    cap, backend = open_camera(args.index)
    if cap is None:
        print(f"打不开相机 index={args.index}")
        return 1

    win = "focus tune - rotate the lens until the number peaks"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1000, 620)

    history: deque[float] = deque(maxlen=180)
    best = 0.0
    best_frame = None
    out_dir = OUTPUT_DIR / "focus_tune"
    out_dir.mkdir(parents=True, exist_ok=True)

    print("把相机对准 3~10 m 外有纹理的目标，慢慢拧镜头；按 s 存下当前最好的一帧，q 退出")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("取帧失败")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        val = sharpness(gray)
        history.append(val)
        if val > best:
            best = val
            best_frame = frame.copy()

        tag, color = grade(val)
        view = frame.copy()
        cv2.putText(view, f"sharpness {val:7.1f}  [{tag}]", (16, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3)
        cv2.putText(view, f"best {best:7.1f}", (16, 92),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (230, 230, 230), 2)
        cv2.putText(view, "s = save best frame   q = quit", (16, view.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # 右侧画一条最近 180 帧的曲线，方便看趋势
        x0, x1 = view.shape[1] - 200, view.shape[1] - 20
        y0, y1 = 20, 160
        cv2.rectangle(view, (x0, y0), (x1, y1), (60, 60, 60), 1)
        if len(history) > 1:
            scale = max(max(history), 1.0)
            pts = []
            for i, v in enumerate(history):
                px = int(x0 + (x1 - x0) * i / (len(history) - 1))
                py = int(y1 - (y1 - y0) * min(v / scale, 1.0))
                pts.append((px, py))
            cv2.polylines(view, [np.array(pts, np.int32)], False, (0, 220, 220), 2)

        cv2.imshow(win, view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s") and best_frame is not None:
            p = out_dir / f"best_{best:.0f}_{datetime.now():%H%M%S}.png"
            cv2.imwrite(str(p), best_frame)
            print(f"已保存当前最佳帧：{rel(p)}（清晰度 {best:.1f}）")

    cap.release()
    cv2.destroyAllWindows()

    print("-" * 60)
    print(f"本次最佳清晰度：{best:.1f}（{grade(best)[0]}）")
    if best < 150:
        print("仍未达标。请依次检查：")
        print("  1. 镜头是否有保护膜未撕；")
        print("  2. 镜头筒是否有可旋转的对焦环（多数 UVC 模组外侧可拧）；")
        print("  3. 目标是否太近或太远——定焦模组的景深有限；")
        print("  4. 环境光是否太暗（暗光下会自动增益，画面发糊）。")
    else:
        print("对焦已达标，可以开始打印棋盘格做内参标定：")
        print("  python tools/capture_calib.py --index 1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
