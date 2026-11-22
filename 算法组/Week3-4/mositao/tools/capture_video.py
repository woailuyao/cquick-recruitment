# -*- coding: utf-8 -*-
"""录像工具：按考核要求的分辨率与帧率采集视频片段。

为什么不用系统相机 App 直接录：
- 系统 App 可能改分辨率/帧率，而**内参是按 1280x720 标定的**，换分辨率即作废；
- 这里固定 1280x720、MSMF 后端、30 fps，并记录实测帧率供 README 使用；
- 同时把拍摄日期、设备索引、实测帧率写进同名 .txt，省得事后回忆。

用法（项目根目录）：
    # 录 70 秒到 data/videos/ball_s01.mp4
    python tools/capture_video.py --index 1 --seconds 70 --name ball_s01

    # 只预览不录（摆机位、调光线时用）
    python tools/capture_video.py --index 1 --preview

界面按键：q 提前结束并保存；s 立即开始录制（--seconds 0 时）
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import VIDEO_DIR, rel                       # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass

BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]
W, H, FPS = 1280, 720, 30


def open_camera(index: int):
    for name, api in BACKENDS:
        cap = cv2.VideoCapture(index, api)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        return cap, name
    return None, ""


def sharpness(gray: np.ndarray) -> float:
    h, w = gray.shape[:2]
    return float(cv2.Laplacian(gray[int(h * .2):int(h * .8), int(w * .2):int(w * .8)],
                               cv2.CV_64F).var())


def main() -> int:
    ap = argparse.ArgumentParser(description="按考核规格录制视频")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=70.0, help="录制秒数")
    ap.add_argument("--name", default="", help="输出文件名（不含扩展名）")
    ap.add_argument("--preview", action="store_true", help="只预览不录制")
    ap.add_argument("--out-dir", default=str(VIDEO_DIR))
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
        print(f"警告：分辨率不是 {W}x{H}，而内参是按 {W}x{H} 标定的——请先解决再录")

    name = args.name or f"clip_{datetime.now():%Y%m%d_%H%M%S}"
    path = out_dir / f"{name}.mp4"
    info_path = out_dir / f"{name}.txt"

    if args.preview:
        print("预览模式：q 退出")
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.putText(frame, f"PREVIEW  sharp={sharpness(g):.0f}", (14, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 220), 2)
            cv2.imshow("preview", frame)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
        cap.release()
        cv2.destroyAllWindows()
        return 0

    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), float(FPS), (got_w, got_h))
    if not writer.isOpened():
        print(f"无法创建视频文件：{rel(path)}")
        cap.release()
        return 1

    print(f"开始录制 {args.seconds:.0f} 秒 -> {rel(path)}（q 可提前结束）")
    t0 = time.perf_counter()
    frames = 0
    fps_marks = []
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("取帧失败，录制中止")
            break
        now = time.perf_counter()
        elapsed = now - t0
        if elapsed > args.seconds:
            break
        writer.write(frame)
        frames += 1
        if frames % 30 == 0:
            fps_marks.append(frames / max(elapsed, 1e-6))

        view = frame.copy()
        cv2.circle(view, (30, 30), 10, (0, 0, 235), -1)
        cv2.putText(view, f"REC {elapsed:5.1f}/{args.seconds:.0f}s  frames={frames}",
                    (52, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 235), 2)
        cv2.imshow("record", view)
        if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
            print("手动结束")
            break

    dt = time.perf_counter() - t0
    writer.release()
    cap.release()
    cv2.destroyAllWindows()

    actual_fps = frames / max(dt, 1e-6)
    size_mb = path.stat().st_size / 1e6 if path.exists() else 0.0
    summary = [
        f"文件      : {rel(path)}",
        f"时长      : {dt:.1f} s",
        f"帧数      : {frames}",
        f"实测帧率  : {actual_fps:.1f} fps（连续取帧法）",
        f"分辨率    : {got_w}x{got_h}",
        f"设备索引  : {args.index}（后端 {backend}）",
        f"文件大小  : {size_mb:.1f} MB",
        f"录制时间  : {datetime.now():%Y-%m-%d %H:%M:%S}",
    ]
    for line in summary:
        print("  " + line)
    info_path.write_text("\n".join(summary) + "\n", encoding="utf-8")
    print(f"参数记录  : {rel(info_path)}")
    if frames < args.seconds * FPS * 0.6:
        print("提示：帧数偏少，可能掉帧。若多次出现，检查 USB 带宽或改用 --index 0 对比。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
