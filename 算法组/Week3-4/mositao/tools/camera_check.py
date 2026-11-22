# -*- coding: utf-8 -*-
"""相机接入自检：枚举设备、核对分辨率/帧率、实测采集帧率、存一帧样张。

对应细则 2.2「相机接入与自检」与 1.8「录屏需展示设备识别过程」：
- 能否成功打开设备；
- 实际输出分辨率是否为 1280x720；
- 能否连续取帧；
- 实际采集帧率（用连续取帧耗时测，不用 CAP_PROP_FPS 的返回值）；
- 断开/占用设备时给出明确错误。

运行（项目根目录）：
    python tools/camera_check.py                # 枚举 0..3 号设备
    python tools/camera_check.py --index 1      # 只测指定设备
    python tools/camera_check.py --save-video 5 # 顺带录 5 秒样例视频
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

from src.common.paths import OUTPUT_DIR, VIDEO_DIR, ensure_runtime_dirs, rel   # noqa: E402

TARGET_W, TARGET_H = 1280, 720

# 后端顺序很关键：MSMF 常能走 MJPG/共享内存通道拿到真正的 720p@30，
# 而 DSHOW 在 YUY2 下会被 USB2 带宽卡到 ~10 fps。ANY 只作最后兜底。
BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]

try:      # 抑制 OpenCV 的 WARN/ERROR 噪声，让报告干净
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                            # noqa: BLE001
    pass


def fourcc_str(value: int) -> str:
    try:
        v = int(value)
        s = "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))
        return s if s.isprintable() else f"0x{v:08x}"
    except Exception:                                        # noqa: BLE001
        return "?"


def open_camera(index: int):
    """依次尝试后端，返回 (cap, 后端名)。"""
    for name, api in BACKENDS:
        cap = cv2.VideoCapture(index, api)
        if cap.isOpened():
            return cap, name
        cap.release()
    return None, ""


def _measure(cap, frames: int):
    """连续取帧测实际帧率与「帧新鲜度」，返回 (fps, 首帧, 成功帧数, 新鲜帧占比)。

    新鲜度是必要的：某些后端会以极快速度返回同一缓冲区的重复帧，
    表面上 30 fps，实际画面根本没更新。这里用相邻帧平均差判定。
    """
    for _ in range(5):          # 预热：自动曝光/白平衡收敛
        cap.read()
    n, t0, first, prev = 0, None, None, None
    diffs = []
    for _ in range(frames):
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        if first is None:
            first = frame
            t0 = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        if prev is not None:
            diffs.append(float(np.abs(gray - prev).mean()))
        prev = gray
        n += 1
    fps = 0.0
    if t0 is not None and n > 1:
        dt = time.perf_counter() - t0
        fps = (n - 1) / dt if dt > 0 else 0.0
    # 真实重复帧是逐像素几乎完全相同（差值≈0）；静止场景的正常帧仍有传感器噪声。
    # 因此用「差值 < 0.02 视为重复」而不是「变化是否够大」，避免把静止画面误判成假帧率。
    dup = sum(1 for d in diffs if d < 0.02)
    fresh = (1.0 - dup / len(diffs)) if diffs else 0.0
    return fps, first, n, fresh


def probe_index(index: int, frames: int = 30, force_fourcc: str | None = None,
                lock_focus: bool = False) -> dict:
    info = {"index": index, "opened": False, "backend": "", "width": 0, "height": 0,
            "fps_reported": 0.0, "fps_actual": 0.0, "fourcc": "", "fourcc_retry": "",
            "fps_retry": 0.0, "sharpness": float("nan"), "autofocus": None,
            "focus_locked": None, "error": "", "frame_path": "", "frame_path_retry": ""}

    cap, backend = open_camera(index)
    if cap is None:
        info["error"] = "无法打开设备（未插上 / 被其他程序占用 / 无此索引）"
        return info

    info["opened"] = True
    info["backend"] = backend
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if force_fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*force_fourcc))

    info["width"] = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    info["height"] = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    info["fps_reported"] = float(cap.get(cv2.CAP_PROP_FPS))
    info["fourcc"] = fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))

    # 自动对焦状态（-1 表示设备不支持该项）
    try:
        info["autofocus"] = float(cap.get(cv2.CAP_PROP_AUTOFOCUS))
    except Exception:                                        # noqa: BLE001
        info["autofocus"] = None

    fps, first, _, fresh = _measure(cap, frames)
    info["fps_actual"] = fps
    info["fresh_ratio"] = fresh

    # 帧率偏低时换 MJPG：UVC 在 YUY2 下常被 USB 带宽卡到 10 fps
    if fps < 25.0 and not force_fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)
        cap.set(cv2.CAP_PROP_FPS, 30)
        info["fourcc_retry"] = fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))
        fps2, first2, _, fresh2 = _measure(cap, frames)
        info["fps_retry"] = fps2
        if fps2 > fps:
            fps, first, fresh = fps2, (first2 if first2 is not None else first), fresh2
            info["fps_actual"], info["fresh_ratio"] = fps, fresh

    out_dir = OUTPUT_DIR / "camera_probe"
    out_dir.mkdir(parents=True, exist_ok=True)
    if first is not None:
        p = out_dir / f"cam{index}_{datetime.now():%H%M%S}.png"
        cv2.imwrite(str(p), first)
        info["frame_path"] = rel(p)
        gray = cv2.cvtColor(first, cv2.COLOR_BGR2GRAY)
        info["mean_brightness"] = float(np.mean(gray))
        info["sharpness"] = float(cv2.Laplacian(gray, cv2.CV_64F).var())

    # 固定对焦：细则要求关闭自动对焦。能设置就设，设不进去如实记录
    if lock_focus:
        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            time.sleep(0.3)
            af_after = float(cap.get(cv2.CAP_PROP_AUTOFOCUS))
            focus_val = float(cap.get(cv2.CAP_PROP_FOCUS))
            info["focus_locked"] = {"autofocus_after": af_after,
                                    "focus_value": focus_val,
                                    "supported": af_after == 0.0 or focus_val >= 0}
            if first is not None:      # 锁焦后再抓一张，便于对比清晰度
                _, f2, _, _ = _measure(cap, max(frames // 2, 10))
                if f2 is not None:
                    p2 = out_dir / f"cam{index}_locked_{datetime.now():%H%M%S}.png"
                    cv2.imwrite(str(p2), f2)
                    info["frame_path_locked"] = rel(p2)
                    info["sharpness_locked"] = float(
                        cv2.Laplacian(cv2.cvtColor(f2, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
        except Exception as exc:                             # noqa: BLE001
            info["focus_locked"] = {"error": str(exc)[:80]}

    cap.release()
    return info


def record_video(index: int, seconds: float) -> str:
    cap, _ = open_camera(index)
    if cap is None:
        return ""
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    VIDEO_DIR.mkdir(parents=True, exist_ok=True)
    out = VIDEO_DIR / f"camera_sample_{datetime.now():%Y%m%d_%H%M%S}.mp4"
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (w, h))
    t0 = time.perf_counter()
    n = 0
    while time.perf_counter() - t0 < seconds:
        ok, frame = cap.read()
        if not ok:
            continue
        vw.write(frame)
        n += 1
    vw.release()
    cap.release()
    print(f"样例视频   : {rel(out)}（{n} 帧，约 {n / max(seconds, 1e-6):.1f} fps）")
    return rel(out)


def judge(info: dict) -> tuple[bool, list[str]]:
    """按细则 2.1/2.2 判定单台相机是否可用于本次考核。"""
    reasons = []
    if (info.get("width"), info.get("height")) != (TARGET_W, TARGET_H):
        reasons.append(f"分辨率 {info.get('width')}x{info.get('height')} 非 1280x720")
    if info.get("fps_actual", 0.0) < 25.0:
        reasons.append(f"实测帧率 {info.get('fps_actual', 0.0):.1f} fps 低于 25")
    if info.get("fresh_ratio", 0.0) < 0.8:
        reasons.append("帧新鲜度不足，疑似返回重复帧")
    return (not reasons), reasons


def main() -> int:
    ap = argparse.ArgumentParser(description="相机接入自检")
    ap.add_argument("--index", type=int, default=None, help="只测指定设备索引")
    ap.add_argument("--max-index", type=int, default=4, help="枚举上限（不含）")
    ap.add_argument("--frames", type=int, default=30, help="每台设备测帧数")
    ap.add_argument("--save-video", type=float, default=0.0, help="另录 N 秒样例视频")
    ap.add_argument("--fourcc", default=None, help="强制像素格式，如 MJPG / YUY2")
    ap.add_argument("--lock-focus", action="store_true",
                    help="尝试关闭自动对焦（细则 2.1 要求固定对焦）")
    args = ap.parse_args()

    ensure_runtime_dirs()
    print("=" * 76)
    print("相机接入自检")
    print(f"时间: {datetime.now():%Y-%m-%d %H:%M:%S}   OpenCV {cv2.__version__}")
    print(f"目标: {TARGET_W}x{TARGET_H} @ 30 fps（细则 2.1 统一要求）")
    print("=" * 76)

    indices = [args.index] if args.index is not None else list(range(args.max_index))
    found = []
    for i in indices:
        info = probe_index(i, args.frames, force_fourcc=args.fourcc, lock_focus=args.lock_focus)
        if not info["opened"]:
            print(f"[{i}] 不可用：{info['error']}")
            continue
        found.append(i)
        res_ok = (info["width"], info["height"]) == (TARGET_W, TARGET_H)
        print(f"[{i}] 已打开（后端 {info['backend']}，像素格式 {info['fourcc']}）")
        print(f"     分辨率      : {info['width']}x{info['height']} "
              f"{'✓ 符合 1280x720' if res_ok else '✗ 与 1280x720 不符，需在 README 说明替代分辨率'}")
        print(f"     帧率        : 驱动标称 {info['fps_reported']:.1f} fps，"
              f"实测 {info['fps_actual']:.1f} fps（连续取帧法）")
        print(f"     帧新鲜度    : {info.get('fresh_ratio', 0):.0%}"
              f"（逐像素几乎相同的帧对比例越低越好；过高=后端返回重复帧，帧率是假的）")
        if info.get("fourcc_retry"):
            print(f"     换 MJPG 后  : 格式 {info['fourcc_retry']}，实测 {info['fps_retry']:.1f} fps")
        print(f"     清晰度      : 拉普拉斯方差 {info.get('sharpness', float('nan')):.1f}"
              f"（<100 一般意味着失焦或纯色画面）")
        _s = info.get("sharpness", 0.0) or 0.0
        print(f"     画质判定    : "
              + ("达标（可进入内参标定）" if _s >= 150
                 else "未达标——细则不管画质，但标定与地面投影必须要清晰，先用 tools/focus_tune.py 对焦"))
        print(f"     样张        : {info['frame_path']}")
        print(f"     画面亮度均值: {info.get('mean_brightness', float('nan')):.1f} "
              f"（过低=镜头盖/遮挡，过高=过曝）")
        if info.get("autofocus") is not None:
            af = info["autofocus"]
            print(f"     自动对焦    : {'开启' if af > 0.5 else '已关闭'}（CAP_PROP_AUTOFOCUS={af:.0f}）")
        # 打开成功但取不到帧，是最常见的真实故障（独占占用 / 驱动异常 / 后端不匹配）
        if info["fps_actual"] < 1.0:
            print("     状态        : 打开成功但取不到帧 -> 可能被独占占用 / 驱动异常 / 换后端再试")
        verdict, reasons = judge(info)
        print(f"     细则判定    : {'通过' if verdict else '需处理'}"
              + (f"（{'; '.join(reasons)}）" if reasons else "（720p、帧率、帧新鲜度均达标）"))
        if info.get("focus_locked"):
            fl = info["focus_locked"]
            if "error" in fl:
                print(f"     锁焦        : 失败 {fl['error']}")
            else:
                print(f"     锁焦        : 自动对焦读回 {fl['autofocus_after']:.0f}，"
                      f"focus 值 {fl['focus_value']:.0f}"
                      f"{'（设备不支持锁焦，需手动拧到固定位置并保持不动）' if not fl['supported'] else ''}")
                if "sharpness_locked" in info:
                    print(f"     锁焦后清晰度: {info['sharpness_locked']:.1f}，"
                          f"样张 {info.get('frame_path_locked', '')}")
        print("-" * 76)

    if not found:
        print()
        print("结论：没有可用相机。请依次排查：")
        print("  1. 相机是否插在 Windows 上（设备管理器 -> 照相机 应能看到）；")
        print("  2. 是否被其他程序占用（浏览器/会议软件/OBS 会独占 UVC 设备）；")
        print("  3. 若在 WSL 里跑，先确认 USB 是否已转发（WSL2 默认不转发，见 tools/README）")
        print("=" * 76)
        return 1

    if args.save_video > 0:
        record_video(found[0], args.save_video)

    print()
    print(f"结论：找到 {len(found)} 台可用相机，索引 {found}。")
    print("建议把上面这些数值（设备索引、分辨率、实测帧率）记进 README。")
    print("=" * 76)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
