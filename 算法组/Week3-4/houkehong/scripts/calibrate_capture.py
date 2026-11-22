# -*- coding: utf-8 -*-
"""
标定图采集（细则 2.5 要求 >=15 张有效图，覆盖中心/四角/不同倾斜/不同距离）

为什么不能随便拍 20 张：连续按空格拍出来的是几乎相同的朝正面图，
畸变系数（尤其 k1/k2）根本约束不住，标出来的内参看着有值其实不可用。
本工具会自动判断"这张和已保存的够不够不一样"，重复构图会提示你换个角度。

用法：

    # 交互采集（推荐）
    python scripts/calibrate_capture.py --camera 1 --pattern 9x6

    # 无人值守自动采集（每检测到棋盘格且构图够新就存一张）
    python scripts/calibrate_capture.py --camera 1 --pattern 9x6 --auto --target 20

按键：
    SPACE  手动保存当前帧（绿色框表示已检测到棋盘格）
    a      切换自动采集
    d      删除最后一张（拍废了用）
    q/ESC  退出

采集完成后：
    python scripts/calibrate_camera.py --images calib_images --pattern 9x6 --square 18
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_DIR = os.path.join(ROOT, "calib_images")


def open_cam(index, width, height):
    for name, flag in (("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)):
        cap = cv2.VideoCapture(index, flag)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        ok = False
        for _ in range(12):
            ok, f = cap.read()
            if ok:
                break
        if ok:
            return cap, name
        cap.release()
    return None, None


def detect(gray, cols, rows):
    """返回 (ok, corners)。优先 SB 版本（更鲁棒、自带亚像素）。"""
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, c = cv2.findChessboardCornersSB(
            gray, (cols, rows),
            getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0) | getattr(cv2, "CALIB_CB_ACCURACY", 0))
        if ok:
            return True, c.astype(np.float32)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        return False, None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    return True, cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), crit)


def signature(corners, w, h):
    """
    构图特征：(归一化中心x, 归一化中心y, 归一化对角尺寸)。

    注意：NumPy 2.x 已删除 ndarray.ptp() 方法，必须用 np.ptp(arr)，
    否则会报 'numpy.ndarray' object has no attribute 'ptp'。
    """
    pts = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    cx, cy = pts[:, 0].mean() / w, pts[:, 1].mean() / h
    size = math.hypot(float(np.ptp(pts[:, 0])),
                      float(np.ptp(pts[:, 1]))) / math.hypot(w, h)
    return (float(cx), float(cy), float(size))


def is_novel(sig, existing, pos_tol=0.06, size_tol=0.10):
    """和已有构图比，中心或尺寸差异够大才算新样本。"""
    for e in existing:
        if (abs(sig[0] - e[0]) < pos_tol and abs(sig[1] - e[1]) < pos_tol
                and abs(sig[2] - e[2]) < size_tol):
            return False
    return True


def coverage_hint(sigs):
    """根据已采集的构图给出覆盖度提示。"""
    if not sigs:
        return "还没有样本"
    x = np.array([s[0] for s in sigs])
    y = np.array([s[1] for s in sigs])
    z = np.array([s[2] for s in sigs])
    tips = []
    if x.min() > 0.30: tips.append("左侧缺")
    if x.max() < 0.70: tips.append("右侧缺")
    if y.min() > 0.30: tips.append("上侧缺")
    if y.max() < 0.70: tips.append("下侧缺")
    if float(np.ptp(z)) < 0.15: tips.append("距离/大小太单一")
    return "覆盖 OK" if not tips else "还缺：" + "、".join(tips)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--pattern", default="9x6")
    ap.add_argument("--square", type=float, default=20.0,
                    help="方格边长（毫米），只用于最后打印标定命令，务必和你实际板子一致")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--out", default=DEFAULT_DIR)
    ap.add_argument("--auto", action="store_true", help="启动即进入自动采集")
    ap.add_argument("--target", type=int, default=20, help="自动采集目标张数")
    ap.add_argument("--no-gui", action="store_true", help="无窗口，用于冒烟测试")
    ap.add_argument("--frames", type=int, default=0, help="无窗口模式下最多处理多少帧")
    args = ap.parse_args()

    if args.camera is None:
        try:
            sys.path.insert(0, ROOT)
            import run as _run
            args.camera = _run.default_camera_index()
        except Exception:
            args.camera = 1

    cols, rows = (int(v) for v in args.pattern.lower().split("x"))
    os.makedirs(args.out, exist_ok=True)

    existing = sorted(glob.glob(os.path.join(args.out, "calib_*.jpg")))
    sigs = []
    # 已存在的图没有构图信息，只能按数量计
    print(f"输出目录: {args.out}  （已有 {len(existing)} 张）")

    cap, backend = open_cam(args.camera, args.width, args.height)
    if cap is None:
        print(f"[错误] 打不开相机 index={args.camera}")
        return 1
    print(f"[相机] index={args.camera} backend={backend}")

    auto = args.auto
    last_auto = 0.0
    n = 0
    saved = len(existing)
    fps = 0.0
    t_prev = time.perf_counter()
    cnt = 0
    frame_i = 0

    print("\n操作: SPACE 保存 | a 切换自动 | d 删最后一张 | q 退出")
    print("要在这些角度各拍几张：正中、四角、左倾/右倾/上倾/下倾、近/远\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[警告] 取帧失败")
            break
        frame_i += 1

        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = detect(gray, cols, rows)

        now = time.perf_counter()
        cnt += 1
        if now - t_prev >= 0.5:
            fps = cnt / (now - t_prev)
            cnt = 0
            t_prev = now

        vis = frame.copy()
        sig = None
        if found:
            cv2.drawChessboardCorners(vis, (cols, rows), corners, found)
            sig = signature(corners, w, h)
            novel = is_novel(sig, sigs)
            color = (0, 255, 0) if novel else (0, 165, 255)
            cv2.rectangle(vis, (0, 0), (w - 1, h - 1), color, 8)

        # 自动采集
        if auto and found and sig is not None and is_novel(sig, sigs) and now - last_auto > 0.7:
            p = os.path.join(args.out, f"calib_{saved:03d}.jpg")
            cv2.imwrite(p, frame)          # 存**不带标注的原始帧**
            sigs.append(sig)
            saved += 1
            last_auto = now
            print(f"[自动] 已保存 {os.path.basename(p)}  共 {saved} 张  {coverage_hint(sigs)}")

        # 状态栏
        status = "BOARD FOUND" if found else "no board"
        cv2.rectangle(vis, (0, 0), (w, 64), (0, 0, 0), -1)
        cv2.putText(vis, f"{status}   saved={saved}/{args.target}   {fps:.1f} fps",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (0, 255, 0) if found else (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(vis, f"pattern {cols}x{rows}   {coverage_hint(sigs)}",
                    (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        if auto:
            cv2.putText(vis, "AUTO", (w - 90, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2, cv2.LINE_AA)

        if args.no_gui:
            if args.frames and frame_i >= args.frames:
                break
            continue

        cv2.imshow("calibration capture", vis)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == 32:                       # SPACE
            if not found:
                print("[跳过] 没检测到棋盘格")
            elif sig is not None and not is_novel(sig, sigs):
                print("[跳过] 这张和已有构图太像，换个角度/距离")
            else:
                p = os.path.join(args.out, f"calib_{saved:03d}.jpg")
                cv2.imwrite(p, frame)
                sigs.append(sig)
                saved += 1
                print(f"[保存] {os.path.basename(p)}  共 {saved} 张  {coverage_hint(sigs)}")
        elif key == ord("a"):
            auto = not auto
            print(f"[自动采集] {'开' if auto else '关'}")
        elif key == ord("d"):
            files = sorted(glob.glob(os.path.join(args.out, "calib_*.jpg")))
            if files:
                os.remove(files[-1])
                saved = max(0, saved - 1)
                if sigs:
                    sigs.pop()
                print(f"[删除] {os.path.basename(files[-1])}  剩余 {saved} 张")

    cap.release()
    if not args.no_gui:
        cv2.destroyAllWindows()

    print("\n" + "=" * 60)
    print(f"本次共 {saved} 张（目标 >= 15，建议 20~25）")
    if saved < 15:
        print("[警告] 不足 15 张，标定结果不可靠")
    print(f"{coverage_hint(sigs)}")
    print("\n下一步：")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\calibrate_camera.py "
          f"--images {os.path.relpath(args.out, ROOT)} --pattern {args.pattern} "
          f"--square {args.square:g}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
