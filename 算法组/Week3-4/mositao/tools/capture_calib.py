# -*- coding: utf-8 -*-
"""棋盘格采集助手：自动/手动两种模式。

设计要点（吃过的一次亏）：
- 只靠「按空格保存」很脆弱：窗口没焦点、没检出棋盘格时都没反应，且不给反馈；
- 因此默认改为 **自动模式**：检出即可存，位姿去重，存够数量自动停；
- 同时给每次按键明确回执，绝不再出现「按了没反应还不知道为什么」。

用法：
    # 自动采集（推荐）：对着棋盘缓慢移动，工具自己存，存够 20 张自动结束
    python tools/capture_calib.py --index 1 --auto --target 20

    # 手动：空格存 / u 撤销 / q 退出
    python tools/capture_calib.py --index 1

    # 无窗口自动（只有控制台，适合远程或被窗口抢焦点时）
    python tools/capture_calib.py --index 1 --auto --no-window --target 20

    # 从已有图片导入（自检/补救用；正式数据必须用考核相机实拍）
    python tools/capture_calib.py --source-images .tmp/synthetic_calib --auto

关键数量关系：**内角点数 = 方格数 − 1**。
A4 打印常见两种：
    9×6 个方格  -> 内角点 8×5
    10×7 个方格 -> 内角点 9×6
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import rel                                   # noqa: E402
from tools.calibrate_intrinsics import detect_corners              # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass

BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]
COVER_GRID = 3
IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}


def open_camera(index: int, exposure: float | None = None):
    for name, api in BACKENDS:
        cap = cv2.VideoCapture(index, api)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 30)
            if exposure is not None:
                # MSMF 下 0.75 = 手动曝光，0.25 才是自动；给定值就锁死，
                # 免得自动曝光在亮/暗之间来回跳导致棋盘过曝或变黑。
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
            return cap, name
        cap.release()
    return None, ""


def sharpness(gray: np.ndarray) -> float:
    h, w = gray.shape[:2]
    ch, cw = int(h * 0.6), int(w * 0.6)
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    return float(cv2.Laplacian(gray[y0:y0 + ch, x0:x0 + cw], cv2.CV_64F).var())


def coverage_mask(corners, shape) -> np.ndarray:
    h, w = shape[:2]
    grid = np.zeros((COVER_GRID, COVER_GRID), bool)
    for u, v in corners.reshape(-1, 2):
        gi = min(COVER_GRID - 1, max(0, int(u / (w / COVER_GRID))))
        gj = min(COVER_GRID - 1, max(0, int(v / (h / COVER_GRID))))
        grid[gj, gi] = True
    return grid


def pose_signature(corners) -> list[float]:
    """直接用检出的全部角点坐标做指纹（检测结果按固定顺序返回）。

    用外接框或四角极值都会在「纯旋转」时失真——框几乎不变、极值点可能重复选中，
    导致不同倾角的样本被误判为重复。整组角点坐标最忠实。
    """
    return [float(v) for v in np.asarray(corners).reshape(-1)]


def too_similar(sig: list[float], saved: list[list[float]],
                rel_thr: float = 0.004, abs_min: float = 3.0) -> bool:
    """按「相对棋盘尺寸的平均角点位移」判断是否与已存样本重复。

    用绝对像素不合适：棋盘在画面里的大小时大时小，且纯旋转时多数角点位移很小。
    以棋盘对角线为尺度归一化后，判据才稳定。
    """
    if not sig or len(sig) < 4:
        return False
    xs, ys = sig[0::2], sig[1::2]
    diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    thr = max(abs_min, rel_thr * diag)
    for prev in saved:
        if len(prev) != len(sig):
            continue
        mean_disp = sum(abs(a - b) for a, b in zip(sig, prev)) / len(sig)
        if mean_disp < thr:
            return True
    return False


def tilt_proxy(corners) -> float:
    """无需内参的倾斜代理量：正对时对边等长，倾斜时透视使对边长度不等。

    取棋盘外围四条边的长度，计算「对边长度相对差异」的均值。
    0 = 完全正对；越大越倾斜。经验值：约 15° 倾斜对应 0.04~0.06。
    """
    pts = corners.reshape(-1, 2)
    h, w = pts.shape
    # 检测结果按行列展平，这里用四角构成的外框估计四条边
    n = len(pts)
    # 用包围四边形：取四个最外角（左上、右上、右下、左下）
    s = pts[:, 0] + pts[:, 1]
    d = pts[:, 0] - pts[:, 1]
    tl, br = pts[np.argmin(s)], pts[np.argmax(s)]
    tr, bl = pts[np.argmax(d)], pts[np.argmin(d)]
    top = np.linalg.norm(tr - tl)
    bottom = np.linalg.norm(br - bl)
    left = np.linalg.norm(bl - tl)
    right = np.linalg.norm(br - tr)
    eps = 1e-6
    a = abs(top - bottom) / (0.5 * (top + bottom) + eps)
    b = abs(left - right) / (0.5 * (left + right) + eps)
    return float((a + b) / 2.0)


def decision(found: bool, sig: list[float], saved: list[list[float]],
             sharp: float, min_sharp: float, tilt: float = 1.0,
             min_tilt: float = 0.0, rel_thr: float = 0.004):
    """是否保存，以及原因。返回 (是否保存, 提示文本, 级别)。

    级别：ok / warn / err，用于给界面配色，也用于控制台回执。
    """
    if not found:
        return False, "未检出棋盘格：检查内角点数(方格数-1)、棋盘是否完整入画、是否太远", "err"
    if sharp < min_sharp:
        return False, f"画面偏糊(清晰度 {sharp:.0f} < {min_sharp:.0f})，先对焦再采集", "warn"
    if min_tilt > 0 and tilt < min_tilt:
        return False, (f"太正对(倾斜量 {tilt:.3f} < {min_tilt:.3f})："
                       f"把棋盘绕左右、上下各转 15°~30°，否则焦距解不出来"), "warn"
    if too_similar(sig, saved, rel_thr):
        # 只提示、不拦截：宁可多存一张，也不要再出现「按了没反应、一张都没存」
        return True, "已保存，但与既有样本较相似（建议再换个角度补几张）", "warn"
    return True, "已保存", "ok"


def save_image(frame, corners, out_dir: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    p = out_dir / f"calib_{stamp}.png"
    cv2.imwrite(str(p), frame)
    return p


def report_and_exit(saved: list, grid_total: np.ndarray, out_dir: Path,
                    cols: int, rows: int) -> int:
    print("-" * 62)
    print(f"共保存 {len(saved)} 张，覆盖度 {int(grid_total.sum())}/9，目录 {rel(out_dir)}")
    if len(saved) < 15:
        print(f"数量不足：建议 ≥15 张（当前 {len(saved)}）。可再跑一次，已存的会自动跳过重复位姿。")
    if grid_total.sum() < 8:
        print("覆盖度不足：把棋盘在画面的九个区域都走一遍，尤其是四个角。")
    if not saved:
        print()
        print("一张都没存下，按可能性排序检查：")
        print("  1. 内角点数填错：数的是「四格交会的内角点」，= 方格数 - 1；")
        print("     9×6 个方格 -> --cols 8 --rows 5；10×7 个方格 -> --cols 9 --rows 6")
        print("  2. 棋盘没完整入画，或离得太远（格子小于约 15 px 就检不出）")
        print("  3. 反光/背光：黑格泛白、白格发灰都会检不出")
        print("  4. 画面糊：先跑 tools/focus_tune.py 把清晰度调到 150 以上")
        return 1
    print("下一步：")
    print(f"  .\\run.bat tools\\calibrate_intrinsics.py --cols {cols} --rows {rows} "
          f"--square-mm <实测方格边长 mm>")
    return 0


def detect_pattern_sizes(args) -> int:
    """自动识别棋盘规格：枚举常见内角点组合，报告哪些能被检出。

    这是「始终 0 张」最快的诊断——内角点数填错比任何其它原因都常见。
    """
    candidates = [(9, 6), (8, 5), (10, 7), (7, 5), (8, 6), (9, 7), (6, 9), (5, 8),
                  (7, 6), (6, 7), (11, 8), (10, 6), (12, 9), (6, 5), (5, 6)]
    cap, backend = open_camera(args.index, getattr(args, "exposure", None))
    if cap is None:
        print(f"打不开相机 index={args.index}")
        return 1
    print(f"相机: index={args.index} 后端={backend}｜采样 {args.frames} 帧用于识别棋盘规格")
    hits: dict = {}
    best_img = None
    for _ in range(args.frames):
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if best_img is None or sharpness(gray) > sharpness(best_img):
            best_img = gray
        for cols, rows in candidates:
            found, _ = detect_corners(gray, cols, rows)
            if found:
                hits[(cols, rows)] = hits.get((cols, rows), 0) + 1
    cap.release()

    print("-" * 62)
    if not hits:
        # 关键诊断：把失败时的画面存下来，用户（或异地排查的人）可直接看
        if best_img is not None:
            dbg = ROOT / "outputs" / "calib_debug"
            dbg.mkdir(parents=True, exist_ok=True)
            p = dbg / f"detect_fail_{datetime.now():%H%M%S}.png"
            cv2.imwrite(str(p), best_img)
            print(f"已保存当时画面：{rel(p)}")
            print("请打开这张图确认：画面里是否真的有一块完整的黑白棋盘格。")
        print("没有任何规格被检出。请检查：")
        print("  1. 棋盘是否在画面里、且完整入画（最常见的原因就是根本没对准）")
        print("  2. 是否太远：单格 < 约 15 px 检不出。A4 横向摆放建议距离 0.4~0.5 m")
        print("  3. 是否反光/背光：强光会让黑格泛白，局部整片失效")
        print("  4. 画面是否真糊：对准有纹理的目标跑 tools/focus_tune.py，读数应 >150")
        return 1
    print("可用的规格（内角点 cols x rows，命中帧数）：")
    for (cols, rows), n in sorted(hits.items(), key=lambda kv: -kv[1]):
        print(f"  --cols {cols} --rows {rows}   命中 {n}/{args.frames} 帧")
    (bc, br), _ = max(hits.items(), key=lambda kv: kv[1])
    print("-" * 62)
    print(f"建议使用：--cols {bc} --rows {br}")
    print("对应方格数：" + f"{bc + 1} x {br + 1}")
    return 0


def run_camera(args, out_dir: Path) -> int:
    cap, backend = open_camera(args.index, getattr(args, "exposure", None))
    if cap is None:
        print(f"打不开相机 index={args.index}（先跑 tools/camera_check.py 确认索引）")
        return 1
    print(f"相机: index={args.index} 后端={backend} "
          f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print(f"棋盘: 内角点 {args.cols}x{args.rows}｜保存到 {rel(out_dir)}")
    print(f"模式: {'自动连拍' if args.auto else '手动'}｜窗口: {'关' if args.no_window else '开'}"
          f"｜清晰度下限 {args.min_sharpness:.0f}")

    saved: list[list[float]] = []
    grid_total = np.zeros((COVER_GRID, COVER_GRID), bool)
    last_save = 0.0
    use_window = not args.no_window
    if use_window:
        cv2.namedWindow("calib capture", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("calib capture", 1000, 620)
        print("操作: 空格保存 / u 撤销 / q 退出")
    else:
        print("无窗口模式：对着棋盘缓慢移动即可，达到目标数量自动结束（Ctrl+C 中止）")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("取帧失败：相机可能被占用或已断开")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sharp = sharpness(gray)
        found, corners = detect_corners(gray, args.cols, args.rows)
        sig = pose_signature(corners) if found else []
        tilt = tilt_proxy(corners) if found else 0.0
        can_save, why, level = decision(found, sig, saved, sharp, args.min_sharpness,
                                        tilt=tilt, min_tilt=args.min_tilt)

        if found:
            grid_total |= coverage_mask(corners, gray.shape)

        # 自动模式：满足条件且距上次保存超过间隔就存
        if args.auto:
            now = time.perf_counter()
            if can_save and (now - last_save) >= args.interval:
                p = save_image(frame, corners, out_dir)
                saved.append(sig)
                last_save = now
                print(f"[{len(saved):2d}/{args.target}] 已保存 {rel(p)}"
                      f"（清晰度 {sharp:.0f}，覆盖 {int(grid_total.sum())}/9）")
                if len(saved) >= args.target:
                    break

        if use_window:
            view = frame.copy()
            if found:
                cv2.drawChessboardCorners(view, (args.cols, args.rows), corners, found)
            color = {"ok": (0, 200, 0), "warn": (0, 165, 255), "err": (0, 0, 220)}[level]
            head = "自动采集中" if args.auto else "手动（空格保存）"
            cv2.putText(view, f"{head}  已存 {len(saved)}/{args.target}", (14, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (240, 240, 240), 2)
            cv2.putText(view, why if not can_save else "可以保存（自动模式会自动存）",
                        (14, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)
            cv2.putText(view, f"sharp={sharp:.0f}  cover={int(grid_total.sum())}/9  "
                              f"tilt={tilt:.3f}", (14, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)
            cell, ox, oy = 26, view.shape[1] - COVER_GRID * 26 - 14, view.shape[0] - COVER_GRID * 26 - 14
            for gj in range(COVER_GRID):
                for gi in range(COVER_GRID):
                    p0 = (ox + gi * cell, oy + gj * cell)
                    cv2.rectangle(view, p0, (p0[0] + cell - 2, p0[1] + cell - 2),
                                  (0, 200, 0) if grid_total[gj, gi] else (190, 190, 190), -1)
            cv2.imshow("calib capture", view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("u"):
                if saved:
                    saved.pop()
                    files = sorted(out_dir.glob("calib_*.png"))
                    if files:
                        files[-1].unlink()
                    print("已撤销最后一张")
                else:
                    print("还没有保存过任何图，无法撤销")
            elif key == ord(" "):
                if can_save:
                    p = save_image(frame, corners, out_dir)
                    saved.append(sig)
                    print(f"已保存 {rel(p)}（第 {len(saved)} 张，清晰度 {sharp:.0f}）")
                    if len(saved) >= args.target:
                        break
                else:
                    print(f"未保存：{why}")

    cap.release()
    if use_window:
        cv2.destroyAllWindows()
    return report_and_exit(saved, grid_total, out_dir, args.cols, args.rows)


def run_from_images(args, out_dir: Path) -> int:
    src = Path(args.source_images)
    if not src.is_absolute():
        src = ROOT / src
    files = sorted(p for p in src.iterdir() if p.suffix.lower() in IMG_EXTS)
    print(f"来源: {rel(src)}（{len(files)} 张）｜棋盘内角点 {args.cols}x{args.rows}")
    saved_sigs: list[list[float]] = []
    grid = np.zeros((COVER_GRID, COVER_GRID), bool)
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = detect_corners(gray, args.cols, args.rows)
        sig = pose_signature(corners) if found else []
        tilt = tilt_proxy(corners) if found else 0.0
        can, why, _ = decision(found, sig, saved_sigs, sharpness(gray), args.min_sharpness,
                               tilt=tilt, min_tilt=args.min_tilt)
        if not can:
            print(f"  跳过 {f.name}：{why}")
            continue
        p = save_image(img, corners, out_dir)
        saved_sigs.append(sig)
        grid |= coverage_mask(corners, gray.shape)
        print(f"  接收 {f.name} -> {rel(p)}")
        if len(saved_sigs) >= args.target:
            break
    return report_and_exit(saved_sigs, grid, out_dir, args.cols, args.rows)


def main() -> int:
    ap = argparse.ArgumentParser(description="棋盘格采集助手")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--cols", type=int, default=9, help="内角点列数 = 方格列数-1")
    ap.add_argument("--rows", type=int, default=6, help="内角点行数 = 方格行数-1")
    ap.add_argument("--out", default="data/calib")
    ap.add_argument("--auto", action="store_true", help="自动连拍")
    ap.add_argument("--target", type=int, default=20, help="目标张数")
    ap.add_argument("--interval", type=float, default=0.6, help="自动模式最小保存间隔（秒）")
    ap.add_argument("--no-window", action="store_true", help="不开窗口（配合 --auto）")
    ap.add_argument("--exposure", type=float, default=None,
                    help="固定曝光（MSMF 手动值，−1.5 左右合适；不给则用相机自动）")
    ap.add_argument("--min-sharpness", type=float, default=80.0,
                    help="低于该清晰度不保存")
    ap.add_argument("--min-tilt", type=float, default=0.045,
                    help="倾斜量门限（0.045 ≈ 12°~15°）；正对样本会让焦距解不出来")
    ap.add_argument("--source-images", default=None, help="从图片目录导入（自检/补救）")
    ap.add_argument("--detect-pattern", action="store_true",
                    help="自动识别棋盘规格（内角点数），不采集")
    ap.add_argument("--frames", type=int, default=15, help="--detect-pattern 采样帧数")
    args = ap.parse_args()

    out_dir = ROOT / args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.detect_pattern:
        return detect_pattern_sizes(args)
    if args.source_images:
        return run_from_images(args, out_dir)
    return run_camera(args, out_dir)


if __name__ == "__main__":
    raise SystemExit(main())
