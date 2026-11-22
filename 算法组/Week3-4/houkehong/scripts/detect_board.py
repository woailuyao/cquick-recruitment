# -*- coding: utf-8 -*-
"""
棋盘格规格自动识别

标定要用对两个参数，错一个内参就废：
    --pattern  内角点数（不是方格数！9x6 内角点 = 10x7 个方格）
    --square   方格边长（毫米，必须用尺子量）

本脚本负责第一个：把棋盘格对着相机，它会试一堆常见图案，
把能检出的全列出来，省得你数错。

用法：

    # 对着相机（相机预览里举起棋盘格）
    python scripts/detect_board.py --camera 1

    # 或者用一张已经拍好的照片
    python scripts/detect_board.py --image path/to/board.jpg

第二个参数（方格边长）必须你用直尺量：量一个黑格的边长，单位毫米，常见 20 / 25 / 30 mm。
量完把两个参数一起传给 calibrate_capture.py。
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

# 常见棋盘格的内角点数（cols, rows）。顺序按出现频率排。
CANDIDATES = [
    (9, 6), (8, 6), (7, 5), (10, 7), (8, 5), (11, 8),
    (6, 5), (5, 4), (6, 4), (9, 7), (12, 9), (4, 3), (8, 7), (10, 8),
]


def grab(camera, width, height, settle=45):
    """
    抓一帧。

    settle 要够大：自动曝光从"对着暗的草地"切到"对着白纸"时会大幅调整，
    实测只等 15 帧（约 0.5 秒）会抓到严重过曝的中间态，整张白掉、角点全检不出。
    """
    for name, flag in (("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)):
        cap = cv2.VideoCapture(camera, flag)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = 自动曝光
        frame = None
        for _ in range(settle):
            ok, f = cap.read()
            if ok and f is not None:
                frame = f
        cap.release()
        if frame is not None:
            return frame, name
    return None, None


def try_pattern(gray, cols, rows):
    """返回 (ok, method, corners)。SB 优先。"""
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, c = cv2.findChessboardCornersSB(
            gray, (cols, rows),
            getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0) | getattr(cv2, "CALIB_CB_ACCURACY", 0))
        if ok:
            return True, "SB", c
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if ok:
        return True, "classic", c
    return False, "", None


def border_margin(corners, img_w, img_h):
    """检测到的角点距画面边缘的最小像素距离。太小说明板子被裁切了。"""
    if corners is None:
        return 0.0
    pts = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    return float(min(pts[:, 0].min(), pts[:, 1].min(),
                     img_w - 1 - pts[:, 0].max(), img_h - 1 - pts[:, 1].max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--image", default=None)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[错误] 读不了图片: {args.image}")
            return 1
        src = args.image
    else:
        if args.camera is None:
            try:
                import run as _run
                args.camera = _run.default_camera_index()
            except Exception:
                args.camera = 1
        img, backend = grab(args.camera, args.width, args.height)
        if img is None:
            print(f"[错误] 打不开相机 index={args.camera}")
            return 1
        src = f"camera{args.camera}/{backend}"
        out = os.path.join(ROOT, "snapshots", "board_probe.jpg")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        cv2.imwrite(out, img)
        print(f"已抓图 -> {out}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mean, std = float(gray.mean()), float(gray.std())
    print("=" * 60)
    print(f"图像: {src}   {img.shape[1]}x{img.shape[0]}")
    print(f"亮度: 均值 {mean:.1f}  标准差 {std:.1f}")
    print("=" * 60)
    if mean > 200 and std < 40:
        print("⚠️  画面严重过曝（整张发白），角点不可能检出。")
        print("   处理：把相机移远一点 / 避免强光直射镜头 / 换个角度别正对灯，")
        print("        然后重新运行。")
        return 4
    if mean < 35:
        print("⚠️  画面太暗，角点也检不出。开灯或换个位置后重试。")
        return 5
    print(f"{'内角点':>8} {'方格数':>8}  检出")
    print("-" * 60)

    hits = []
    corners_of = {}
    for cols, rows in CANDIDATES:
        ok, method, c = try_pattern(gray, cols, rows)
        if ok:
            hits.append((cols, rows, method))
            corners_of[(cols, rows)] = c

    # 一块 9x6 的板子内部必然也能匹配到 6x5、5x4 这类子网格，
    # 所以按角点数从多到少排序，最多的一定是整块板子。
    hits.sort(key=lambda t: t[0] * t[1], reverse=True)

    h_img, w_img = gray.shape[:2]
    for i, (cols, rows, method) in enumerate(hits):
        mark = "✅ 选这个" if i == 0 else "   (子网格，忽略)"
        print(f"{f'{cols}x{rows}':>8} {f'{cols+1}x{rows+1}':>8}  {method:<8} {mark}")
    print("-" * 60)

    # 关键检查：最外层角点是否贴到画面边缘。
    # 贴边说明只拍到了板子的一部分，此时"最多角点"未必是整块板，
    # 结论会偏小，必须让用户重新取景。
    if hits:
        top = hits[0]
        margin = border_margin(corners_of.get((top[0], top[1])), w_img, h_img)
        print(f"最外层角点距画面边缘: {margin:.0f} px "
              f"({'贴边' if margin < 12 else '正常'})")
        if margin < 12:
            print()
            print("⚠️  棋盘格被画面裁切了！只能看到一部分，")
            print("   所以上面『角点最多的那个』很可能不是整块板子，结论不可信。")
            print()
            print("   请这样重新取景：")
            print("   · 让整块板的四条边都完整进画面，四周各留 30~50 px 余量")
            print("   · 不要拍到手、不要贴到画面边上")
            print("   · 板子尽量正对相机、光线均匀、避免反光")
            print("   然后重新运行本命令。")
            return 3

    if not hits:
        print("没有检出任何常见图案。可能原因：")
        print("  1. 棋盘格没完整进入画面（四边都要看到白边）");
        print("  2. 画面太暗/太亮、反光严重");
        print("  3. 板子倾斜过度或被手挡住");
        print("  4. 这张板子不是常见规格，手动数一下内角点：")
        print("     横向『内部交点』个数 x 纵向『内部交点』个数")
        print("     （不是方格数！8 个方格的边长上是 7 个内角点）")
        return 2

    c, r, _ = hits[0]
    print(f"\n结论：内角点 {c} x {r}  （即 {c+1} x {r+1} 个方格）")
    if len(hits) > 1:
        print("其余命中的都是这块板内部的子网格（板子越大越容易匹配到），忽略即可。")
        print("如果结论看起来不对，用手数一遍：横向内角点个数 x 纵向内角点个数"
              "（不是方格数）")

    # 把识别结果画回图上存一份，供人工复核。
    # 自动识别最容易犯的错就是"板子只拍到一部分，于是报出一个偏小的子网格"，
    # 光看数字看不出来，画出来一眼就能发现。
    try:
        ann = img.copy()
        cc = corners_of.get((c, r))
        if cc is not None:
            cv2.drawChessboardCorners(ann, (c, r), cc, True)
            pts = np.asarray(cc, dtype=np.float64).reshape(-1, 2)
            x0, y0 = pts.min(axis=0).astype(int)
            x1, y1 = pts.max(axis=0).astype(int)
            cv2.rectangle(ann, (x0 - 12, y0 - 12), (x1 + 12, y1 + 12), (0, 0, 255), 2)
        cv2.rectangle(ann, (0, 0), (ann.shape[1], 40), (0, 0, 0), -1)
        cv2.putText(ann, f"detected {c}x{r} inner corners ({c+1}x{r+1} squares)",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        apath = os.path.join(ROOT, "snapshots", "board_probe_annotated.jpg")
        cv2.imwrite(apath, ann)
        print(f"\n复核图 -> {apath}")
        print("请打开看一眼：红框是否把**整块板**框住？如果板子还有一部分在红框外，")
        print("说明画面没拍全，请重新取景后再跑一次。")
    except Exception as e:
        print(f"(复核图生成失败: {e})")

    print("\n还需要你用直尺量一个【方格的边长】，单位毫米（常见 20 / 25 / 30）。")
    print("量完运行：")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\calibrate_capture.py "
          f"--camera {args.camera if args.camera is not None else 1} --pattern {c}x{r}")
    print(f"  .\\.venv\\Scripts\\python.exe scripts\\calibrate_camera.py "
          f"--images calib_images --pattern {c}x{r} --square <量到的毫米数>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
