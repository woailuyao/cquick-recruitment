"""task2 工具集：相机标定 + 数据整理 + 场地图生成

用法：
  python tools.py calib record|capture|extract|calibrate|diag|focus
  python tools.py data  extract|dataset|normalize|prelabel|flag|xml2txt
  python tools.py map   [--r 0.75]
"""

import argparse
import csv
import json
import math
import shutil
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np


# ======================================================================
# 一、相机：录制 / 拍照 / 抽帧 / 标定 / 诊断
# ======================================================================



BASE = Path(__file__).resolve().parent.parent / "calibration"


def open_camera(source, width=1280, height=720):
    cap = cv2.VideoCapture(source, cv2.CAP_MSMF)
    if not cap.isOpened():
        cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"打不开摄像头 {source}，换一个编号试试：--source 0/1/2/3")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def try_warmup(cap, tries=40):
    for _ in range(tries):
        ok, frame = cap.read()
        if ok and frame is not None and float(frame.mean()) > 1:
            return frame
        time.sleep(0.02)
    return None


def warmup(cap, source, tries=40):
    frame = None
    ok = False
    for _ in range(tries):
        ok, frame = cap.read()
        if ok and frame is not None and float(frame.mean()) > 1:
            break
        time.sleep(0.02)
    if not ok or frame is None:
        raise SystemExit(f"摄像头 {source} 没有画面")
    return frame


def find_corners(gray, cols, rows):
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows))
    if ok:
        return True, corners
    ok, corners = cv2.findChessboardCorners(
        gray, (cols, rows),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if ok:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return ok, corners


def parse_pattern(text):
    cols, rows = (int(v) for v in text.lower().split("x"))
    return cols, rows


def save_png(path, img):
    ok, buf = cv2.imencode(".png", img)
    if ok:
        buf.tofile(str(path))


def key_yaml(path, k, dist, size, rms, n_images):
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[0, 2], k[1, 2]
    d = dist.ravel()
    lines = [
        "# 由 tools_calib.py calibrate 自动生成",
        f"image_size: [{size[0]}, {size[1]}]",
        f"n_images: {n_images}",
        f"rms_reprojection_error: {rms:.6f}",
        "camera_matrix:",
        f"  - [{k[0,0]:.6f}, {k[0,1]:.6f}, {k[0,2]:.6f}]",
        f"  - [{k[1,0]:.6f}, {k[1,1]:.6f}, {k[1,2]:.6f}]",
        f"  - [{k[2,0]:.6f}, {k[2,1]:.6f}, {k[2,2]:.6f}]",
        f"fx: {fx:.6f}",
        f"fy: {fy:.6f}",
        f"cx: {cx:.6f}",
        f"cy: {cy:.6f}",
        "dist_coeffs:",
        f"  - [{d[0]:.8f}, {d[1]:.8f}, {d[2]:.8f}, {d[3]:.8f}, {d[4]:.8f}]",
        f"k1: {d[0]:.8f}",
        f"k2: {d[1]:.8f}",
        f"p1: {d[2]:.8f}",
        f"p2: {d[3]:.8f}",
        f"k3: {d[4]:.8f}",
        "dist_model: k1,k2,p1,p2,k3",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def pose_key(corners, cols, w, h):
    pts = corners.reshape(-1, 2)
    cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
    span = max(pts[:, 0].max() - pts[:, 0].min(), pts[:, 1].max() - pts[:, 1].min())
    m = min(w, h)
    area_bin = 0 if span < 0.3 * m else (1 if span < 0.55 * m else (2 if span < 0.8 * m else 3))
    gx = min(3, max(0, int(cx / w * 4)))
    gy = min(3, max(0, int(cy / h * 4)))
    ang = np.degrees(np.arctan2(pts[cols - 1][1] - pts[0][1], pts[cols - 1][0] - pts[0][0]))
    ang_bin = min(2, max(0, int((ang + 45) // 30)))
    return (gx, gy, area_bin, ang_bin)


def too_similar(a, b, tol=4.0):
    return np.sqrt(((a - b) ** 2).sum(axis=1)).mean() < tol


def cmd_record(args):
    out = Path(args.out) if args.out else BASE / "raw_calib.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    source = int(args.source) if str(args.source).isdigit() else args.source
    cap = open_camera(source)
    frame = try_warmup(cap)
    if frame is None:
        cap.release()
        cap = open_camera(source, width=None)
        frame = try_warmup(cap)
        if frame is None:
            cap.release()
            raise SystemExit(f"摄像头 {source} 没有画面：换个编号试试 --source 0/1/2/3")
    h, w = frame.shape[:2]
    print(f"实际分辨率 {w}x{h}")
    if (w, h) != (1280, 720):
        print(f"注意：相机给的是 {w}x{h}，不是 1280x720，按这个分辨率录制并标定")
    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (w, h))
    print("开始录制，按 q 停止")
    print("动作：板子放正中→左上→右上→右下→左下，每处停 2 秒并慢慢倾斜")
    frames = 0
    t0 = time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        writer.write(frame)
        frames += 1
        cv2.putText(frame, f"REC {frames}  {time.time() - t0:.0f}s", (10, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        if not args.no_show:
            cv2.imshow("record", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        elif args.max_frames and frames >= args.max_frames:
            break
    cap.release()
    writer.release()
    cv2.destroyAllWindows()
    print(f"录完 {frames} 帧，{time.time() - t0:.0f} 秒 -> {out}")
    return 0


def cmd_capture(args):
    cols, rows = parse_pattern(args.pattern)
    out = Path(args.out) if args.out else BASE / "img"
    out.mkdir(parents=True, exist_ok=True)
    saved = len(list(out.glob("*.png")))
    print(f"内角点 {cols}x{rows}，保存目录 {out}，已有 {saved} 张")
    source = int(args.source) if str(args.source).isdigit() else args.source
    cap = open_camera(source)
    warmup(cap, source)
    last_auto = 0.0
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        if args.frames and n > args.frames:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = find_corners(gray, cols, rows)
        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, (cols, rows), corners, found)
        color = (0, 255, 0) if found else (0, 0, 255)
        cv2.putText(view, f"{'FOUND' if found else 'no board'}  saved={saved}", (12, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(view, "space=save  a=auto  q=quit", (12, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        do_save = False
        if args.no_show:
            if found and args.auto:
                do_save = True
        else:
            cv2.imshow("calib capture", view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" ") and found:
                do_save = True
            if key == ord("a") and found and time.time() - last_auto > 0.7:
                do_save = True
                last_auto = time.time()
        if do_save:
            saved += 1
            save_png(out / f"calib_{saved:03d}.png", frame)
            print(f"[{saved}] saved calib_{saved:03d}.png")
    cap.release()
    cv2.destroyAllWindows()
    print(f"结束，共 {saved} 张")
    return 0


def calib_cmd_extract(args):
    cols, rows = parse_pattern(args.pattern)
    out = Path(args.out) if args.out else BASE / "img"
    out.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频 {args.video}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    plan = total // args.step
    print(f"视频 {total} 帧，每 {args.step} 帧看 1 帧，预计 {plan} 次，检测宽度 {args.detect_width}")
    candidates = []
    idx = checked = 0
    t0 = time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if idx % args.step:
            continue
        checked += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        scale = args.detect_width / w if args.detect_width and args.detect_width < w else 1.0
        small = gray if scale == 1.0 else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        sharp = float(cv2.Laplacian(small, cv2.CV_64F).var())
        if sharp >= args.min_sharp:
            found, corners = find_corners(small, cols, rows)
            if found:
                corners = corners * (1.0 / scale)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners.astype(np.float32), (9, 9), (-1, -1), criteria)
                candidates.append({
                    "sharp": sharp,
                    "frame": frame.copy(),
                    "idx": idx,
                    "corners": corners.reshape(-1, 2),
                    "key": pose_key(corners, cols, w, h),
                })
        if checked % 100 == 0:
            used = time.time() - t0
            speed = checked / max(used, 1e-6)
            left = (plan - checked) / speed if speed else 0
            print(f"  已检查 {checked}/{plan} 已用 {used:.0f}s 剩余约 {left:.0f}s 可用 {len(candidates)}", flush=True)
    cap.release()
    print(f"共检出可用帧 {len(candidates)}")
    if not candidates:
        raise SystemExit("一个角点都没找到：确认内角点是 9x6、板子完整且清晰")
    best_per_key = {}
    for c in candidates:
        k = c["key"]
        if k not in best_per_key or c["sharp"] > best_per_key[k]["sharp"]:
            best_per_key[k] = c
    picked = sorted(best_per_key.values(), key=lambda c: -c["sharp"])[: args.want]
    if len(picked) < args.want:
        rest = sorted(candidates, key=lambda c: -c["sharp"])
        for c in rest:
            if len(picked) >= args.want:
                break
            if all(c["idx"] != p["idx"] and not too_similar(c["corners"], p["corners"]) for p in picked):
                picked.append(c)
    picked.sort(key=lambda c: c["idx"])
    for i, c in enumerate(picked, 1):
        save_png(out / f"calib_{i:03d}.png", c["frame"])
        print(f"[{i:02d}] 第 {c['idx']} 帧 清晰度 {c['sharp']:.0f} -> calib_{i:03d}.png")
    print(f"共存 {len(picked)} 张到 {out}")
    if len(picked) < 15:
        print("不足 15 张：要么重录，要么 --step 调到 2")
    return 0


def cmd_calibrate(args):
    cols, rows = parse_pattern(args.pattern)
    img_dir = Path(args.dir) if args.dir else BASE / "img"
    out_yaml = Path(args.out) if args.out else BASE / "intrinsics.yaml"
    files = sorted(list(img_dir.glob("*.png")) + list(img_dir.glob("*.jpg")))
    if not files:
        raise SystemExit(f"{img_dir} 里没有图片，先 capture 或 extract")
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square
    objpoints, imgpoints, used = [], [], []
    size = None
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"跳过（读不了）：{f.name}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        ok, corners = find_corners(gray, cols, rows)
        if not ok:
            print(f"跳过（没找到 {cols}x{rows} 角点）：{f.name}")
            continue
        objpoints.append(objp.copy())
        imgpoints.append(corners)
        used.append(f.name)
    if len(used) < 5:
        raise SystemExit(f"只识别到 {len(used)} 张，至少 5 张（建议 15 张以上）")
    rms, k, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)
    print(f"用到的图片：{len(used)} 张 {size[0]}x{size[1]}")
    print(f"重投影误差 RMS = {rms:.4f} px")
    print(f"fx={k[0,0]:.2f} fy={k[1,1]:.2f} cx={k[0,2]:.2f} cy={k[1,2]:.2f}")
    d = dist.ravel()
    print(f"k1={d[0]:.5f} k2={d[1]:.5f} p1={d[2]:.5f} p2={d[3]:.5f} k3={d[4]:.5f}")
    worst = []
    for i, (op, ip, rv, tv) in enumerate(zip(objpoints, imgpoints, rvecs, tvecs)):
        proj, _ = cv2.projectPoints(op, rv, tv, k, dist)
        err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - ip.reshape(-1, 2), axis=1)))
        worst.append((err, used[i]))
    worst.sort(reverse=True)
    print("误差最大的 3 张：")
    for err, name in worst[:3]:
        print(f"  {name}  {err:.3f} px")
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    key_yaml(out_yaml, k, dist, size, rms, len(used))
    print(f"已写入 {out_yaml}")
    sample = cv2.imread(str(img_dir / used[0]))
    h, w = sample.shape[:2]
    new_k, _ = cv2.getOptimalNewCameraMatrix(k, dist, (w, h), 1, (w, h))
    fixed = cv2.undistort(sample, k, dist, None, new_k)
    pair = np.hstack([sample, fixed])
    cv2.putText(pair, "before", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(pair, "after undistort", (w + 12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    ok, buf = cv2.imencode(".jpg", pair)
    if ok:
        buf.tofile(str(out_yaml.parent / "undistort_compare.jpg"))
        print(f"去畸变对比图 {out_yaml.parent / 'undistort_compare.jpg'}")
    return 0


def cmd_diag(args):
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit("打不开视频，检查路径")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    checked = ok_sharp = ok_found = 0
    sharp_list = []
    i = 0
    cols, rows = parse_pattern(args.pattern)
    print(f"开始处理，共 {total} 帧，每 10 帧看 1 帧")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        i += 1
        if i % 10:
            continue
        checked += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        s = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        sharp_list.append(s)
        if s >= args.min_sharp:
            ok_sharp += 1
        found, _ = cv2.findChessboardCornersSB(gray, (cols, rows))
        if found:
            ok_found += 1
        if checked % 50 == 0:
            print(f"  已检查 {checked} 帧，命中角点 {ok_found} 次", flush=True)
    cap.release()
    print()
    print(f"总帧: {total}")
    print(f"抽样检查: {checked} 帧")
    print(f"清晰度 >= {args.min_sharp:.0f} 的: {ok_sharp} ({100 * ok_sharp / max(checked, 1):.0f}%)")
    print(f"能找到 {cols}x{rows} 角点的: {ok_found} ({100 * ok_found / max(checked, 1):.0f}%)")
    print(f"清晰度中位数 {np.median(sharp_list):.1f} 最小 {min(sharp_list):.1f} 最大 {max(sharp_list):.1f}")
    return 0


def cmd_focus(args):
    source = int(args.source) if str(args.source).isdigit() else args.source
    cap = open_camera(source)
    best = 0.0
    print("举起棋盘格前后移动，看左上角数字；q 退出")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("读不到画面")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        found, _ = cv2.findChessboardCornersSB(gray, (9, 6))
        best = max(best, sharp)
        color = (0, 255, 0) if sharp >= 150 else (0, 165, 255) if sharp >= 80 else (0, 0, 255)
        cv2.putText(frame, f"sharp {sharp:.0f}  best {best:.0f}  corners {'YES' if found else 'no'}",
                    (10, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        if not args.no_show:
            cv2.imshow("focus check", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        elif args.max_frames:
            args.max_frames -= 1
            if args.max_frames <= 0:
                break
    cap.release()
    cv2.destroyAllWindows()
    print(f"本次最高清晰度 {best:.0f}")
    return 0


def _calib_main(argv=None):
    ap = argparse.ArgumentParser(description="相机标定工具合集")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("record", help="录标定视频")
    p.add_argument("--source", default="1")
    p.add_argument("--out", default="")
    p.add_argument("--no-show", action="store_true")
    p.add_argument("--max-frames", type=int, default=0)
    p.set_defaults(func=cmd_record)

    p = sub.add_parser("capture", help="实时拍照")
    p.add_argument("--source", default="1")
    p.add_argument("--pattern", default="9x6")
    p.add_argument("--out", default="")
    p.add_argument("--auto", action="store_true")
    p.add_argument("--no-show", action="store_true")
    p.add_argument("--frames", type=int, default=0)
    p.set_defaults(func=cmd_capture)

    p = sub.add_parser("extract", help="从视频抽帧")
    p.add_argument("--video", required=True)
    p.add_argument("--pattern", default="9x6")
    p.add_argument("--out", default="")
    p.add_argument("--want", type=int, default=24)
    p.add_argument("--step", type=int, default=5)
    p.add_argument("--min-sharp", type=float, default=40.0)
    p.add_argument("--detect-width", type=int, default=960)
    p.set_defaults(func=calib_cmd_extract)

    p = sub.add_parser("calibrate", help="算内参")
    p.add_argument("--pattern", default="9x6")
    p.add_argument("--square", type=float, default=25.0)
    p.add_argument("--dir", default="")
    p.add_argument("--out", default="")
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("diag", help="诊断视频清晰度/角点命中率")
    p.add_argument("--video", required=True)
    p.add_argument("--pattern", default="9x6")
    p.add_argument("--min-sharp", type=float, default=60.0)
    p.set_defaults(func=cmd_diag)

    p = sub.add_parser("focus", help="实时看清晰度")
    p.add_argument("--source", default="1")
    p.add_argument("--no-show", action="store_true")
    p.add_argument("--max-frames", type=int, default=0)
    p.set_defaults(func=cmd_focus)

    args = ap.parse_args(argv)
    raise SystemExit(args.func(args))

# ======================================================================
# 二、数据：抽帧 / 建数据集 / 框规范化 / 预标注 / 查错
# ======================================================================



HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DS = ROOT / "dataset"
NAMES = ["L", "T", "X", "Point"]

DEFAULT_VIDEO_DIR = str(Path.home() / "Desktop")
DEFAULT_VIDEOS = [
    "dchbggvvvvvvvvhvgvgcgvvvvvsdcsavi.avi.avi.avi",
    "dchbggvvvvvvvvklhvgvgcgvvvvvsdcsavi.avi.avi.avi.avi",
    "dchbggvvvvvvvvvvvvvsdcsavi.avi.avi",
    "dchbhbsdcsavi.avi",
    "hbhvgcfc.avi.avi",
    "zuiqiushipin(gun).avi.avi.avi.avi",
    "zuiqiushipin(qiubudong).avi",
    "zuiqiushipin(qiubudong2).avi.avi",
    "zuiqiushipin(qiubudong3).avi.avi.avi",
    "zuiqiushipin2(2).avi.avi.avi.avi.avi.avi.avi",
    "zuiqiushipin2(3).avi.avi.avi.avi.avi.avi.avi.avi",
    "zuiqiushipin2(66).avi.avi.avi.avi.avi.avi.avi.avi.avi",
    "zuiqiushipin2(xian).avi.avi.avi.avi.avi",
]


def dhash(gray):
    small = cv2.resize(gray, (9, 8), interpolation=cv2.INTER_AREA)
    bits = (small[:, 1:] > small[:, :-1]).reshape(-1)
    h = 0
    for b in bits:
        h = (h << 1) | int(b)
    return h


def hamming(a, b):
    return bin(a ^ b).count("1")


def frame_score(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    edges = cv2.Canny(gray, 60, 160)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 60, minLineLength=80, maxLineGap=20)
    total = 0.0
    if lines is not None:
        for x1, y1, x2, y2 in np.reshape(lines, (-1, 4)):
            total += math.hypot(int(x2) - int(x1), int(y2) - int(y1))
    return sharp, total, dhash(gray)


def slug(name):
    keep = "".join(ch for ch in name if ch.isalnum() or ch == "_")
    return keep[:14]


def load_used(man_path):
    used = {}
    if not man_path.exists():
        return used
    with open(man_path, encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            used.setdefault(r["video"], set()).add(int(r["frame"]))
    return used


def read_manifest(path):
    with open(path, encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def cmd_extract(args):
    video_dir = Path(args.video_dir)
    if not video_dir.exists():
        raise SystemExit(f"视频目录不存在：{video_dir}")
    if args.videos:
        videos = args.videos.split("|")
    else:
        exts = {".avi", ".mp4", ".mov", ".mkv"}
        videos = sorted(p.name for p in video_dir.iterdir()
                        if p.is_file() and p.suffix.lower() in exts)
        if not videos:
            raise SystemExit(f"{video_dir} 里没有视频文件")
        print(f"发现 {len(videos)} 段视频：")
        for v in videos:
            print(f"  {v}")
    img_dir = DS / "images" / args.out_dir
    img_dir.mkdir(parents=True, exist_ok=True)
    used = load_used(DS / "manifest.csv")

    pool = {}
    for name in videos:
        path = video_dir / name
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            print(f"跳过（打不开）: {name}")
            continue
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if n <= 0:
            cap.release()
            continue
        step = max(1, n // args.samples)
        rows = []
        skip_frames = used.get(name, set())
        for i in range(0, n, step):
            if any(abs(i - u) < args.gap for u in skip_frames):
                continue
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, frame = cap.read()
            if not ok:
                continue
            sharp, lines, h = frame_score(frame)
            if sharp >= args.min_sharp and lines >= args.min_line:
                rows.append({"idx": i, "sharp": sharp, "lines": lines, "hash": h})
        cap.release()
        pool[name] = rows
        print(f"{name[:34]:36s} 抽样 {(n + step - 1) // step:4d}  合格 {len(rows):4d}")

    active = [k for k in pool if pool[k]]
    quota = args.total / max(len(active), 1)
    picked = []
    for name in active:
        rows = sorted(pool[name], key=lambda r: -r["lines"])
        want = int(round(quota))
        chosen = []
        for r in rows:
            if all(hamming(r["hash"], p["hash"]) >= 8 for p in picked + chosen):
                chosen.append(r)
            if len(chosen) >= want:
                break
        for r in chosen:
            r["video"] = name
        picked.extend(chosen)
        print(f"  取 {len(chosen)} 张")

    picked.sort(key=lambda r: (r["video"], r["idx"]))
    picked = picked[: args.total]

    manifest = []
    for r in picked:
        cap = cv2.VideoCapture(str(video_dir / r["video"]))
        cap.set(cv2.CAP_PROP_POS_FRAMES, r["idx"])
        ok, frame = cap.read()
        cap.release()
        if not ok:
            continue
        out = img_dir / f"{slug(r['video'])}_{r['idx']:05d}.jpg"
        cv2.imwrite(str(out), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        manifest.append({"image": out.name, "video": r["video"], "frame": r["idx"],
                         "sharp": round(r["sharp"], 1), "lines": round(r["lines"])})

    man = DS / args.manifest
    with open(man, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["image", "video", "frame", "sharp", "lines"])
        w.writeheader()
        w.writerows(manifest)
    print(f"\n共保存 {len(manifest)} 张到 {img_dir}")
    print("manifest:", man)
    return 0


def cmd_dataset(args):
    sources = [
        {"img": DS / "images" / "raw", "lab": DS / "labels_raw", "man": DS / "manifest.csv"},
        {"img": DS / "images" / "raw2", "lab": DS / "labels_raw2", "man": DS / "manifest2.csv"},
        {"img": DS / "images" / "raw3", "lab": DS / "labels_raw3", "man": DS / "manifest3.csv"},
        {"img": DS / "images" / "raw4", "lab": DS / "labels_raw4", "man": DS / "manifest4.csv"},
    ]
    items = []
    for src in sources:
        if not src["man"].exists():
            continue
        for r in read_manifest(src["man"]):
            stem = Path(r["image"]).stem
            img = src["img"] / r["image"]
            lab = src["lab"] / (stem + ".txt")
            if not img.exists():
                continue
            boxes = 0
            if lab.exists():
                boxes = sum(1 for l in lab.read_text(encoding="utf-8").splitlines() if l.strip())
            items.append({"img": img, "lab": lab if lab.exists() else None,
                          "video": r["video"], "stem": stem, "boxes": boxes})

    by_video = defaultdict(list)
    for it in items:
        by_video[it["video"]].append(it)

    total_boxes = sum(it["boxes"] for it in items)
    target = total_boxes * args.val_ratio
    order = sorted(by_video, key=lambda v: -sum(i["boxes"] for i in by_video[v]))
    val_videos, acc = [], 0
    for v in order:
        if acc >= target:
            break
        val_videos.append(v)
        acc += sum(i["boxes"] for i in by_video[v])

    for split in ("train", "val"):
        for sub in ("images", "labels"):
            d = DS / sub / split
            if d.exists():
                shutil.rmtree(d)
            d.mkdir(parents=True)

    stats = defaultdict(int)
    for v, group in by_video.items():
        split = "val" if v in val_videos else "train"
        for it in group:
            shutil.copy2(it["img"], DS / "images" / split / it["img"].name)
            dst = DS / "labels" / split / (it["stem"] + ".txt")
            if it["lab"] is not None:
                shutil.copy2(it["lab"], dst)
            else:
                dst.write_text("", encoding="utf-8")
            stats[split + "_img"] += 1
            stats[split + "_box"] += it["boxes"]

    print(f"train {stats['train_img']} 张 / {stats['train_box']} 框")
    print(f"val   {stats['val_img']} 张 / {stats['val_box']} 框")
    print("val 视频:")
    for v in val_videos:
        n = len(by_video[v])
        b = sum(i["boxes"] for i in by_video[v])
        print(f"  {v[:44]:46s} {n:3d} 张 {b:3d} 框")

    if stats["train_img"] == 0 and stats["val_img"] == 0:
        print("没有找到任何图片，data.yaml 不更新。")
        print("先把 dataset/images/{train,val} 放回来（交付版被移到外部备份目录 *_optional）")
        return 1

    yaml = DS / "marker_data.yaml"
    names = "\n".join(f"  {i}: {c}" for i, c in enumerate(NAMES))
    yaml.write_text(
        "path: " + str(DS).replace("\\", "/") + "\n"
        "train: images/train\n"
        "val: images/val\n"
        f"nc: {len(NAMES)}\n"
        "names:\n" + names + "\n",
        encoding="utf-8",
    )
    print("data.yaml ->", yaml)
    return 0


def line_width(gray, cx, cy):
    hsv = cv2.cvtColor(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 2] > 150) & (hsv[..., 1] < 60)).astype(np.uint8)
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    h, w = dist.shape
    x0, x1 = max(0, cx - 10), min(w, cx + 11)
    y0, y1 = max(0, cy - 10), min(h, cy + 11)
    return float(dist[y0:y1, x0:x1].max()) * 2.0


def cmd_normalize(args):
    total = changed = 0
    for split in ("train", "val"):
        img_dir = DS / "images" / split
        lab_dir = DS / "labels" / split
        if not lab_dir.exists():
            continue
        for lab in sorted(lab_dir.glob("*.txt")):
            lines = [l.split() for l in lab.read_text(encoding="utf-8").splitlines() if l.strip()]
            if not lines:
                continue
            img = cv2.imread(str(img_dir / (lab.stem + ".jpg")), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            h, w = img.shape
            out = []
            for cls, cx, cy, bw, bh in lines:
                cx, cy = float(cx) * w, float(cy) * h
                lw = line_width(img, int(round(cx)), int(round(cy)))
                side = float(np.clip(args.k * lw, args.min_side, args.max_side))
                out.append(f"{cls} {cx / w:.6f} {cy / h:.6f} {side / w:.6f} {side / h:.6f}")
                total += 1
                old_side = float(bw) * w
                if abs(old_side - side) / max(old_side, 1e-6) > 0.25:
                    changed += 1
            lab.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"共处理 {total} 个框，其中 {changed} 个尺寸改动超过 25%")
    return 0


def cmd_prelabel(args):
    from ultralytics import YOLO

    img_dir = DS / "images" / args.img_dir
    lab_dir = DS / "labels_raw2"
    lab_dir.mkdir(parents=True, exist_ok=True)
    (lab_dir / "classes.txt").write_text("\n".join(NAMES) + "\n", encoding="utf-8")

    model = YOLO(args.weights)
    images = sorted(img_dir.glob("*.jpg"))
    if not images:
        print(f"{img_dir} 里没有图片，先跑 extract")
        return 1

    counter = Counter()
    empty = 0
    for i, p in enumerate(images, 1):
        img = cv2.imread(str(p))
        h, w = img.shape[:2]
        res = model.predict(img, imgsz=args.imgsz, conf=args.conf, verbose=False, device=args.device)[0]
        lines = []
        for box in res.boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            cls = int(box.cls[0])
            cx = (x1 + x2) / 2 / w
            cy = (y1 + y2) / 2 / h
            lines.append(f"{cls} {cx:.6f} {cy:.6f} {(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}")
            counter[cls] += 1
        if not lines:
            empty += 1
        (lab_dir / (p.stem + ".txt")).write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        if i % 40 == 0:
            print(f"已处理 {i}/{len(images)}")
    print(f"\n图片 {len(images)} 张，预标注框 {sum(counter.values())} 个，空图 {empty} 张")
    for idx, name in enumerate(NAMES):
        if counter.get(idx):
            print(f"  {name:6s} {counter[idx]}")
    print("labels:", lab_dir)
    return 0


def cmd_flag(args):
    sys.path.insert(0, str(HERE))
    from live_localize import refine

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    sources = [(DS / "images" / "raw", DS / "labels_raw"),
               (DS / "images" / "raw2", DS / "labels_raw2")]
    rows, crops = [], []
    for img_dir, lab_dir in sources:
        if not lab_dir.exists():
            continue
        for lab in sorted(lab_dir.glob("*.txt")):
            if lab.name == "classes.txt":
                continue
            lines = [l.split() for l in lab.read_text(encoding="utf-8").splitlines() if l.strip()]
            targets = [l for l in lines if int(l[0]) == args.cls]
            if not targets:
                continue
            path = img_dir / (lab.stem + ".jpg")
            bgr = cv2.imread(str(path))
            if bgr is None:
                continue
            h, w = bgr.shape[:2]
            for idx, l in enumerate(targets):
                cx, cy = float(l[1]) * w, float(l[2]) * h
                bw, bh = float(l[3]) * w, float(l[4]) * h
                side = max(bw, bh)
                res = refine(bgr, cx, cy, side * 1.1)
                if res is None:
                    status, err, ndir = "no_line", 999.0, 0
                else:
                    u, v, ndir, _ = res
                    err = math.hypot(u - cx, v - cy)
                    status = "ok" if (ndir >= 2 and err <= side * 0.6) else "suspect"
                rows.append({"image": path.name, "label": lab.name, "box_index": idx,
                             "cx": round(cx, 1), "cy": round(cy, 1), "side": round(side, 1),
                             "err": round(min(err, 999), 1), "dirs": ndir, "status": status})
                if status != "ok" and len(crops) < 24:
                    s = int(max(side * 2.6, 120))
                    x0, y0 = max(0, int(cx) - s // 2), max(0, int(cy) - s // 2)
                    crop = bgr[y0:y0 + s, x0:x0 + s].copy()
                    if crop.size == 0:
                        continue
                    cv2.rectangle(crop, (int(cx) - x0 - int(bw / 2), int(cy) - y0 - int(bh / 2)),
                                  (int(cx) - x0 + int(bw / 2), int(cy) - y0 + int(bh / 2)), (0, 0, 255), 2)
                    crop = cv2.resize(crop, (300, 300))
                    cv2.putText(crop, f"{path.stem[-5:]} {status} e={min(err, 999):.0f}", (6, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                    crops.append(crop)

    ok = sum(1 for r in rows if r["status"] == "ok")
    print(f"框共 {len(rows)} 个：正常 {ok}，可疑 {len(rows) - ok}")
    for k in ("no_line", "suspect"):
        print(f"  {k}: {sum(1 for r in rows if r['status'] == k)}")
    out_csv = out_dir / "review.csv"
    with open(out_csv, "w", newline="", encoding="utf-8-sig") as f:
        w_ = csv.DictWriter(f, fieldnames=["image", "label", "box_index", "cx", "cy", "side", "err", "dirs", "status"])
        w_.writeheader()
        w_.writerows(rows)
    print("明细:", out_csv)
    while crops and len(crops) % 4:
        crops.append(np.zeros_like(crops[0]))
    if crops:
        rows_img = [np.hstack(crops[i:i + 4]) for i in range(0, len(crops), 4)]
        out = out_dir / "suspect.jpg"
        cv2.imwrite(str(out), np.vstack(rows_img))
        print("可疑样本图:", out)
    return 0


def cmd_xml2txt(args):
    """把 labelImg 存成 PascalVOC 的 .xml 转成 YOLO .txt。"""
    import xml.etree.ElementTree as ET

    lab_dir = DS / args.label_dir
    img_dir = DS / "images" / args.img_dir
    xmls = sorted(lab_dir.glob("*.xml"))
    if not xmls:
        print(f"{lab_dir} 里没有 .xml")
        return 0
    name2id = {n: i for i, n in enumerate(NAMES)}
    done = 0
    for x in xmls:
        try:
            root = ET.parse(x).getroot()
        except ET.ParseError as e:
            print(f"跳过（xml 坏了）：{x.name} {e}")
            continue
        size = root.find("size")
        w = h = None
        if size is not None:
            w = int(float(size.findtext("width", "0")))
            h = int(float(size.findtext("height", "0")))
        if not w or not h:
            img = cv2.imread(str(img_dir / (x.stem + ".jpg")))
            if img is None:
                print(f"跳过（拿不到尺寸）：{x.name}")
                continue
            h, w = img.shape[:2]
        lines = []
        for obj in root.findall("object"):
            name = (obj.findtext("name") or "").strip()
            if name not in name2id:
                print(f"跳过未知类别 {name!r}：{x.name}")
                continue
            bb = obj.find("bndbox")
            x1 = float(bb.findtext("xmin")); y1 = float(bb.findtext("ymin"))
            x2 = float(bb.findtext("xmax")); y2 = float(bb.findtext("ymax"))
            cx = (x1 + x2) / 2 / w
            cy = (y1 + y2) / 2 / h
            bw = abs(x2 - x1) / w
            bh = abs(y2 - y1) / h
            lines.append(f"{name2id[name]} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        (lab_dir / (x.stem + ".txt")).write_text(
            "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        done += 1
        if args.delete_xml:
            x.unlink()
    print(f"转换 {done} 个 xml -> txt" + ("，已删除 xml" if args.delete_xml else ""))
    return 0


def _data_main(argv=None):
    ap = argparse.ArgumentParser(description="标志点数据集工具合集")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("extract", help="从桌面视频抽标志帧")
    p.add_argument("--total", type=int, default=180)
    p.add_argument("--min-sharp", type=float, default=30.0)
    p.add_argument("--min-line", type=float, default=6000.0)
    p.add_argument("--samples", type=int, default=420)
    p.add_argument("--gap", type=int, default=45)
    p.add_argument("--video-dir", default=DEFAULT_VIDEO_DIR)
    p.add_argument("--videos", default="", help="用 | 分隔的视频名，默认用内置 13 段")
    p.add_argument("--out-dir", default="raw2")
    p.add_argument("--manifest", default="manifest2.csv")
    p.set_defaults(func=cmd_extract)

    p = sub.add_parser("dataset", help="合成 train/val")
    p.add_argument("--val-ratio", type=float, default=0.25)
    p.set_defaults(func=cmd_dataset)

    p = sub.add_parser("normalize", help="框统一成方块")
    p.add_argument("--k", type=float, default=3.0)
    p.add_argument("--min-side", type=float, default=20.0)
    p.add_argument("--max-side", type=float, default=110.0)
    p.set_defaults(func=cmd_normalize)

    p = sub.add_parser("prelabel", help="用已有模型预标注")
    p.add_argument("--weights", default=str(ROOT / "runs" / "marker_n640_v5" / "weights" / "best.pt"))
    p.add_argument("--conf", type=float, default=0.12)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--device", default="0")
    p.add_argument("--img-dir", default="raw2")
    p.set_defaults(func=cmd_prelabel)

    p = sub.add_parser("flag", help="挑出可疑框")
    p.add_argument("--cls", type=int, default=0, help="要检查的类别 id，默认 0=L")
    p.add_argument("--out", default=str(ROOT / "dataset" / "review"))
    p.set_defaults(func=cmd_flag)

    p = sub.add_parser("xml2txt", help="把 PascalVOC 的 xml 转成 YOLO txt")
    p.add_argument("--label-dir", default="labels_raw3")
    p.add_argument("--img-dir", default="raw3")
    p.add_argument("--delete-xml", action="store_true")
    p.set_defaults(func=cmd_xml2txt)

    args = ap.parse_args(argv)
    raise SystemExit(args.func(args))

# ======================================================================
# 三、场地图：按实测尺寸画 field_map.png + json
# ======================================================================



HALF_L = 3.90
HALF_W = 3.00
PEN_X = HALF_L - 1.90
PEN_HALF_W = 2.00
GOAL_X = HALF_L - 0.94
GOAL_HALF_W = 1.35

PX_PER_M = 110
MARGIN_M = 3.20
MARGIN_TOP = 3.20

LINE = (40, 40, 40)
BG = (255, 255, 255)


def to_px(x, y, ox, oy, s):
    return int(round(ox - y * s)), int(round(oy - x * s))


def _map_main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--r", type=float, default=0.75, help="中圈半径（米）")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "config" / "field_map_generated.png"),
                    help="默认写 field_map_generated.png，不会覆盖交付用的 field_map.png")
    args = ap.parse_args(argv)

    s = PX_PER_M
    w = int((2 * HALF_W + 2 * MARGIN_M) * s)
    h = int((2 * HALF_L + MARGIN_TOP + MARGIN_M) * s + 30)
    ox = w // 2
    oy = int((HALF_L + MARGIN_TOP) * s)

    img = np.full((h, w, 3), BG, np.uint8)
    th = 3

    def line(a, b, t=th):
        cv2.line(img, to_px(a[0], a[1], ox, oy, s), to_px(b[0], b[1], ox, oy, s), LINE, t, cv2.LINE_AA)

    def rect(x0, x1, y0, y1):
        line((x0, y0), (x0, y1))
        line((x0, y1), (x1, y1))
        line((x1, y1), (x1, y0))
        line((x1, y0), (x0, y0))

    for sx in (-1, 1):
        rect(HALF_L * sx, PEN_X * sx, -PEN_HALF_W, PEN_HALF_W)
        rect(HALF_L * sx, GOAL_X * sx, -GOAL_HALF_W, GOAL_HALF_W)

    rect(-HALF_L, HALF_L, -HALF_W, HALF_W)
    line((0, -HALF_W - 0.28), (0, HALF_W + 0.28), th)
    cv2.circle(img, to_px(0, 0, ox, oy, s), int(round(args.r * s)), LINE, th, cv2.LINE_AA)

    colors = {0: (255, 200, 0), 1: (0, 190, 235), 2: (230, 0, 230), 3: (0, 130, 255)}
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts.append((0, HALF_L * sx, HALF_W * sy))
            pts.append((0, PEN_X * sx, PEN_HALF_W * sy))
            pts.append((0, GOAL_X * sx, GOAL_HALF_W * sy))
            pts.append((1, HALF_L * sx, PEN_HALF_W * sy))
            pts.append((1, HALF_L * sx, GOAL_HALF_W * sy))
    for sy in (-1, 1):
        pts.append((1, 0.0, HALF_W * sy))
    if args.r:
        pts.append((2, args.r, 0.0))
        pts.append((2, -args.r, 0.0))
    pts.append((3, 0.0, 0.0))

    for cls, x, y in pts:
        c = to_px(x, y, ox, oy, s)
        cv2.circle(img, c, 5, colors[cls], -1, cv2.LINE_AA)

    cv2.arrowedLine(img, (24, 78), (24, 22), (0, 0, 255), 2, tipLength=0.28)
    cv2.putText(img, "+X", (30, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    cv2.arrowedLine(img, (w - 20, 24), (w - 78, 24), (0, 0, 255), 2, tipLength=0.28)
    cv2.putText(img, "+Y", (w - 78, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    legend = [("L", colors[0]), ("T", colors[1]), ("X", colors[2]), ("Point", colors[3])]
    ly = h - 12
    lx = 12
    for text, col in legend:
        cv2.circle(img, (lx, ly - 5), 5, col, -1, cv2.LINE_AA)
        cv2.putText(img, text, (lx + 12, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.45, LINE, 1)
        lx += 74

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.chmod(0o666)
    cv2.imwrite(str(out), img)
    meta = {
        "px_per_m": PX_PER_M,
        "origin_px": [ox, oy],
        "axis": "x_up_y_left",
        "field_m": [2 * HALF_L, 2 * HALF_W],
        "size": [w, h],
    }
    out.with_suffix(".json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(out, img.shape, "origin", (ox, oy))


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print(__doc__)
        return 0
    group, rest = argv[0], argv[1:]
    table = {"calib": _calib_main, "data": _data_main, "map": _map_main}
    if group not in table:
        print("未知分组：%s" % group)
        print(__doc__)
        return 2
    return table[group](rest)


if __name__ == "__main__":
    raise SystemExit(main())
