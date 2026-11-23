"""相机内参与畸变标定（任务二 2.5 节「必须完成」项）。

为什么要用它：
  考核细则 2.5 明确要求用棋盘格/ChArUco 完成实际内参标定，至少 15 张有效图像，
  覆盖画面中心、四角、不同倾斜角度和距离；标定分辨率必须与推理分辨率一致（1280x720）。
  「固定焦距」不等于「内参已知」，镜头标称视场角不能替代标定结果。

三个子命令：
  capture    实时抓图（只有检测到棋盘格的帧才允许保存），带角点实时预览
  calibrate  从图片目录标定，输出 config/camera_intrinsics.yaml + 去畸变对比图
  selftest   用虚拟相机验证标定算法本身正确（不需要相机、不需要打印板子）

用法：
  # 0) 先做一次算法自检，确认脚本在你的环境里是对的（约 20 秒）
  python tools/calibrate_camera.py selftest

  # 1) 对着棋盘格抓 20 张（建议 18~25 张，少于 15 张会被警告）
  python tools/calibrate_camera.py capture --index 1 --out calibration/raw

  # 2) 标定
  python tools/calibrate_camera.py calibrate --images calibration/raw

  # 3) 目视检查去畸变效果
  python tools/calibrate_camera.py undistort --image calibration/raw/calib_001.jpg

抓图时按键：
  SPACE  保存当前帧（必须先识别到完整角点，否则拒绝保存）
  a      自动模式：每 0.8 秒自动保存一张（角点稳定且与上一张姿态差异足够时）
  q/ESC  退出

棋盘格口径（最容易搞错的一点）：
  --cols / --rows 是「内角点」数量，不是方格数量。
  A4 上打印 10x7 个方格 => 内角点是 9x6 => --cols 9 --rows 6
  用 tools/make_checkerboard.py 生成的板子，标题里会直接写明两者对应关系。
"""
import argparse
import datetime as dt
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CALIB_DIR = ROOT / "calibration"
OUT_YAML = ROOT / "config" / "camera_intrinsics.yaml"
CHECK_DIR = ROOT / "check_out"
IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")


# --------------------------------------------------------------------------- #
# 公共工具
# --------------------------------------------------------------------------- #
def resolve_backend(name: str):
    """auto：Windows 用 DSHOW（FFMPEG 后端枚举不到摄像头），Linux 用 V4L2。"""
    table = {
        "any": cv2.CAP_ANY,
        "dshow": cv2.CAP_DSHOW,
        "msmf": cv2.CAP_MSMF,
        "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
    }
    if name != "auto":
        return table[name]
    if sys.platform == "win32":
        return table["dshow"]
    if sys.platform.startswith("linux"):
        return table["v4l2"]
    return cv2.CAP_ANY


def object_points(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """板子上每个内角点在「板坐标系」中的 3D 坐标（Z=0，单位 mm）。"""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_mm)
    return objp


def find_corners(gray: np.ndarray, cols: int, rows: int):
    """先试 SB 检测器（自带亚像素），失败再退回经典检测器 + cornerSubPix。"""
    pattern = (cols, rows)

    ok, corners = cv2.findChessboardCornersSB(
        gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        return True, corners.astype(np.float32), "SB"

    ok, corners = cv2.findChessboardCornersSB(
        gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if ok:
        return True, corners.astype(np.float32), "SB-exhaustive"

    ok, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return False, None, "-"
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners.astype(np.float32), "classic"


def imread_any(path: Path):
    """cv2.imread 不支持中文路径，统一用 imdecode。"""
    buf = np.fromfile(str(path), dtype=np.uint8)
    if buf.size == 0:
        return None
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def imwrite_any(path: Path, img: np.ndarray) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(path.suffix or ".jpg", img)
    if not ok:
        return False
    buf.tofile(str(path))
    return True


# --------------------------------------------------------------------------- #
# 1. capture：实时抓图
# --------------------------------------------------------------------------- #
def cmd_capture(args):
    RAW = Path(args.out)
    if not RAW.is_absolute():
        RAW = ROOT / RAW
    RAW.mkdir(parents=True, exist_ok=True)

    backend = resolve_backend(args.backend)
    cap = cv2.VideoCapture(args.index, backend)
    if not cap.isOpened():
        sys.exit(f"错误：打不开摄像头 index={args.index} backend={args.backend}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (aw, ah) != (args.width, args.height):
        print(f"⚠ 请求 {args.width}x{args.height}，实际 {aw}x{ah}。")
        print("  标定分辨率必须与推理分辨率一致，否则内参不可直接复用！")

    existing = len([p for p in RAW.iterdir() if p.suffix.lower() in IMG_EXT])
    print(f"输出目录 : {RAW}（已有 {existing} 张）")
    print(f"板子口径 : 内角点 {args.cols} 列 x {args.rows} 行，方格 {args.square} mm")
    print(f"图像尺寸 : {aw}x{ah}")
    print("\n操作：SPACE 保存 | a 自动连拍 | q/ESC 退出")
    print("要求：≥15 张，覆盖画面中心/四角、不同倾斜角(±30°)、不同距离\n")

    saved = 0
    last_stamp = 0.0
    last_center = None
    while True:
        ok, frame = cap.read()
        if not ok:
            print("取帧失败，退出")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners, how = find_corners(gray, args.cols, args.rows)

        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, (args.cols, args.rows), corners, found)
            c = corners.reshape(-1, 2).mean(axis=0)
            cv2.putText(view, f"OK  corners={len(corners)}  ({how})", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
            cv2.putText(view, f"center=({c[0]:.0f},{c[1]:.0f})", (12, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        else:
            c = None
            cv2.putText(view, "NO BOARD - move/tilt the board", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(view, f"saved={saved}  {aw}x{ah}", (12, ah - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if args.no_preview:
            cv2.imshow("calib capture", cv2.resize(view, (aw // 2, ah // 2)))
        else:
            cv2.imshow("calib capture", view)

        key = cv2.waitKey(1) & 0xFF
        quit_now = key in (ord("q"), 27)

        want_save = key == ord(" ")
        if key == ord("a"):
            args.auto = True
            print("自动连拍已开启：每 0.8 秒存一张（姿态变化不足会自动跳过）")

        if args.auto and found and not want_save:
            now = cv2.getTickCount() / cv2.getTickFrequency()
            moved = True
            if last_center is not None:
                moved = np.linalg.norm(c - last_center) > 25
            if now - last_stamp > 0.8 and moved:
                want_save = True
                last_center = c
                last_stamp = now

        if want_save:
            if not found:
                print("✗ 未识别到完整棋盘格，拒绝保存（避免污染标定集）")
            else:
                saved += 1
                out = RAW / f"calib_{saved:03d}.jpg"
                imwrite_any(out, frame)
                print(f"✓ 保存 {out.name}  ({saved} 张)")

        if quit_now:
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n本次共保存 {saved} 张到 {RAW}")
    if existing + saved < 15:
        print("⚠ 总数不足 15 张，考核要求「至少 15 张有效标定图像」")


# --------------------------------------------------------------------------- #
# 2. calibrate：标定
# --------------------------------------------------------------------------- #
def run_calibration(files, objp, cols, rows, prune):
    """返回 (rms, K, dist, used_files, per_view_err, ok_corners)。"""
    obj_points, img_points, names, shapes = [], [], [], []
    for p in files:
        img = imread_any(p)
        if img is None:
            print(f"  跳过（无法读取）{p.name}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners, how = find_corners(gray, cols, rows)
        if not ok:
            print(f"  跳过（未检出角点）{p.name}")
            continue
        obj_points.append(objp.copy())
        img_points.append(corners)
        names.append(p.name)
        shapes.append(gray.shape[::-1])
        print(f"  ✓ {p.name:24s} 角点 {len(corners):3d}  ({how})")

    if len(obj_points) < 5:
        sys.exit(f"错误：有效图像只有 {len(obj_points)} 张，至少需要 5 张（考核建议 15 张以上）")

    sizes = sorted(set(shapes))
    if len(sizes) > 1:
        sys.exit(f"错误：图像尺寸不一致 {sizes}。标定与推理必须是同一分辨率，请先统一再标定。")
    w, h = sizes[0]

    flags = 0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    def _calib(idx):
        rms, K, dist, rv, tv = cv2.calibrateCamera(
            [obj_points[i] for i in idx], [img_points[i] for i in idx],
            (w, h), None, None, flags=flags, criteria=criteria)
        errs = []
        for i in idx:
            proj, _ = cv2.projectPoints(obj_points[i], rv[idx.index(i)], tv[idx.index(i)], K, dist)
            e = np.linalg.norm(img_points[i].reshape(-1, 2) - proj.reshape(-1, 2), axis=1).mean()
            errs.append((names[i], float(e)))
        return rms, K, dist, errs, rv, tv

    idx = list(range(len(obj_points)))
    rms, K, dist, errs, rv, tv = _calib(idx)

    # 剔除明显失败的图（未对焦 / 板子弯了 / 角点错位），再标一次
    dropped = []
    if prune > 0:
        while True:
            bad = [n for n, e in errs if e > prune]
            if not bad or len(idx) - len(bad) < 10:
                break
            keep = [i for i in idx if names[i] not in bad]
            dropped += [(n, e) for n, e in errs if n in bad]
            idx = keep
            rms, K, dist, errs, rv, tv = _calib(idx)
    return rms, K, dist, [names[i] for i in idx], errs, dropped, (w, h), rv


def pose_diversity(rv):
    """评估棋盘格姿态多样性（防病态标定的关键指标）。

    返回 (tilts, azimuths)：每张图的棋盘法线与相机光轴的夹角（度，0=正对），
    以及法线水平投影的方位角（度）。病态标定的典型特征：倾角范围 <15°
    且方位角挤在一个窄扇区（比如全程贴墙拍，只变距离不变朝向）。
    """
    tilts, azimuths = [], []
    for r in rv:
        R, _ = cv2.Rodrigues(np.asarray(r))
        n = R @ np.array([0.0, 0.0, 1.0])
        if n[2] < 0:
            n = -n
        tilts.append(float(np.degrees(np.arccos(np.clip(n[2], -1, 1)))))
        azimuths.append(float(np.degrees(np.arctan2(n[1], n[0]))))
    return tilts, azimuths


def _circular_range(angles_deg):
    """角度列表的圆上分布范围（360 - 最大空隙），单位度。"""
    if len(angles_deg) < 2:
        return 0.0
    a = np.sort(np.mod(angles_deg, 360.0))
    gaps = np.diff(np.concatenate([a, a[:1] + 360.0]))
    return float(360.0 - gaps.max())


def cmd_calibrate(args):
    src = Path(args.images)
    if not src.is_absolute():
        src = ROOT / src
    files = sorted(p for p in src.iterdir() if p.suffix.lower() in IMG_EXT)
    if not files:
        sys.exit(f"错误：{src} 里没有图片。先跑 capture 子命令抓图。")

    print(f"读取 {len(files)} 张图片：{src}")
    objp = object_points(args.cols, args.rows, args.square)
    rms, K, dist, used, errs, dropped, (w, h), rv = run_calibration(
        files, objp, args.cols, args.rows, args.prune)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    dist = np.asarray(dist).ravel()
    # 统一成 OpenCV 5 参数顺序 [k1,k2,p1,p2,k3]
    if dist.size == 4:
        dist = np.array([dist[0], dist[1], dist[2], dist[3], 0.0])
    elif dist.size > 5:
        dist = dist[:5]

    print("\n" + "=" * 62)
    print(f"有效图像 : {len(used)} / {len(files)}   (剔除 {len(dropped)} 张高误差图)")
    for n, e in dropped:
        print(f"   剔除 {n}  单图重投影 {e:.3f} px")
    print(f"图像尺寸 : {w} x {h}")
    print(f"fx = {fx:8.3f}   fy = {fy:8.3f}")
    print(f"cx = {cx:8.3f}   cy = {cy:8.3f}")
    print(f"畸变 [k1,k2,p1,p2,k3] = {np.array2string(dist, precision=5)}")
    print(f"整体重投影 RMS = {rms:.4f} px")
    print(f"单图误差 最小/中位/最大 = {min(e for _, e in errs):.3f} / "
          f"{np.median([e for _, e in errs]):.3f} / {max(e for _, e in errs):.3f} px")

    # ---- 病态标定体检（2026-11-10 实战踩坑后新增）----
    tilts, azims = pose_diversity(rv)
    tilt_range = max(tilts) - min(tilts)
    az_range = _circular_range(azims)
    problems = []
    if abs(fx / fy - 1) > 0.02:
        problems.append(f"fx/fy = {fx/fy:.3f}（方形像素应≈1，偏离说明拟合病态）")
    if not (0 <= cx <= w and 0 <= cy <= h):
        problems.append(f"主点 ({cx:.0f},{cy:.0f}) 跑到画面外，明显异常")
    if tilt_range < 15.0:
        problems.append(f"棋盘倾角范围仅 {tilt_range:.1f}°（应≥15°）——20 张几乎同一个朝向")
    if az_range < 45.0:
        problems.append(f"棋盘旋转方向只覆盖 {az_range:.0f}° 扇区（应≥45°）——缺少不同方向的倾斜")
    print("-" * 62)
    print(f"姿态多样性 : 倾角 {min(tilts):.1f}°~{max(tilts):.1f}°（范围 {tilt_range:.1f}°）"
          f" | 方位覆盖 {az_range:.0f}° | {'健康' if not problems else '不足'}")
    if problems:
        print("⚠ 病态标定体检发现问题：")
        for p in problems:
            print(f"   - {p}")
        print("   重拍要点：棋盘拿在手里，每张图换朝向——正对/上仰/下俯/左歪/右歪/对角，")
        print("   再配合不同距离(20~50cm)和画面位置(中心/四角)。只变距离不变朝向必翻车。")
    if rms < 0.5:
        print("评价：优秀（RMS < 0.5 px）")
    elif rms < 1.0:
        print("评价：可用（RMS < 1.0 px）")
    else:
        print("评价：偏差偏大，建议重拍：板子要平整、避免运动模糊、多覆盖四角")
    print("=" * 62)

    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out
    data = {
        "#": "相机内参与畸变系数（由 tools/calibrate_camera.py 生成，勿手改数值）",
        "camera_name": args.camera_name,
        "image_width": int(w),
        "image_height": int(h),
        "camera_matrix": {"rows": 3, "cols": 3,
                          "data": [round(float(v), 6) for v in K.ravel()]},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5,
                                    "data": [round(float(v), 8) for v in dist]},
        "rms_reprojection_error": round(float(rms), 4),
        "num_images_used": len(used),
        "num_images_total": len(files),
        "board": {"pattern_cols": args.cols, "pattern_rows": args.rows,
                  "square_size_mm": float(args.square)},
        "per_view_errors_px": {n: round(e, 4) for n, e in errs},
        "calibrated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "calibrated_host": sys.platform,
        "note": "分辨率改变、裁剪或缩放后必须重新标定或同步换算内参。",
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"\n已写入 {out}")

    # 去畸变对比图：最直观的“标定到底对不对”检查
    sample = used[0]
    img = imread_any(src / sample)
    undist = cv2.undistort(img, K, dist)
    diff = cv2.absdiff(img, undist)
    side = np.hstack([img, undist])
    try:
        cv2.putText(side, "raw", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        cv2.putText(side, "undistorted (straight lines must be straight)",
                    (w + 12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    except cv2.error:
        pass
    CHECK_DIR.mkdir(parents=True, exist_ok=True)
    cmp_path = CHECK_DIR / "calib_undistort_compare.jpg"
    imwrite_any(cmp_path, side)
    print(f"去畸变对比图 -> {cmp_path}")
    print(f"                该图平均像素差 {diff.mean():.2f}（过小说明畸变很小或标定没生效）")


# --------------------------------------------------------------------------- #
# 3. undistort：单独看去畸变效果
# --------------------------------------------------------------------------- #
def cmd_undistort(args):
    cfg = Path(args.yaml)
    if not cfg.is_absolute():
        cfg = ROOT / cfg
    if not cfg.exists():
        sys.exit(f"错误：找不到 {cfg}，先跑 calibrate")
    d = yaml.safe_load(cfg.read_text(encoding="utf-8"))
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(d["distortion_coefficients"]["data"], dtype=np.float64)

    if args.image:
        src = Path(args.image)
        if not src.is_absolute():
            src = ROOT / src
    else:
        raw_dir = ROOT / "calibration" / "raw"
        cands = sorted(p for p in raw_dir.iterdir()
                       if p.suffix.lower() in IMG_EXT) if raw_dir.is_dir() else []
        if not cands:
            sys.exit("错误：没传 --image，且 calibration/raw 里没有图片")
        src = cands[0]
        print(f"未指定 --image，默认取 {src}")

    img = imread_any(src)
    if img is None:
        sys.exit(f"错误：读不到图片 {src}")
    h, w = img.shape[:2]
    if (w, h) != (d["image_width"], d["image_height"]):
        print(f"⚠ 图片 {w}x{h} 与标定尺寸 {d['image_width']}x{d['image_height']} 不一致，结果仅供参考")

    undist = cv2.undistort(img, K, dist)
    side = np.hstack([img, undist])
    out = CHECK_DIR / f"undistort_{src.stem}.jpg"
    imwrite_any(out, side)
    print(f"对比图 -> {out}")


# --------------------------------------------------------------------------- #
# 4. selftest：虚拟相机验证算法正确性
# --------------------------------------------------------------------------- #
def render_synthetic_board(K, dist, R, t, cols, rows, square_mm, size):
    """用逆映射渲染一张「带镜头畸变」的棋盘格图。

    做法：对每个像素，先去畸变得到归一化坐标 -> 射线 -> 与板平面(Z=0)求交
    -> 得到板坐标(a,b) -> 解析判断落在黑格还是白格。这样畸变是真实渲染进去的，
    标定时若能还原出 K 和 dist，说明整条数学链路（含去畸变）是对的。
    """
    w, h = size[0], size[1]
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float32),
                         np.arange(h, dtype=np.float32))
    pts = np.stack([uu, vv], axis=-1).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pts, K, dist).reshape(-1, 2)

    d = np.concatenate([norm, np.ones((norm.shape[0], 1), np.float32)], axis=1).astype(np.float64)
    n = R[:, 2].reshape(3)
    denom = d @ n
    s = float(n @ t) / np.where(np.abs(denom) < 1e-9, 1e-9, denom)
    valid = (np.abs(denom) > 1e-9) & (s > 0)

    P = d * s[:, None]
    local = (P - t) @ R          # 等价 R^T (P-t)
    a, b = local[:, 0], local[:, 1]

    board_w = cols * square_mm
    board_h = rows * square_mm
    inside = valid & (a >= 0) & (a < board_w) & (b >= 0) & (b < board_h)

    ia = np.floor(a / square_mm).astype(np.int32)
    ib = np.floor(b / square_mm).astype(np.int32)
    black = ((ia + ib) % 2 == 1)

    img = np.full((h * w,), 255, np.uint8)
    img[inside & black] = 20
    img = img.reshape(h, w)
    img = cv2.GaussianBlur(img, (3, 3), 0.6)
    noise = np.random.default_rng(0).normal(0, 2.0, img.shape)
    img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def build_view(i, cols, rows, square_mm, w, h, fx):
    """构造一组「把板子放进画面」的位姿：距离、俯仰/偏航/滚转都不同。"""
    rng = np.random.default_rng(1000 + i)
    board_w, board_h = cols * square_mm, rows * square_mm
    center = np.array([board_w / 2, board_h / 2, 0.0])

    # 注意单位：板坐标与方格边长都是毫米，距离也必须用毫米，混用会得到荒谬的位姿
    Z = (0.45 + 0.05 * (i % 6)) * 1000.0            # 450 ~ 700 mm
    ax = np.deg2rad(rng.uniform(-32, 32))           # 绕相机 X（俯仰）
    ay = np.deg2rad(rng.uniform(-32, 32))           # 绕相机 Y（偏航）
    az = np.deg2rad(rng.uniform(-25, 25))           # 绕光轴（滚转）

    def Rx(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def Ry(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def Rz(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    R0 = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])  # 板面朝向相机
    R = Rz(az) @ Rx(ax) @ Ry(ay) @ R0
    t = np.array([0, 0, Z], float) - R @ center
    return R, t


def cmd_selftest(args):
    print("虚拟相机自检：已知真值 -> 渲染带畸变图像 -> 走真实标定流程 -> 比对")
    w, h = 1280, 720
    cols, rows = args.cols, args.rows          # 内角点
    square_mm = args.square
    sq_cols, sq_rows = cols + 1, rows + 1      # 方格数

    fx_true = 900.0
    K_true = np.array([[fx_true, 0, w / 2 - 8], [0, fx_true, h / 2 + 5], [0, 0, 1]], float)
    dist_true = np.array([0.12, -0.25, 0.0008, -0.0005, 0.08], float)

    tmp = CALIB_DIR / "_selftest"
    tmp.mkdir(parents=True, exist_ok=True)
    for p in tmp.glob("*.png"):
        p.unlink()

    n_views = args.views
    print(f"  生成 {n_views} 个视角（板 {sq_cols}x{sq_rows} 方格，方格 {square_mm}mm，"
          f"内角点 {cols}x{rows}）")
    made = 0
    for i in range(n_views):
        R, t = build_view(i, sq_cols, sq_rows, square_mm, w, h, fx_true)
        img = render_synthetic_board(K_true, dist_true, R, t, sq_cols, sq_rows, square_mm, (w, h))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, _, _ = find_corners(gray, cols, rows)
        if not ok:
            print(f"  视角 {i:02d} 未检出角点，跳过（虚拟渲染的边界情况）")
            continue
        made += 1
        imwrite_any(tmp / f"syn_{made:03d}.png", img)
    print(f"  实际可用 {made} 张 -> {tmp}")

    if made < 8:
        sys.exit("自检失败：可检出视角太少，渲染参数有问题")

    objp = object_points(cols, rows, square_mm)
    files = sorted(tmp.glob("*.png"))
    rms, K, dist, used, errs, dropped, (rw, rh), _rv = run_calibration(
        files, objp, cols, rows, 1.5)

    dist = np.asarray(dist).ravel()
    if dist.size == 4:
        dist = np.array([dist[0], dist[1], dist[2], dist[3], 0.0])

    print("\n" + "=" * 62)
    print("真值 vs 估计：")
    print(f"  fx  {fx_true:9.3f} -> {K[0,0]:9.3f}  偏差 {abs(K[0,0]-fx_true)/fx_true*100:6.3f}%")
    print(f"  fy  {fx_true:9.3f} -> {K[1,1]:9.3f}  偏差 {abs(K[1,1]-fx_true)/fx_true*100:6.3f}%")
    print(f"  cx  {K_true[0,2]:9.3f} -> {K[0,2]:9.3f}  偏差 {abs(K[0,2]-K_true[0,2]):6.3f} px")
    print(f"  cy  {K_true[1,2]:9.3f} -> {K[1,2]:9.3f}  偏差 {abs(K[1,2]-K_true[1,2]):6.3f} px")
    print(f"  dist 真值 {np.array2string(dist_true, precision=4)}")
    print(f"  dist 估计 {np.array2string(dist, precision=4)}")
    print(f"  RMS = {rms:.4f} px   使用 {len(used)} 张")

    err_fx = abs(K[0, 0] - fx_true) / fx_true
    err_cx = abs(K[0, 2] - K_true[0, 2])
    ok = err_fx < 0.01 and err_cx < 3.0 and rms < 1.0
    print("=" * 62)
    print(("✅ 自检通过：标定数学链路正确，等相机到了直接抓图即可" if ok
           else "❌ 自检未通过：偏差过大，先别用真实数据标定"))
    return 0 if ok else 1


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description="相机内参/畸变标定（棋盘格），含虚拟相机自检",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add_board(p):
        p.add_argument("--cols", type=int, default=9, help="内角点列数（默认 9）")
        p.add_argument("--rows", type=int, default=6, help="内角点行数（默认 6）")
        p.add_argument("--square", type=float, default=20.0, help="方格边长 mm（默认 20）")

    p1 = sub.add_parser("capture", help="实时抓图")
    p1.add_argument("--index", type=int, default=0, help="摄像头索引（/dev/video0 -> 0）")
    p1.add_argument("--backend", default="auto", choices=["auto", "any", "dshow", "msmf", "v4l2"])
    p1.add_argument("--width", type=int, default=1280)
    p1.add_argument("--height", type=int, default=720)
    p1.add_argument("--out", default="calibration/raw")
    p1.add_argument("--no-preview", action="store_true", help="窗口缩小一半，低性能机器用")
    p1.add_argument("--auto", action="store_true", help="启动即开自动连拍")
    add_board(p1)
    p1.set_defaults(func=cmd_capture)

    p2 = sub.add_parser("calibrate", help="从图片目录标定")
    p2.add_argument("--images", default="calibration/raw")
    p2.add_argument("--out", default="config/camera_intrinsics.yaml")
    p2.add_argument("--camera-name", default="mono_usb_camera")
    p2.add_argument("--prune", type=float, default=1.5,
                    help="单图重投影误差超过该值(px)则剔除后重标；0 关闭")
    add_board(p2)
    p2.set_defaults(func=cmd_calibrate)

    p3 = sub.add_parser("undistort", help="生成去畸变对比图")
    p3.add_argument("--image", default=None,
                    help="要目检的图；不传则自动用 calibration/raw 里第一张")
    p3.add_argument("--yaml", default="config/camera_intrinsics.yaml")
    p3.set_defaults(func=cmd_undistort)

    p4 = sub.add_parser("selftest", help="虚拟相机自检（不需要相机）")
    p4.add_argument("--views", type=int, default=16, help="生成多少个视角（默认 16）")
    add_board(p4)
    p4.set_defaults(func=cmd_selftest)

    args = ap.parse_args()
    sys.exit(args.func(args) or 0)


if __name__ == "__main__":
    main()
