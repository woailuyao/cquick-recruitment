"""任务二 · 第 1 步：相机内参标定（棋盘格）

为什么要做：像素坐标 (u,v) 要变成真正的"米"，必须先知道相机的 fx/fy/cx/cy
和镜头畸变 k1/k2/p1/p2/k3。这一步不做好，后面所有距离都会整体偏。

用法（在仓库根目录执行）：

    # 1) 让脚本自己猜棋盘格是几乘几（推荐先跑这个）
    python calibration/calibrate_intrinsics.py --images data/calib/intrinsics --auto-size

    # 2) 如果你已经知道内角点数量（比如 9x6 = 横向 9 个角、纵向 6 个角）
    python calibration/calibrate_intrinsics.py --images data/calib/intrinsics --cols 9 --rows 6 --square 25

    # 3) 拍完发现有几张糊了/歪了，可以自动剔除误差最大的图
    python calibration/calibrate_intrinsics.py --images data/calib/intrinsics --auto-size --max-rms 0.8

注意：
  * 棋盘格的"内角点"= 黑白格交点的个数。9x6 的板子，格子数是 10x7。
  * --square 是一个格子的真实边长，单位毫米（拿尺子量，别用标称值）。
  * 图片分辨率必须和你最终跑定位时的分辨率一致（考核要求 1280x720）。
"""

import argparse
import os
import sys

import cv2
import numpy as np

def _repo_root():
    """向上找到包含 config/field.yaml 的目录，作为仓库根。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        if os.path.exists(os.path.join(d, "config", "field.yaml")):
            return d
        d = os.path.dirname(d)
    return os.getcwd()

ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision2.console import setup_console  # noqa: E402
from vision2.io_utils import imread, imwrite, list_images  # noqa: E402
from vision2.yaml_mini import save_yaml  # noqa: E402

setup_console()


def _say(msg=""):
    print(msg, flush=True)


def _criteria():
    return (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)


def detect_corners(gray, size, fast=False):
    """在灰度图里找棋盘格内角点，返回 (ok, corners[N,1,2])。

    fast=True 用最省的参数，只用来"扫尺寸"，不适合最终标定。
    """
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    if not fast:
        if hasattr(cv2, "CALIB_CB_EXHAUSTIVE"):
            flags |= cv2.CALIB_CB_EXHAUSTIVE
        if hasattr(cv2, "CALIB_CB_ACCURACY"):
            flags |= cv2.CALIB_CB_ACCURACY
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, size, flags)
        if ok:
            return True, corners
    if fast:
        return False, None
    ok, corners = cv2.findChessboardCorners(
        gray,
        size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not ok:
        return False, None
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _criteria())
    return True, corners


def _ideal_grid(cols, rows):
    """理想棋盘格角点坐标（单位可以任意，只用来看结构是否自洽）。"""
    p = np.zeros((cols * rows, 2), np.float32)
    k = 0
    for j in range(rows):
        for i in range(cols):
            p[k] = (i, j)
            k += 1
    return p


def lattice_error(corners, size, tol=2.0):
    """检查检出的点是不是一个规整的 (cols x rows) 方格网。

    用单应变换拟合：如果尺寸猜对了，误差只有零点几像素；
    猜错了（把 9x6 当 6x9），点会被排错顺序，误差会大到几十像素。
    """
    pts = corners.reshape(-1, 2).astype(np.float32)
    if len(pts) != size[0] * size[1]:
        return 1e9
    ideal = _ideal_grid(*size)
    Hm, _ = cv2.findHomography(ideal, pts, 0)
    if Hm is None:
        return 1e9
    proj = cv2.perspectiveTransform(ideal.reshape(-1, 1, 2), Hm).reshape(-1, 2)
    return float(np.mean(np.linalg.norm(proj - pts, axis=1)))


def guess_pattern(grays, sample_n=3, scan_width=640):
    """猜棋盘格内角点尺寸。

    做法：把图缩小一半，对一批候选尺寸逐个检测，再用「方格网自洽性」筛掉误检，
    在通过的候选里挑角点最多的那个（角点越多，标定覆盖越好）。
    """
    candidates = []
    for cols in range(4, 15):
        for rows in range(4, 16):
            if cols * rows < 20:
                continue
            candidates.append((cols, rows))

    sample = grays[: min(sample_n, len(grays))]
    scan, scales = [], []
    for g in sample:
        w = g.shape[1]
        s = min(1.0, scan_width / float(w))
        scan.append(cv2.resize(g, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s < 1.0 else g)
        scales.append(s)

    scored = []
    for cols, rows in candidates:
        errs, hits = [], 0
        for g, s in zip(scan, scales):
            ok, c = detect_corners(g, (cols, rows), fast=True)
            if not ok:
                continue
            e = lattice_error(c, (cols, rows))
            if e < 1.5 * max(s, 1e-6):
                hits += 1
                errs.append(e / max(s, 1e-6))
        if errs:
            scored.append((hits, cols * rows, (cols, rows), float(np.mean(errs))))

    # 先看角点数（越大越可能是真棋盘格），再看命中数
    scored.sort(key=lambda t: (-t[1], -t[0]))

    _say("  尺寸(列x行)   命中   平均误差      点数")
    for hits, npts, size, err in scored[:5]:
        _say("  %2d x %-2d      %d/%d    %5.2f px    %3d" % (size[0], size[1], hits, len(sample), err, npts))
    if not scored:
        return None
    best = scored[0][2]
    _say("  => 采用 %d x %d（%d 个内角点）" % (best[0], best[1], best[0] * best[1]))
    return best


def main():
    ap = argparse.ArgumentParser(description="棋盘格相机内参标定")
    ap.add_argument("--images", required=True, help="棋盘格照片文件夹")
    ap.add_argument("--cols", type=int, default=0, help="内角点列数（横向），0=自动猜")
    ap.add_argument("--rows", type=int, default=0, help="内角点行数（纵向），0=自动猜")
    ap.add_argument("--square", type=float, default=20.0, help="一个格子的边长，单位毫米")
    ap.add_argument("--auto-size", action="store_true", help="自动猜棋盘格尺寸（同 cols/rows=0）")
    ap.add_argument("--max-rms", type=float, default=1.0, help="单张图重投影误差超过这个值就剔除（像素）")
    ap.add_argument("--out", default=os.path.join(ROOT, "calibration", "out"), help="结果输出目录")
    args = ap.parse_args()

    paths = list_images(args.images)
    if not paths:
        _say("[错误] 在 %s 里没找到图片（支持 jpg/png/bmp/webp）" % args.images)
        return 1

    _say("== 读取图片 ==")
    imgs, names = [], []
    for p in paths:
        img = imread(p)
        if img is None:
            _say("  [跳过] 读不出来: %s（可能是 HEIC 或路径问题）" % os.path.basename(p))
            continue
        imgs.append(img)
        names.append(os.path.basename(p))
    if not imgs:
        _say("[错误] 没有一张能读的图")
        return 1

    shapes = {img.shape[:2] for img in imgs}
    if len(shapes) > 1:
        _say("[错误] 这些图片分辨率不一致：%s" % sorted(shapes))
        _say("       标定必须在同一分辨率下做，请统一成同一种尺寸再来。")
        return 1
    h, w = imgs[0].shape[:2]
    _say("  共 %d 张图，分辨率 %d x %d" % (len(imgs), w, h))
    if (w, h) != (1280, 720):
        _say("  [注意] 考核统一要求 1280x720。如果你最终也用这个分辨率，可以继续；否则请重新拍。")

    grays = [cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) for im in imgs]

    # ---------------- 1. 确定棋盘格尺寸
    if args.cols and args.rows:
        size = (args.cols, args.rows)
    elif args.auto_size or (not args.cols and not args.rows):
        _say("\n== 自动识别棋盘格尺寸 ==")
        size = guess_pattern(grays)
        if size is None:
            _say("[错误] 一张都没检出棋盘格。可能原因：")
            _say("       1) 照片里棋盘格太小/太糊/反光；2) 图案不是棋盘格；3) 用 --cols/--rows 手动指定。")
            return 1
        _say("  采用内角点尺寸：%d x %d" % size)
    else:
        _say("[错误] 请同时给 --cols 和 --rows，或者用 --auto-size")
        return 1

    # ---------------- 2. 找角点
    _say("\n== 检测角点 ==")
    objp = np.zeros((size[0] * size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : size[0], 0 : size[1]].T.reshape(-1, 2)
    objp *= args.square / 1000.0  # 毫米 → 米

    objpoints, imgpoints, used_names = [], [], []
    for name, gray in zip(names, grays):
        ok, corners = detect_corners(gray, size)
        if ok:
            objpoints.append(objp.copy())
            imgpoints.append(corners.astype(np.float32).reshape(-1, 1, 2))
            used_names.append(name)
            _say("  [成功] %s" % name)
        else:
            _say("  [失败] %s  （退到一边，不参与标定）" % name)

    n_ok = len(objpoints)
    _say("\n  可用于标定的图：%d 张" % n_ok)
    if n_ok < 6:
        _say("[错误] 成功检出的太少（至少要 6 张，建议 15 张以上）。")
        _say("       再补拍：把棋盘格放到画面四角和中间，前后左右倾斜一点，保证整块都在画面里。")
        return 1
    if n_ok < 15:
        _say("  [提醒] 少于 15 张，标定结果可能不稳，建议补到 15~25 张。")

    # ---------------- 3. 标定 + 剔除误差大的图
    _say("\n== 标定 ==")
    K, dist, rms, rvecs, tvecs = calibrate(objpoints, imgpoints, (w, h))
    per_err = view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist)
    _say("  初次标定：整体重投影误差 = %.4f 像素" % rms)

    dropped = []
    while n_ok > 8 and max(per_err) > args.max_rms:
        worst = int(np.argmax(per_err))
        _say("  剔除误差最大的图：%s（%.3f 像素）" % (used_names[worst], per_err[worst]))
        dropped.append(used_names[worst])
        for lst in (objpoints, imgpoints, used_names):
            del lst[worst]
        n_ok -= 1
        K, dist, rms, rvecs, tvecs = calibrate(objpoints, imgpoints, (w, h))
        per_err = view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist)

    if dropped:
        _say("  剔除后保留 %d 张，整体重投影误差 = %.4f 像素" % (len(used_names), rms))

    # ---------------- 4. 报告
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    k1, k2, p1, p2, k3 = dist.ravel()[:5] if dist.size >= 5 else (0, 0, 0, 0, 0)
    fov_x = 2 * np.rad2deg(np.arctan(w / (2 * fx)))
    fov_y = 2 * np.rad2deg(np.arctan(h / (2 * fy)))

    _say("\n== 结果 ==")
    _say("  fx = %.3f   fy = %.3f" % (fx, fy))
    _say("  cx = %.3f   cy = %.3f   (图像中心应为 %.1f, %.1f)" % (cx, cy, w / 2, h / 2))
    _say("  k1 = %.6f  k2 = %.6f  p1 = %.6f  p2 = %.6f  k3 = %.6f" % (k1, k2, p1, p2, k3))
    _say("  视场角约 %.1f° x %.1f°" % (fov_x, fov_y))
    _say("  整体重投影误差 = %.4f 像素" % rms)

    if rms < 0.3:
        _say("  [很好] 误差 < 0.3 像素，标定质量高。")
    elif rms < 0.6:
        _say("  [可用] 误差 < 0.6 像素，一般够用。")
    elif rms < 1.0:
        _say("  [一般] 误差 < 1.0 像素，建议多拍几张、把棋盘格摊平、避免反光后重标。")
    else:
        _say("  [较差] 误差 > 1.0 像素，基本是：棋盘格没摊平 / 照片糊了 / 只拍了画面中间。建议重拍。")

    # ---------------- 5. 保存
    os.makedirs(args.out, exist_ok=True)
    intr_path = os.path.join(args.out, "camera_intrinsics.yaml")
    save_yaml(
        intr_path,
        {
            "image_width": int(w),
            "image_height": int(h),
            "camera_name": "task2_intrinsics",
            "camera_matrix": [float(v) for v in K.ravel()],
            "distortion_coefficients": [float(v) for v in dist.ravel()],
            "dist_coeffs": [float(v) for v in dist.ravel()],
            "rms_reprojection_error": float(rms),
            "board": {
                "cols_inner": int(size[0]),
                "rows_inner": int(size[1]),
                "square_size_mm": float(args.square),
                "image_count_used": len(used_names),
                "image_count_dropped": len(dropped),
            },
            "fov_deg": [float(fov_x), float(fov_y)],
        },
    )
    _say("\n  已保存内参: %s" % intr_path)
    if dropped:
        _say("  被剔除的图: %s" % ", ".join(dropped))

    # ---------------- 6. 去畸变视觉检查
    check = make_undistort_check(imgs, K, dist, indices=[names.index(n) for n in used_names])
    if check is not None:
        vis_path = os.path.join(args.out, "undistort_check.jpg")
        if imwrite(vis_path, check):
            _say("  去畸变对比图: %s（左边原图，右边去畸变）" % vis_path)

    _say("\n下一步：把 %s 复制到你的仓库 calibration/camera_intrinsics.yaml" % os.path.basename(intr_path))
    _say("       然后继续做外参标定（src/tools/calibrate_extrinsics.py）。")
    return 0


def calibrate(objpoints, imgpoints, image_size):
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, criteria=_criteria()
    )
    return K, dist, float(rms), rvecs, tvecs


def view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist):
    errs = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / np.sqrt(len(proj))
        errs.append(float(err))
    return errs


def collect_points(grays, names, size, square_mm, min_images=4):
    """用指定尺寸把每张图的角点收进来，返回 dict；不够 min_images 张返回 None。"""
    objp = np.zeros((size[0] * size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : size[0], 0 : size[1]].T.reshape(-1, 2)
    objp *= square_mm / 1000.0

    objpoints, imgpoints, used = [], [], []
    for name, gray in zip(names, grays):
        ok, corners = detect_corners(gray, size)
        if ok:
            objpoints.append(objp.copy())
            imgpoints.append(corners.astype(np.float32).reshape(-1, 1, 2))
            used.append(name)
    if len(objpoints) < min_images:
        return None
    return {
        "size": tuple(size),
        "objpoints": objpoints,
        "imgpoints": imgpoints,
        "names": used,
        "detected": len(objpoints),
        "total": len(grays),
    }


def pick_best_size(grays, names, size, square_mm, min_images=4, verbose=True):
    """棋盘格的"横竖数反了"也能救：把 (列,行) 和 (行,列) 都试一遍，取重投影误差小的。

    因为很多棋盘格的格子是正方形，横着数是 6x9 还是 9x6 只能靠标定结果反推。
    """
    cands = [tuple(size)]
    transposed = (size[1], size[0])
    if transposed != tuple(size):
        cands.append(transposed)

    best = None
    for cand in cands:
        pack = collect_points(grays, names, cand, square_mm, min_images=min_images)
        if pack is None:
            if verbose:
                _say("  [跳过] %d x %d：检出的照片不足 %d 张" % (cand[0], cand[1], min_images))
            continue
        K, dist, rms, rvecs, tvecs = calibrate(pack["objpoints"], pack["imgpoints"], grays[0].shape[::-1])
        pack.update({"K": K, "dist": dist, "rms": rms, "rvecs": rvecs, "tvecs": tvecs})
        if verbose:
            _say("  试 %d x %d：%d 张检出，重投影误差 %.3f 像素" % (cand[0], cand[1], pack["detected"], rms))
        if best is None or pack["rms"] < best["rms"]:
            best = pack
    return best


def make_undistort_check(imgs, K, dist, indices):
    """把一张用于标定的图做成 原图 | 去畸变 的对比图，肉眼看直线直不直。"""
    if not indices:
        return None
    best = imgs[indices[0]]
    und = cv2.undistort(best, K, dist)
    h = min(best.shape[0], 720)
    a = cv2.resize(best, (int(best.shape[1] * h / best.shape[0]), h))
    b = cv2.resize(und, (int(und.shape[1] * h / und.shape[0]), h))
    gap = np.full((h, 10, 3), 255, np.uint8)
    return np.hstack([a, gap, b])


if __name__ == "__main__":
    sys.exit(main())
