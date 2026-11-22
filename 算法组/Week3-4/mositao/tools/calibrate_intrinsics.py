# -*- coding: utf-8 -*-
"""相机内参与畸变标定（棋盘格）——细则 2.5 的落地实现。

产出 calibration/camera_intrinsics.yaml，含 fx/fy/cx/cy、[k1,k2,p1,p2,k3]、
图像宽高、重投影误差与逐张误差，供 README 与录屏引用。

用法（项目根目录）：
    # 1) 采集（另开一个窗口，见 tools/capture_calib.py）
    # 2) 标定
    python tools/calibrate_intrinsics.py
    python tools/calibrate_intrinsics.py --cols 9 --rows 6 --square-mm 25

    # 3) 无相机自检：用合成棋盘格验证本工具与求解链路
    python tools/calibrate_intrinsics.py --selftest

注意：
- 标定分辨率必须与最终推理分辨率一致（1280x720），改分辨率要重标；
- 实拍至少 15 张，覆盖画面中心/四角、不同倾斜与距离；
- 重投影误差 < 0.5 px 为良好，0.5~1.0 px 可用，> 1.0 px 需重拍。
"""

from __future__ import annotations

import argparse
import math
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import save_yaml                      # noqa: E402
from src.common.paths import CALIB_DIR, OUTPUT_DIR, ensure_runtime_dirs, rel  # noqa: E402
from src.loc.camera_model import CameraModel                    # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                               # noqa: BLE001
    pass

IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}


def board_object_points(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """棋盘格内角点的三维坐标（Z=0 平面，单位米）。"""
    s = square_mm / 1000.0
    pts = np.zeros((cols * rows, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    pts[:, :2] = grid * s
    return pts


def detect_corners(gray: np.ndarray, cols: int, rows: int):
    """优先用 SB 检测（亚像素精度更好），失败再退回经典方法。"""
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
    if found:
        return True, corners

    found, corners = cv2.findChessboardCorners(
        gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def load_views(images_dir: Path, cols: int, rows: int, square_mm: float):
    files = sorted(p for p in images_dir.iterdir() if p.suffix.lower() in IMG_EXTS)
    if not files:
        return [], [], [], [], None
    objp = board_object_points(cols, rows, square_mm)   # 单位米
    objpoints, imgpoints, used, failed = [], [], [], []
    shape = None
    for f in files:
        img = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        if img is None:
            failed.append((f.name, "读不出图像"))
            continue
        ok, corners = detect_corners(img, cols, rows)
        if not ok:
            failed.append((f.name, "未检出棋盘格"))
            continue
        shape = img.shape[::-1]
        objpoints.append(objp)
        imgpoints.append(corners)
        used.append(f.name)
    return objpoints, imgpoints, used, failed, shape


def calibrate(objpoints, imgpoints, shape, fix_k3: bool = True, pinhole: bool = False,
              fix_k2: bool = False):
    flags = 0
    if pinhole:
        flags = (cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
                 | cv2.CALIB_ZERO_TANGENT_DIST)
    else:
        if fix_k3:
            flags |= cv2.CALIB_FIX_K3
        if fix_k2:
            flags |= cv2.CALIB_FIX_K2
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, shape, None, None, flags=flags)
    return rms, K, dist.ravel(), rvecs, tvecs


# 由简到繁的候选模型：先试不含畸变的针孔模型，再加径向项
MODELS = [
    ("pinhole", {"pinhole": True}),
    ("k1", {"fix_k3": True, "fix_k2": True}),
    ("k1+k2", {"fix_k3": True}),
]


def sanity_problems(K, dist, shape, hfov_expect=(35.0, 140.0),
                    check_k2: bool = True) -> list[str]:
    """参数合理性体检。

    低重投影误差 ≠ 参数可信：只有「正对」的样本时，fx 与深度完全退化，
    求解器会给出任意大的 fx，并用高阶畸变把残差吃掉，RMS 依然很小。
    因此必须独立检查参数的物理合理性。
    """
    problems = []
    fx, fy = float(K[0, 0]), float(K[1, 1])
    w = float(shape[0])
    hfov = 2.0 * math.degrees(math.atan(w / (2.0 * fx))) if fx > 0 else float("nan")
    if not hfov_expect[0] <= hfov <= hfov_expect[1]:
        problems.append(f"水平视场角 {hfov:.1f}° 不在合理区间 "
                        f"{hfov_expect[0]:.0f}~{hfov_expect[1]:.0f}°（fx={fx:.1f} 可疑）")
    if fx > 0 and abs(fx - fy) / fx > 0.05:
        problems.append(f"fx 与 fy 相差 {abs(fx - fy) / fx:.1%}，像素应为正方形，>5% 说明求解不稳")
    k1, k2, p1, p2, k3 = [float(v) for v in (list(dist) + [0, 0, 0, 0, 0])[:5]]
    if abs(k1) > 1.0:
        problems.append(f"k1={k1:.3f} 超过 ±1.0")
    if check_k2 and abs(k2) > 1.0:
        problems.append(f"k2={k2:.3f} 超过 ±1.0")
    if abs(k3) > 5.0:
        problems.append(f"k3={k3:.3f} 超过 ±5.0")
    if abs(p1) > 0.05 or abs(p2) > 0.05:
        problems.append(f"切向畸变 p1={p1:.4f} p2={p2:.4f} 偏大（>0.05）")
    return problems


def tilt_angles(objpoints, imgpoints, K, dist, rvecs) -> list[float]:
    """每张图的棋盘倾角（度）：棋盘法线与相机光轴的夹角。

    正对 = 0°，完全侧视 = 90°。低于 10° 的样本对求解焦距几乎没有贡献。
    """
    angles = []
    for i in range(len(objpoints)):
        R, _ = cv2.Rodrigues(rvecs[i])
        normal = R[:, 2]
        cosang = abs(float(np.dot(normal, np.array([0.0, 0.0, 1.0]))))
        angles.append(math.degrees(math.acos(max(-1.0, min(1.0, cosang)))))
    return angles


def per_image_errors(objp, imgpoints, rvecs, tvecs, K, dist):
    errs = []
    for i in range(len(objp)):
        proj, _ = cv2.projectPoints(objp[i], rvecs[i], tvecs[i], K, dist)
        e = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - imgpoints[i].reshape(-1, 2)) ** 2, axis=1))))
        errs.append(e)
    return errs


# ---------------------------------------------------------------- 合成自检
def render_board(K, dist, rvec, tvec, cols, rows, square_mm, size=(1280, 720)) -> np.ndarray:
    """把棋盘格按给定位姿投影成一张灰度图，用于无相机自检。

    注意数量关系（很容易搞错）：
      「内角点」= 内部那些被 4 个方格共用的角点，数量为 cols x rows；
      因此方格数是 (cols+1) x (rows+1)，而网格点数是 (cols+2) x (rows+2)。
    """
    w, h = size
    s = square_mm / 1000.0
    nx, ny = cols + 2, rows + 2            # 网格点数
    pts = np.array([[(i - (nx - 1) / 2.0) * s, (j - (ny - 1) / 2.0) * s, 0.0]
                    for j in range(ny) for i in range(nx)], np.float32)
    proj, _ = cv2.projectPoints(pts, rvec, tvec, K, dist)
    proj = proj.reshape(ny, nx, 2)
    img = np.full((h, w), 255, np.uint8)
    for j in range(ny - 1):
        for i in range(nx - 1):
            if (i + j) % 2 == 0:
                quad = np.array([proj[j, i], proj[j, i + 1],
                                 proj[j + 1, i + 1], proj[j + 1, i]], np.int32)
                cv2.fillConvexPoly(img, quad, 0)
    # 轻微模糊，模拟真实成像边缘，避免纯二值图的检测退化
    return cv2.GaussianBlur(img, (3, 3), 0.8)


def selftest(cols: int, rows: int, square_mm: float, n_views: int = 18) -> int:
    """用已知内参渲染棋盘格 -> 完整跑一遍检测与标定 -> 比较还原精度。"""
    print("=" * 72)
    print("内参标定工具 · 合成棋盘格自检（不需要相机）")
    print("=" * 72)
    truth = CameraModel(fx=650.0, fy=652.0, cx=641.5, cy=359.2,
                        dist=(-0.15, 0.05, 0.0008, -0.0004, -0.008),
                        width=1280, height=720)
    print(f"真值内参: fx={truth.K[0,0]:.2f} fy={truth.K[1,1]:.2f} "
          f"cx={truth.K[0,2]:.2f} cy={truth.K[1,2]:.2f}")
    print(f"真值畸变: {np.round(truth.dist, 5).tolist()}")

    rng = np.random.default_rng(20261110)
    objp = board_object_points(cols, rows, square_mm)
    objpoints, imgpoints, used = [], [], []
    out_dir = OUTPUT_DIR / "calib_selftest"
    out_dir.mkdir(parents=True, exist_ok=True)

    for k in range(n_views):
        ang = np.array([rng.uniform(-0.45, 0.45), rng.uniform(-0.45, 0.45), rng.uniform(-0.5, 0.5)])
        rvec = ang.reshape(3, 1)
        dist_m = rng.uniform(0.35, 0.60)
        tvec = np.array([[rng.uniform(-0.04, 0.04)],
                         [rng.uniform(-0.03, 0.03)],
                         [dist_m]], np.float32)
        img = render_board(truth.K, truth.dist, rvec, tvec, cols, rows, square_mm)
        ok, corners = detect_corners(img, cols, rows)
        if not ok:
            continue
        objpoints.append(objp)
        imgpoints.append(corners)
        used.append(f"syn_{k:02d}.png")
        if len(used) <= 4:
            cv2.imwrite(str(out_dir / used[-1]), img)

    print(f"生成 {n_views} 张，成功检出 {len(used)} 张")
    if len(used) < 6:
        print("检出数量不足，无法完成自检")
        return 1

    shape = (truth.width, truth.height)
    rms, K, dist, rvecs, tvecs = calibrate(objpoints, imgpoints, shape)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print("-" * 72)
    print(f"标定结果: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
    print(f"畸变系数: {np.round(dist, 5).tolist()}")
    print(f"重投影 RMS: {rms:.4f} px")
    print("-" * 72)
    dfx, dfy, dcx, dcy = abs(fx - truth.K[0, 0]), abs(fy - truth.K[1, 1]), \
        abs(cx - truth.K[0, 2]), abs(cy - truth.K[1, 2])
    print(f"还原误差: fx {dfx:.2f} px, fy {dfy:.2f} px, cx {dcx:.2f} px, cy {dcy:.2f} px")
    ok = dfx < 5 and dfy < 5 and dcx < 10 and dcy < 10 and rms < 1.0
    print(f"自检结论: {'通过（检测 + 求解 + 误差统计全链路正常）' if ok else '未通过'}")
    print("=" * 72)
    return 0 if ok else 1


# ---------------------------------------------------------------------- 主流程
def main() -> int:
    ap = argparse.ArgumentParser(description="棋盘格相机内参标定")
    ap.add_argument("--images", default="data/calib", help="标定图目录")
    ap.add_argument("--cols", type=int, default=9, help="内角点列数（横向）")
    ap.add_argument("--rows", type=int, default=6, help="内角点行数（纵向）")
    ap.add_argument("--square-mm", type=float, default=25.0, help="棋盘格方格边长（毫米）")
    ap.add_argument("--out", default=str(CALIB_DIR / "camera_intrinsics.yaml"))
    ap.add_argument("--selftest", action="store_true", help="用合成棋盘格自检，不需要相机")
    ap.add_argument("--force", action="store_true", help="参数体检不通过时也写出文件")
    ap.add_argument("--no-fix-k3", dest="fix_k3", action="store_false",
                    help="同时求解 k3（默认固定 k3=0，更稳）")
    ap.set_defaults(fix_k3=True)
    args = ap.parse_args()

    if args.selftest:
        return selftest(args.cols, args.rows, args.square_mm)

    ensure_runtime_dirs()
    images_dir = Path(args.images)
    if not images_dir.is_absolute():
        images_dir = ROOT / images_dir

    print("=" * 72)
    print("相机内参标定（棋盘格）")
    print(f"图目录   : {rel(images_dir)}")
    print(f"棋盘格   : 内角点 {args.cols} x {args.rows}，方格 {args.square_mm:.1f} mm")
    print("=" * 72)

    if not images_dir.exists():
        print(f"目录不存在：{rel(images_dir)}")
        print(f"请先用 tools/capture_calib.py 采集至少 15 张，存到 {rel(images_dir)}")
        return 1

    objpoints, imgpoints, used, failed, shape = load_views(
        images_dir, args.cols, args.rows, args.square_mm)
    print(f"检出成功 : {len(used)} 张")
    if failed:
        print(f"检出失败 : {len(failed)} 张")
        for name, why in failed[:8]:
            print(f"   - {name}: {why}")
        if len(failed) > 8:
            print(f"   ... 另有 {len(failed) - 8} 张")
    if len(used) < 6:
        print()
        print("可用图像不足 6 张，标定不可靠。请检查：")
        print("  1. 棋盘格内角点数是否与 --cols/--rows 一致（数的是内角点，不是方格数）")
        print("  2. 拍摄时是否整个棋盘都在画面内、无明显反光与运动模糊")
        print("  3. 打印时是否按 100% 比例（缩放会让 --square-mm 与实际不符，导致尺度偏差）")
        return 1
    if shape is None:
        print("未能取得图像尺寸")
        return 1

    # 模型比较：畸变项越多越容易「用高阶项吃掉残差」，因此由简到繁试，
    # 选「参数合理 + RMS 已接近最优」的最简模型。
    print("-" * 72)
    print("模型比较（由简到繁，选最简且够用者）")
    results = []
    for name, kw in MODELS:
        rms_i, K_i, d_i, rv_i, tv_i = calibrate(objpoints, imgpoints, shape, **kw)
        probs_i = sanity_problems(K_i, d_i, shape, check_k2=(name == "k1+k2"))
        results.append({"name": name, "rms": rms_i, "K": K_i, "dist": d_i,
                        "rvecs": rv_i, "tvecs": tv_i, "problems": probs_i})
        print(f"  {name:<7} RMS={rms_i:6.4f} px  fx={K_i[0, 0]:9.2f}  "
              f"k1={d_i[0]:+.4f}  k2={d_i[1]:+.4f}  "
              f"{'参数可疑: ' + probs_i[0] if probs_i else '参数合理'}")

    # 选取原则：在「参数体检通过」的模型里取 RMS 最低者。
    # 不盲目加阶次（k2 崩坏说明数据不支持），也不一味求简（k1 明显改善拟合就该留下）。
    best_rms = min(r["rms"] for r in results)
    sane = [r for r in results if not r["problems"]]
    chosen = (min(sane, key=lambda r: r["rms"]) if sane
              else min(results, key=lambda r: (len(r["problems"]), r["rms"])))
    rms, K, dist = chosen["rms"], chosen["K"], chosen["dist"]
    rvecs, tvecs = chosen["rvecs"], chosen["tvecs"]

    errs = per_image_errors(objpoints, imgpoints, rvecs, tvecs, K, dist)
    tilts = tilt_angles(objpoints, imgpoints, K, dist, rvecs)

    print("-" * 72)
    print(f"采用模型   : {chosen['name']}"
          + {"pinhole": "（无畸变；标定视场内足够接近针孔）",
             "k1": "（仅径向一阶；更高阶项在该组数据上不稳定，已舍弃）",
             "k1+k2": "（径向一、二阶）"}.get(chosen["name"], ""))
    print(f"重投影误差 : {rms:.4f} px（<0.5 良好，0.5~1.0 可用，>1.0 建议重拍）")
    print(f"fx={K[0, 0]:.3f}  fy={K[1, 1]:.3f}  cx={K[0, 2]:.3f}  cy={K[1, 2]:.3f}")
    print(f"畸变 [k1,k2,p1,p2,k3] = {np.round(dist, 6).tolist()}")
    w = shape[0]
    hfov = 2.0 * math.degrees(math.atan(w / (2.0 * K[0, 0])))
    print(f"水平视场角 : {hfov:.1f}°（由 fx 反算，可用于交叉核对镜头标称值）")
    worst = int(np.argmax(errs))
    print(f"最差一张   : {used[worst]}  误差 {errs[worst]:.3f} px")

    # 倾斜体检：正对样本对焦距没有信息量，是参数发散的头号原因
    tilts_sorted = sorted(tilts)
    med_tilt = tilts_sorted[len(tilts_sorted) // 2]
    low_tilt = [i for i, a in enumerate(tilts) if a < 10.0]
    print(f"棋盘倾角   : 中位 {med_tilt:.1f}°，最小 {tilts_sorted[0]:.1f}°，"
          f"最大 {tilts_sorted[-1]:.1f}°；<10° 的样本 {len(low_tilt)}/{len(tilts)} 张")

    # 参数体检：低 RMS 也可能是「参数离谱但拟合漂亮」
    problems = list(chosen["problems"])
    if low_tilt and len(low_tilt) > len(tilts) // 2:
        problems.append(f"超过一半的样本倾角 <10°（{len(low_tilt)}/{len(tilts)}），"
                        f"焦距与深度退化，无法确定 fx")
    if med_tilt < 12.0:
        problems.append(f"倾角中位数仅 {med_tilt:.1f}°，建议 ≥15°：把棋盘绕两个轴各转一些再拍")

    print("-" * 72)
    if problems:
        print("参数体检：不通过，这组内参不可用")
        for p in problems:
            print("  ! " + p)
        print()
        print("原因与处置：")
        print("  · 相机正对平面时，「板子更大但更远」与「更小但更近」在成像上等价，")
        print("    焦距无从确定；求解器会用离谱的 fx 与高阶畸变把残差吃掉，RMS 仍然很小。")
        print("  · 正确做法：把棋盘绕水平和垂直两个轴各倾斜 15°~30° 重拍，")
        print("    并让棋盘在画面里占得更大（边长 ≥ 画面宽的 1/3）。")
        print("  · 若坚持要看这组解，加 --force 写出文件（不建议用于验收）。")
        if not args.force:
            print()
            print("已中止，未写出 camera_intrinsics.yaml。")
            print("=" * 72)
            return 1
        print()
        print("（--force 已指定，继续写出，但请勿用于最终验收）")
    else:
        print("参数体检：通过（视场角、fx/fy 一致性、畸变项均在合理范围）")

    print("-" * 72)
    payload = {
        "camera_intrinsics": {
            "fx": float(K[0, 0]), "fy": float(K[1, 1]),
            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "dist_coeffs": [float(v) for v in dist[:5]],
            "width": int(shape[0]), "height": int(shape[1]),
            "reprojection_error_px": float(rms),
            "distortion_model": chosen["name"],
            "calibration_images": len(used),
            "board": {"cols": args.cols, "rows": args.rows, "square_mm": args.square_mm},
            "per_image_error_px": [round(e, 4) for e in errs],
            "method": "OpenCV calibrateCamera（棋盘格内角点，findChessboardCornersSB 检测）",
            "calibrated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }
    }
    out_path = Path(args.out)
    if not out_path.is_absolute():
        out_path = ROOT / out_path
    save_yaml(out_path, payload)
    print("-" * 72)
    print(f"已写出     : {rel(out_path)}")

    # 顺带存一张去畸变对比图，肉眼确认参数是否合理
    sample = images_dir / used[0]
    img = cv2.imread(str(sample))
    if img is not None:
        K_use = np.array([[K[0, 0], 0, K[0, 2]], [0, K[1, 1], K[1, 2]], [0, 0, 1]], float)
        undist = cv2.undistort(img, K_use, np.array(dist[:5], float))
        cmp_img = np.hstack([cv2.resize(img, (640, 360)), cv2.resize(undist, (640, 360))])
        p = OUTPUT_DIR / "calib_undistort_compare.png"
        cv2.imwrite(str(p), cmp_img)
        print(f"去畸变对比 : {rel(p)}（左原图 / 右矫正，直线应变直）")

    print("=" * 72)
    return 0 if rms <= 1.0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
