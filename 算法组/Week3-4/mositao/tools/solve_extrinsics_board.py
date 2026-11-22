# -*- coding: utf-8 -*-
"""用地面上的棋盘格反解外参（低机位最稳的一条路）。

为什么用棋盘格：
    0.73 m 这类低机位，场地标志在画面里被压成窄带，8 个手工点也只能做到十几像素残差，
    位姿的深度/横向分量病态。棋盘格有 54 个**亚像素**角点，且布局已知，PnP 立刻变成
    良态问题——这是标定界的标准做法。

摆放要求：
    1) 平铺在地面上，别卷边；
    2) 板面完整入画，离相机 1.5~3 m 最好；
    3) 板**中心**放在一个已知的场地坐标上（默认场地中心 (0,0)）；
    4) 板的长边尽量平行于场地 X 轴（对数后，偏差 2° 以内无所谓——这里把板子朝向
       也当成未知量一起解）。

板子规格默认与内参标定一致：9×6 内角点、方格 25 mm。

用法（项目根目录）：
    run.bat tools\\solve_extrinsics_board.py --image dataset\\manucalib3\\board.png
    run.bat tools\\solve_extrinsics_board.py --image ... --board-cx 0 --board-cy 0 --write

输出：
    outputs/extrinsics_board.md / outputs/extrinsics_board.png
    （--write 时写 calibration/camera_extrinsics.yaml）
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import load_yaml, save_yaml                            # noqa: E402
from src.common.paths import CALIB_DIR, OUTPUT_DIR, rel                         # noqa: E402
from src.loc.camera_model import CameraModel, Extrinsics                        # noqa: E402
from tools.fit_extrinsics_fixed_height import describe                           # noqa: E402
from tools.realtime_field import build_field_map                                 # noqa: E402
from tools.solve_extrinsics_grid import field_polyline                            # noqa: E402


def detect_corners(gray, cols, rows):
    """找棋盘内角点：优先 SB（更稳），退回经典方法，再做亚像素细化。"""
    ok, corners = False, None
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
    if not ok:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        return None
    if corners is not None and not hasattr(cv2, "findChessboardCornersSB"):
        pass
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    corners = cv2.cornerSubPix(gray, corners.astype(np.float32), (9, 9), (-1, -1), crit)
    return corners.reshape(-1, 2)


def main() -> int:
    ap = argparse.ArgumentParser(description="棋盘格外参标定")
    ap.add_argument("--image", required=True)
    ap.add_argument("--cols", type=int, default=9, help="内角点列数")
    ap.add_argument("--rows", type=int, default=6, help="内角点行数")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--board-cx", type=float, default=0.0, help="板中心在场地的 x（米）")
    ap.add_argument("--board-cy", type=float, default=0.0, help="板中心在场地的 y（米）")
    ap.add_argument("--vertical", action="store_true",
                    help="板子是竖起来的（推荐）：局部 x 水平、局部 y 竖直")
    ap.add_argument("--board-cz", type=float, default=0.0,
                    help="竖直板的中心离地高度（米），配合 --vertical")
    ap.add_argument("--board-yaw", type=float, default=None,
                    help="板相对场地 X 轴的朝向（度）；不给则一起解")
    ap.add_argument("--intrinsics", default=str(CALIB_DIR / "camera_intrinsics.yaml"))
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--geometry", default="config/field_geometry.yaml")
    ap.add_argument("--site", default="config/field_site.yaml")
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    img_path = Path(args.image) if Path(args.image).is_absolute() else ROOT / args.image
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"读不出图片：{img_path}")
        return 2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cam = CameraModel.from_yaml(ROOT / args.intrinsics)
    K = cam.K
    dist = np.asarray(cam.dist, float).reshape(-1, 1)

    corners = detect_corners(gray, args.cols, args.rows)
    if corners is None:
        print("没找到棋盘角点。检查：板子是否完整入画、光照是否足够、规格（--cols/--rows）是否对")
        return 1
    print(f"图片 {img_path.name}（{img.shape[1]}×{img.shape[0]}）｜角点 {len(corners)} 个")

    # 板坐标系（板中心为原点，z=0）
    sq = args.square_mm / 1000.0
    xs = (np.arange(args.cols) - (args.cols - 1) / 2.0) * sq
    ys = (np.arange(args.rows) - (args.rows - 1) / 2.0) * sq
    obj_board = np.array([[x, y, 0.0] for y in ys for x in xs], float)

    def solve_with_yaw(yaw_deg):
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        if args.vertical:
            # 竖直板：局部 (x横, y竖) → 场地 (cx + cos·x, cy + sin·x, cz + y)
            obj = np.array([[args.board_cx + c * x - s * 0.0,
                             args.board_cy + s * x + c * 0.0,
                             args.board_cz + y] for x, y, _z in obj_board])
        else:
            obj = np.array([[args.board_cx + c * x - s * y,
                             args.board_cy + s * x + c * y, 0.0] for x, y, _z in obj_board])
        # 平面目标有两支镜像解，ITERATIVE 只给其中一支，常常把相机解到板心下方。
        # 用 IPPE 枚举全部解，只保留「相机在板心上方」的物理可行支。
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj, corners, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if n == 0:
            return None
        best = None
        for k in range(n):
            rk, tk = rvecs[k], tvecs[k]
            Rk, _ = cv2.Rodrigues(rk)
            if (-Rk.T @ tk).ravel()[2] <= 0.0:
                continue
            rk, tk = cv2.solvePnPRefineLM(obj, corners, K, dist, rk, tk)
            Rk, _ = cv2.Rodrigues(rk)
            if (-Rk.T @ tk).ravel()[2] <= 0.0:
                continue
            proj, _ = cv2.projectPoints(obj, rk, tk, K, dist)
            rms = float(np.sqrt(((proj.reshape(-1, 2) - corners) ** 2).sum(1).mean()))
            if best is None or rms < best[2]:
                best = (rk, tk, rms)
        return best

    if args.board_yaw is not None:
        best = solve_with_yaw(args.board_yaw)
        if best is None:
            print("PnP 失败")
            return 1
        yaw_used = args.board_yaw
    else:
        # 板朝向当作未知量：粗扫 ±15° 再细化
        best, yaw_used = None, 0.0
        for y in np.arange(-15, 15.01, 1.0):
            r = solve_with_yaw(float(y))
            if r and (best is None or r[2] < best[2]):
                best, yaw_used = r, float(y)
        for y in np.arange(yaw_used - 1, yaw_used + 1.01, 0.1):
            r = solve_with_yaw(float(y))
            if r and (best is None or r[2] < best[2]):
                best, yaw_used = r, float(y)
        if best is None:
            print("PnP 失败")
            return 1

    rvec, tvec, rms = best
    R_pnp, _ = cv2.Rodrigues(rvec)
    R_ext = R_pnp.T
    t_ext = (-R_pnp.T @ tvec).ravel()

    d = describe(np.array([0, 0, 0, 0], float), t_ext[2]) if False else None
    optic = R_ext @ np.array([0.0, 0.0, 1.0])
    cam_x = R_ext @ np.array([1.0, 0.0, 0.0])
    roll_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(cam_x[2])))))
    print(f"板朝向（一并解出）：{yaw_used:+.1f}°")
    print(f"重投影 RMS：{rms:.2f} px")
    print(f"相机位置：x={t_ext[0]:+.3f} m  y={t_ext[1]:+.3f} m  h={t_ext[2]:.3f} m")
    print(f"光轴方向：yaw={math.degrees(math.atan2(optic[1], optic[0])):+.2f}°  "
          f"pitch={math.degrees(math.atan2(-optic[2], math.hypot(optic[0], optic[1]))):.2f}°  "
          f"roll={roll_deg:+.2f}°")

    # 叠图：把场地线投影上去
    fm = build_field_map(ROOT / args.field, ROOT / args.geometry, mirror=True,
                         site_path=ROOT / args.site)
    Pline = field_polyline(fm)
    ext = Extrinsics(R_ext, t_ext)
    p_cam = ext.base_to_cam(Pline)
    uv, okm = cam.project(p_cam)
    okm &= p_cam[:, 2] > 0.1
    vis = img.copy()
    last = None
    for i in range(len(uv)):
        if not okm[i]:
            last = None
            continue
        p = (int(uv[i][0]), int(uv[i][1]))
        if last is not None and abs(p[0] - last[0]) < 400 and abs(p[1] - last[1]) < 400:
            cv2.line(vis, last, p, (255, 160, 0), 2, cv2.LINE_AA)
        last = p
    out_png = OUTPUT_DIR / "extrinsics_board.png"
    cv2.imwrite(str(out_png), vis)

    md = OUTPUT_DIR / "extrinsics_board.md"
    with md.open("w", encoding="utf-8") as f:
        f.write("# 棋盘格外参标定\n\n")
        f.write(f"- 图片：`{rel(img_path)}`\n- 棋盘：{args.cols}×{args.rows} 内角点，方格 "
                f"{args.square_mm:.1f} mm\n")
        f.write(f"- 板中心场地坐标：({args.board_cx:+.3f}, {args.board_cy:+.3f}) m，"
                f"板朝向 {yaw_used:+.1f}°\n")
        f.write(f"- 重投影 RMS：{rms:.2f} px（{len(corners)} 个角点）\n\n")
        f.write("| 量 | 值 |\n|---|---:|\n")
        f.write(f"| x (m) | {t_ext[0]:+.3f} |\n| y (m) | {t_ext[1]:+.3f} |\n")
        f.write(f"| h (m) | {t_ext[2]:.3f} |\n")
        f.write(f"| yaw (°) | {math.degrees(math.atan2(optic[1], optic[0])):+.2f} |\n")
        f.write(f"| pitch (°) | {math.degrees(math.atan2(-optic[2], math.hypot(optic[0], optic[1]))):.2f} |\n\n")
        f.write(f"叠图：`{rel(out_png)}`\n")
    print(f"输出: {rel(md)} / {rel(out_png)}")

    if args.write:
        out_yaml = CALIB_DIR / "camera_extrinsics.yaml"
        save_yaml(out_yaml, {"camera_extrinsics": {
            "R": R_ext.tolist(), "t": t_ext.tolist(),
            "camera_height_m": round(float(t_ext[2]), 4),
            "pitch_deg": round(math.degrees(math.atan2(-optic[2],
                                                       math.hypot(optic[0], optic[1]))), 3),
            "yaw_deg": round(math.degrees(math.atan2(optic[1], optic[0])), 3),
            "roll_deg": 0.0,
            "demo": False,
            "method": f"棋盘格 PnP（{args.cols}×{args.rows} 内角点 / {args.square_mm:.0f} mm），"
                      f"板中心场地坐标 ({args.board_cx:.3f}, {args.board_cy:.3f})，板朝向 {yaw_used:.1f}°",
            "residual_px": round(rms, 3),
            "source_image": rel(img_path),
        }})
        print(f"输出: {rel(out_yaml)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
