"""任务二 · 第 2 步：相机外参标定（相机相对地面的高度 / 俯仰 / 滚转）

为什么要做：同样一个像素，相机举高 10 厘米，它对应的地面点就会差出几十厘米。
单目要算"米"，必须知道相机离地多高、镜头往下低了多少度。

做法（推荐，最准）：把棋盘格**平铺在地面上**，相机固定不动作，拍 5~10 张
（板子放在相机前方不同位置、不同距离、不同朝向），然后跑这个脚本。
原理：棋盘格所在的平面 = 地面，用 solvePnP 解出相机相对板子的位姿，
就能反推出相机离地高度、俯仰角、滚转角。

用法（在仓库根目录执行）：

    # A. 用地面棋盘格照片标定
    python calibration/calibrate_extrinsics.py --images data/calib/ground \
        --intrinsics calibration/out/camera_intrinsics.yaml --cols 9 --rows 6 --square 25

    # B. 懒人法：拿尺子量高度、用量角器/水平仪 App 量俯仰角（精度差，能跑通用）
    python calibration/calibrate_extrinsics.py --intrinsics calibration/out/camera_intrinsics.yaml \
        --height 0.85 --pitch 20

注意：
  * 标定外参的相机必须和之后跑定位的相机、安装位置、分辨率完全一样，固定死。
  * 板子要贴地摊平（用胶带压住四角），有一点翘就会让高度和俯仰都偏。
  * 照片里的棋盘格最好占画面 1/4 以上，且出现在画面中下部（地面区域）。
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
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from calibrate_intrinsics import _say, detect_corners  # noqa: E402
from vision2.geometry import (  # noqa: E402
    GroundProjector,
    angles_from_rotation,
    pose_from_plane,
    rotation_from_height_pitch_roll,
)
from vision2.console import setup_console  # noqa: E402
from vision2.io_utils import imread, imwrite, list_images  # noqa: E402
from vision2.yaml_mini import load_yaml, save_yaml  # noqa: E402

setup_console()


def load_intrinsics(path):
    intr = load_yaml(path)
    K = np.asarray(intr["camera_matrix"], dtype=np.float64).reshape(3, 3)
    dist = np.asarray(intr.get("dist_coeffs") or intr.get("distortion_coefficients"), dtype=np.float64).reshape(-1, 1)
    w = intr.get("image_width")
    h = intr.get("image_height")
    return K, dist, w, h


def board_pose(corners, size, square_m, K, dist):
    """由角点求「相机 → 棋盘格」位姿（板子在板坐标系里是 z=0 平面）。

    solvePnP 给的是「板 → 相机」（R_cb, t_cb），这里转成「相机 → 板」（R_bc, t_bc），
    因为 pose_from_plane 要的是后者。
    """
    cols, rows = size
    objp = np.zeros((cols * rows, 3), np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m

    # 先去畸变，拿到归一化坐标，再用单位内参做 PnP
    und = cv2.undistortPoints(corners.reshape(-1, 1, 2), K, dist)
    ok, rvec, tvec = cv2.solvePnP(objp, und, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    R_cb, _ = cv2.Rodrigues(rvec)
    t_cb = tvec.reshape(3)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    proj = proj.astype(np.float32)
    rms = float(cv2.norm(corners.astype(np.float32).reshape(-1, 1, 2), proj, cv2.NORM_L2) / np.sqrt(len(proj)))

    R_bc = R_cb.T
    t_bc = -R_bc @ t_cb
    return R_bc, t_bc, rms


def robust_median(values, k=3.0):
    """去掉离群的测量值后取中位数，返回 (中位数, 保留的索引)。"""
    v = np.asarray(values, dtype=np.float64)
    med = np.median(v)
    mad = np.median(np.abs(v - med))
    scale = 1.4826 * mad
    if scale < 1e-9:
        keep = np.ones_like(v, dtype=bool)
    else:
        keep = np.abs(v - med) <= k * scale
    if keep.sum() < 2:
        keep = np.ones_like(v, dtype=bool)
    return float(np.median(v[keep])), np.flatnonzero(keep)


def draw_ground_grid(img, projector, max_x=5.0, max_y=2.0, step=1.0):
    """在图上画出地面网格（前向 0~max_x 米，左右 ±max_y 米），用来肉眼检查外参。"""
    vis = img.copy()
    H_, W_ = vis.shape[:2]
    rect = (0, 0, W_ - 1, H_ - 1)

    def P(x, y):
        uv = projector.ground_to_pixel(x, y)
        if uv is None:
            return None
        u, v = uv
        if not (np.isfinite(u) and np.isfinite(v)):
            return None
        return (int(round(u)), int(round(v)))

    def draw_run(points, color, thick=2):
        for a, b in zip(points[:-1], points[1:]):
            if a is None or b is None:
                continue
            ok, pa, pb = cv2.clipLine(rect, a, b)
            if ok:
                cv2.line(vis, pa, pb, color, thick, cv2.LINE_AA)

    # 沿 x 方向（横向线，每个 x 一条）
    x = max(step, 0.0)
    while x <= max_x + 1e-9:
        ys = np.arange(-max_y, max_y + 1e-9, step)
        pts = [P(x, y) for y in ys]
        draw_run(pts, (0, 200, 255) if abs(x - round(x)) < 1e-6 else (0, 160, 140))
        x += step

    # 沿 y 方向（纵向线，每个 y 一条）
    y = -max_y
    while y <= max_y + 1e-9:
        xs = np.arange(0.1, max_x + 1e-9, step)
        pts = [P(xv, y) for xv in xs]
        draw_run(pts, (0, 200, 255) if abs(y) < 1e-6 else (0, 160, 140))
        y += step

    for d in (1.0, 2.0, 3.0, 4.0, 5.0):
        p = P(d, 0.0)
        if p is not None and -100 < p[0] < W_ + 100 and -50 < p[1] < H_ + 50:
            cv2.putText(vis, "%.0fm" % d, (p[0] + 6, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2, cv2.LINE_AA)
    return vis


def main():
    ap = argparse.ArgumentParser(description="相机相对地面的外参标定")
    ap.add_argument("--images", help="地面棋盘格照片文件夹")
    ap.add_argument("--intrinsics", required=True, help="内参 yaml（calibrate_intrinsics.py 的输出）")
    ap.add_argument("--cols", type=int, default=9, help="内角点列数")
    ap.add_argument("--rows", type=int, default=6, help="内角点行数")
    ap.add_argument("--square", type=float, default=25.0, help="格子边长，毫米")
    ap.add_argument("--height", type=float, help="手工模式：相机光心离地高度（米）")
    ap.add_argument("--pitch", type=float, help="手工模式：俯仰角（度，向下为正）")
    ap.add_argument("--roll", type=float, default=0.0, help="手工模式：滚转角（度）")
    ap.add_argument("--out", default=os.path.join(ROOT, "calibration", "out"), help="输出目录")
    ap.add_argument("--debug", action="store_true", help="把检测结果画到图上保存")
    args = ap.parse_args()

    K, dist, w, h = load_intrinsics(args.intrinsics)
    _say("== 内参 ==")
    _say("  fx=%.2f fy=%.2f cx=%.2f cy=%.2f  图像 %sx%s" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2], w, h))

    # ---------------- 手工模式
    if args.height is not None and args.pitch is not None and not args.images:
        R_gc, t_gc = rotation_from_height_pitch_roll(args.height, args.pitch, args.roll)
        p, r = angles_from_rotation(R_gc)
        save_extrinsics(args.out, R_gc, t_gc, args.height, p, r, count=0, method="manual_measure")
        _say("\n== 手工外参 ==")
        _say("  高度 %.3f m，俯仰 %.2f°，滚转 %.2f°" % (args.height, p, r))
        _say("  [提醒] 手工量出来的值误差通常有几度、几厘米，能跑通但定位精度一般。")
        return 0

    if not args.images:
        _say("[错误] 要么给 --images（地面棋盘格照片），要么给 --height 和 --pitch 手工模式")
        return 1

    # ---------------- 照片模式
    paths = list_images(args.images)
    if not paths:
        _say("[错误] %s 里没有图片" % args.images)
        return 1

    size = (args.cols, args.rows)
    square_m = args.square / 1000.0
    _say("\n== 逐张求解 ==")
    results = []
    imgs, names = [], []
    for p in paths:
        img = imread(p)
        if img is None:
            continue
        if (w, h) and (img.shape[1], img.shape[0]) != (w, h):
            _say("  [跳过] %s 分辨率 %dx%d 与内参 %sx%s 不一致"
                 % (os.path.basename(p), img.shape[1], img.shape[0], w, h))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = detect_corners(gray, size)
        if not ok:
            _say("  [失败] %s  没检出棋盘格（整块要都在画面里、别反光）" % os.path.basename(p))
            continue
        pose = board_pose(corners, size, square_m, K, dist)
        if pose is None:
            _say("  [失败] %s  solvePnP 失败" % os.path.basename(p))
            continue
        R_bc, t_bc, rms = pose
        try:
            info = pose_from_plane(R_bc, t_bc)
        except ValueError as exc:
            _say("  [失败] %s  %s" % (os.path.basename(p), exc))
            continue
        results.append(info)
        imgs.append((img, corners))
        names.append(os.path.basename(p))
        _say("  [成功] %-28s 高度 %.3f m  俯仰 %5.2f°  滚转 %5.2f°  (重投影 %.2f px)"
             % (os.path.basename(p), info["height_m"], info["pitch_deg"], info["roll_deg"], rms))

    if not results:
        _say("\n[错误] 一张都没成功。检查：棋盘格是否整块在画面里？--cols/--rows 对不对？格子是不是反光？")
        return 1

    # ---------------- 汇总
    hs = [r["height_m"] for r in results]
    ps = [r["pitch_deg"] for r in results]
    rs = [r["roll_deg"] for r in results]
    h_med, keep_h = robust_median(hs)
    p_med, keep_p = robust_median(ps)
    r_med, keep_r = robust_median(rs)

    _say("\n== 汇总（%d 张有效） ==" % len(results))
    _say("  高度：中位数 %.3f m（各张 %.3f ~ %.3f）" % (h_med, min(hs), max(hs)))
    _say("  俯仰：中位数 %.2f°（各张 %.2f ~ %.2f）" % (p_med, min(ps), max(ps)))
    _say("  滚转：中位数 %.2f°（各张 %.2f ~ %.2f）" % (r_med, min(rs), max(rs)))

    if len(results) >= 2:
        spread_h = max(hs) - min(hs)
        spread_p = max(ps) - min(ps)
        if spread_h > 0.03:
            _say("  [警告] 不同照片算出的高度差 %.1f cm。可能是相机在两脚间动过，或板子没贴平。" % (spread_h * 100))
        if spread_p > 3.0:
            _say("  [警告] 俯仰角相差 %.1f°。相机可能没固定死，或板子翘起来了。" % spread_p)
        if spread_h <= 0.03 and spread_p <= 3.0:
            _say("  [很好] 各张结果一致，外参可信。")
    if len(results) < 5:
        _say("  [提醒] 有效照片 %d 张，建议 5 张以上，结果会更稳。" % len(results))

    R_gc, t_gc = rotation_from_height_pitch_roll(h_med, p_med, r_med)
    save_extrinsics(args.out, R_gc, t_gc, h_med, p_med, r_med, count=len(results),
                    method="ground_checkerboard")

    # ---------------- 视觉检查：把地面网格画到图上
    try:
        proj = GroundProjector(K, dist, R_gc, t_gc, width=w, height=h)
        vis = draw_ground_grid(imgs[0][0], proj)
        if args.debug:
            cv2.drawChessboardCorners(vis, size, imgs[0][1], True)
        vis_path = os.path.join(args.out, "ground_grid_check.jpg")
        if imwrite(vis_path, vis):
            _say("\n  核对图: %s" % vis_path)
            _say("  图上的黄色网格 = 程序认为的地面（0.5 m 一格，红线是 1m/2m/3m 标记）。")
            _say("  如果这张图的网格和真实地面完全贴合，外参就对了；歪了就是俯仰/滚转错。")
    except Exception as exc:  # pragma: no cover
        _say("  [跳过] 生成核对图失败：%s" % exc)

    _say("\n下一步：把 calibration/out/camera_extrinsics.yaml 复制到仓库 calibration/")
    return 0


def save_extrinsics(out_dir, R_gc, t_gc, height, pitch, roll, count, method):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "camera_extrinsics.yaml")
    save_yaml(
        path,
        {
            "camera_name": "task2_extrinsics",
            "frame": "ground_plane_origin_below_camera",
            "x_axis": "forward",
            "y_axis": "left",
            "z_axis": "up",
            "height_m": float(height),
            "pitch_deg": float(pitch),
            "roll_deg": float(roll),
            "t_gc": [float(v) for v in t_gc],
            "R_gc": [float(v) for v in np.asarray(R_gc).ravel()],
            "method": method,
            "image_count": int(count),
            "unit": "meter",
        },
    )
    _say("\n  已保存外参: %s" % path)
    _say("  相机高度 %.3f m，俯仰 %.2f°，滚转 %.2f°" % (height, pitch, roll))


if __name__ == "__main__":
    sys.exit(main())
