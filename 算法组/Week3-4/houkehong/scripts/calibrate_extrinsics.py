# -*- coding: utf-8 -*-
"""
外参标定：反解相机的安装高度与俯仰角

为什么需要专门标定
------------------
外参数 (camera_height_m, pitch_deg) 决定"像素 -> 地面点"的映射。
本项目相机垂直视场只有 ±13°，看地面时视线非常贴近水平，
俯仰角差 1°，6 m 处的地面距离就会差 0.4~0.9 m —— 直接超过 0.5 m 的验收线。
所以俯仰角不能靠手机水平仪估，必须用已知距离反解。

原理
----
地面上距离相机正下方 D 米处的那一点，其载体坐标为 (D, 0, 0)。
给定候选外参 (h, θ)，把它投影到图像，得到预测的像素行 v_pred。
用卷尺量出真实 D，从画面上读出真实 v，构造残差：

    r_i(h, θ) = v_pred(D_i; h, θ) - v_i_measured

对多个点做最小二乘，解出 (h, θ)。只有一个未知量时（h 用卷尺量准了）
退化为单参数求解，更稳健 —— **推荐固定 h**。

实际操作（三脚架固定后进行）
----------------------------
1. 三脚架摆好、云台锁死，之后**不要再动**。
2. 用卷尺量相机光心离地高度 h（镜头中心，不是支架底）。
3. 在地面上，从相机正下方的地面投影点出发，沿镜头朝向拉一条卷尺。
4. 在 2 m / 4 m / 6 m / 8 m 处各放一个小标记（球、水瓶、胶带都行）。
5. 运行本工具拍照，用鼠标依次点每个标记**与地面接触的那一点**，输入它对应的 D。
6. 工具解出 θ，并报告每个点的距离误差（米），这才是真正关心的指标。

用法
----
    # 推荐：h 已用卷尺量准，只解俯仰角
    python scripts/calibrate_extrinsics.py --camera 1 --height 1.32

    # h 和 θ 都解（点至少 3 个）
    python scripts/calibrate_extrinsics.py --camera 1

    # 用已有的图片（现场没电脑时先拍照，回来再标）
    python scripts/calibrate_extrinsics.py --from-image snapshots/extr.jpg

    # 数学自检，不需要相机
    python scripts/calibrate_extrinsics.py --selftest

    # 解出来后直接写入 config/camera.yaml
    python scripts/calibrate_extrinsics.py --camera 1 --height 1.32 --write
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from src.config import load_camera  # noqa: E402
from src.geometry import CameraModel, Extrinsics  # noqa: E402

CAMERA_YAML = os.path.join(ROOT, "config", "camera.yaml")
MAX_W, MAX_H = 1280, 800


# ---------------------------------------------------------------- 正演
def project_ground(cam: CameraModel, h: float, pitch_deg: float,
                   distances) -> tuple[np.ndarray, np.ndarray]:
    """
    给定外参，把地面上距离 D 的点投影到像素。

    地面点载体坐标 = (D, 0, 0)（相机正前方 D 米、地面高度）
    注意：坐标系原点在相机光心的地面投影点，所以地面是 z = 0，
    而相机在 z = h。Extrinsics.t 的 z 分量就是相机高度。
    """
    ex = Extrinsics.from_mount(height_m=h, pitch_deg=pitch_deg)
    D = np.asarray(distances, dtype=np.float64).reshape(-1)
    P_b = np.stack([D, np.zeros_like(D), np.zeros_like(D)], axis=1)   # (N,3)
    # 载体 -> 相机：P_c = R^T (P_b - t)
    P_c = (ex.R.T @ (P_b - ex.t.reshape(1, 3)).T).T
    uv, ok = cam.project(P_c)
    return uv, ok


def solve(cam: CameraModel, samples, fix_height=None, pitch_guess=-20.0):
    """
    samples: [(D_measured, v_measured), ...]

    固定 h 时是单参数问题，直接网格 + 二分细化；
    否则做二维网格 + 局部细化。都不用外部优化库。
    """
    D = np.array([s[0] for s in samples], dtype=np.float64)
    V = np.array([s[1] for s in samples], dtype=np.float64)

    def cost(h, p):
        uv, ok = project_ground(cam, h, p, D)
        if not np.all(ok):
            return 1e9
        return float(np.mean((uv[:, 1] - V) ** 2))

    if fix_height is not None:
        h = float(fix_height)
        # 俯仰角搜索：从 0 到 -60 度，粗糙扫描后细化
        best_p, best_c = None, 1e18
        for p in np.arange(0.0, -60.0, -0.5):
            c = cost(h, p)
            if c < best_c:
                best_c, best_p = c, p
        # 在最优附近细化
        for step in (0.1, 0.02, 0.005, 0.001):
            cand = np.arange(best_p - 10 * step, best_p + 10 * step, step)
            for p in cand:
                c = cost(h, p)
                if c < best_c:
                    best_c, best_p = c, p
        return h, best_p, best_c

    # 二维：h 在 0.4~2.5 m，pitch 在 0~-60°
    best = (None, None, 1e18)
    for h in np.arange(0.4, 2.6, 0.05):
        for p in np.arange(0.0, -60.0, -1.0):
            c = cost(h, p)
            if c < best[2]:
                best = (h, p, c)
    h0, p0 = best[0], best[1]
    for sh, sp in ((0.02, 0.2), (0.005, 0.05), (0.001, 0.01)):
        improved = True
        while improved:
            improved = False
            for dh in (-sh, 0, sh):
                for dp in (-sp, 0, sp):
                    c = cost(h0 + dh, p0 + dp)
                    if c < best[2]:
                        best = (h0 + dh, p0 + dp, c)
                        h0, p0 = best[0], best[1]
                        improved = True
    return best[0], best[1], best[2]


def report(cam: CameraModel, h: float, pitch: float, samples) -> bool:
    """
    打印每个点的诊断。

    最有说服力的指标不是像素残差，而是**反算出来的距离误差（米）**：
    把实测像素行 v 反投影到地面，看得到多远的距离，和卷尺量的 D 比。
    """
    ex = Extrinsics.from_mount(height_m=h, pitch_deg=pitch)

    def v_to_distance(v):
        uv = np.array([[cam.cx, v]], dtype=np.float64)
        d_cam = cam.pixel_to_ray_camera(uv)
        o_b, d_b = ex.ray_to_carrier(d_cam)
        pts, ok = ex.ground_intersection(o_b, d_b)
        if not ok[0]:
            return float("nan")
        return float(math.hypot(pts[0, 0], pts[0, 1]))

    print(f"\n解出: camera_height_m = {h:.3f} m , pitch_deg = {pitch:.2f}")
    print("=" * 66)
    print(f"{'实测距离':>9}{'画面行 v':>10}{'反算距离':>10}{'误差':>9}   判定")
    print("-" * 66)
    errs = []
    for D, v in samples:
        d = v_to_distance(v)
        e = d - D
        errs.append(abs(e))
        flag = "OK" if abs(e) <= 0.15 else ("偏大" if abs(e) <= 0.4 else "**超差**")
        print(f"{D:>8.2f}m{v:>10.1f}{d:>9.2f}m{e:>+8.2f}m   {flag}")
    print("-" * 66)
    errs = np.array(errs)
    print(f"平均绝对误差 {errs.mean():.3f} m   最大 {errs.max():.3f} m")
    if errs.max() <= 0.15:
        print("✅ 拟合良好，可以用于验收（验收线 0.5 m）")
        return True
    if errs.max() <= 0.4:
        print("⚠️ 误差偏大。常见原因：卷尺起点不是相机正下方、点击点不是地面接触点、"
              "或标志本身有高度")
        return True
    print("❌ 误差超差。请重测：确认卷尺从相机地面投影点起算、点的是地面接触点、"
          "且三脚架全程没动")
    return False


def geometry_sanity(cam: CameraModel, h: float, pitch: float, samples) -> bool:
    """
    几何合理性检查 —— 光有低残差还不够，姿态本身可能很糟。

    背景：一次真实标定解出了 h=0.67 m / pitch=-0.89°（相机在膝盖高度、几乎水平）。
    3 个点拟合残差只有 4.5 px，看起来"很好"，但实际上：
        画面中央对应地面 44 m，地面只占画面底部 1/3，
        6→8 m 之间只有 40 像素（5 cm/px），远处更是 0.34 m/px。
    这种"贴地掠射"视角对任何微小误差都会放大，必须报警。

    检查项：
      1. 画面中央对应的地面距离（>15 m 说明镜头对着地平线，不是对着地面）
      2. 6 m 处的像素/米 敏感度（<20 px/m 说明远处被严重压缩）
    """
    ex = Extrinsics.from_mount(height_m=h, pitch_deg=pitch)

    def dist_at(v):
        uv = np.array([[cam.cx, v]], dtype=np.float64)
        d_cam = cam.pixel_to_ray_camera(uv)
        o_b, d_b = ex.ray_to_carrier(d_cam)
        pts, ok = ex.ground_intersection(o_b, d_b, max_range=200.0)
        if not ok[0]:
            return float("inf")
        return float(math.hypot(pts[0, 0], pts[0, 1]))

    d_bottom = dist_at(cam.height - 1)
    d_center = dist_at(cam.cy)
    d_top = dist_at(0)

    # 6 m 附近的分辨率：走 0.2 m 对应多少像素
    def v_at(D):
        uv, ok = project_ground(cam, h, pitch, [D])
        return float(uv[0, 1]) if ok[0] else float("nan")

    v1, v2 = v_at(6.0), v_at(6.2)
    px_per_m = abs(v1 - v2) / 0.2 if math.isfinite(v1) and math.isfinite(v2) else float("nan")

    print("\n" + "=" * 66)
    print("几何合理性检查（残差小不代表姿态对）")
    print("-" * 66)
    print(f"  画面底部  -> 地面 {d_bottom:8.2f} m")
    print(f"  画面中央  -> 地面 {d_center:8.2f} m" if math.isfinite(d_center)
          else "  画面中央  -> 打不到地面（朝上）")
    print(f"  画面顶部  -> 地面 {d_top:8.2f} m" if math.isfinite(d_top)
          else "  画面顶部  -> 打不到地面（朝上）")
    print(f"  6 m 附近分辨率: {px_per_m:.1f} px / 米"
          + ("" if not math.isfinite(px_per_m) else f"  (1 px ≈ {100/px_per_m:.1f} cm)"))

    bad = []
    if math.isfinite(d_center) and d_center > 15.0:
        bad.append(f"画面中央对应 {d_center:.0f} m 的地面 —— 镜头基本对着地平线，"
                   f"不是对着场地")
    if math.isfinite(px_per_m) and px_per_m < 20:
        bad.append(f"6 m 处只有 {px_per_m:.1f} px/米 —— 远处被严重压缩，"
                   f"标志位置误差会被放大")
    if math.isfinite(d_bottom) and d_bottom > 3.0:
        bad.append(f"画面底部也只能看到 {d_bottom:.1f} m 以外 —— 近处看不到地面")

    if bad:
        print("\n❌ 姿态不合理，即使残差很小也不要用于验收：")
        for b in bad:
            print(f"   · {b}")
        print("\n   常见原因：")
        print("     · 相机高度量错了（量的是支架/标记高度，不是镜头中心离地高度）")
        print("     · 点击的点不是地面接触点（点到了标志顶部或墙上）")
        print("     · 镜头俯角太小，几乎平视 —— 应下压到 15~25°")
        print(f"   参考：h=1.3 m + pitch=-20° 时，画面覆盖地面约 1.8~9.5 m，")
        print(f"         6 m 处约 90 px/米（1 px ≈ 1.1 cm）")
        return False
    print("\n✅ 姿态合理：地面占据画面的主要区域，远处分辨率足够")
    return True


def write_back(h: float, pitch: float, src_samples, err: float):
    with open(CAMERA_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    ex = data.setdefault("extrinsics", {})
    ex["camera_height_m"] = round(float(h), 4)
    ex["pitch_deg"] = round(float(pitch), 3)
    pts = ", ".join(f"D={d:.2f}m->v={v:.0f}" for d, v in src_samples)
    ex["method"] = (f"TAPE-MEASURE 反解 {len(src_samples)} 点；"
                    f"距离平均误差 {err:.3f}m；样本 {pts}")
    with open(CAMERA_YAML, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"\n已写入 {CAMERA_YAML}")


# ---------------------------------------------------------------- 交互
def pick_points(img, cam, win="extrinsics"):
    """在图上依次点击地面点，每点一次输入它到相机正下方的实测距离。"""
    scale = min(1.0, MAX_W / img.shape[1], MAX_H / img.shape[0])
    disp = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) \
        if scale < 1.0 else img.copy()
    pts = []          # [(D, v_full)]
    pending = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pending.append((x / scale, y / scale))

    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)

    print("\n点击地面标记的『与地面接触点』。每点一次在下方终端输入实测距离（米），"
          "回车确认。至少 2 点，建议 3~4 点。按 q 结束。")
    while True:
        show = disp.copy()
        for i, (D, v) in enumerate(pts):
            x = int(cam.cx * scale)
            yy = int(v * scale)
            cv2.drawMarker(show, (x, yy), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(show, f"{D:.2f}m", (x + 12, yy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(show, "click ground points, type distance in terminal, q to finish",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(show, f"points={len(pts)}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.imshow(win, show)

        key = cv2.waitKey(30)
        if key != -1 and (key & 0xFF) in (27, ord("q")):
            break

        if pending:
            cx, cy = pending.pop(0)
            print(f"  点击位置 u={cx:.0f} v={cy:.0f}")
            try:
                s = input("  该点到相机正下方的实测距离 (米，直接回车跳过): ").strip()
            except EOFError:
                s = ""
            if not s:
                continue
            try:
                D = float(s)
            except ValueError:
                print("  输入无效，已跳过")
                continue
            if D <= 0.2:
                print("  距离太小，已跳过")
                continue
            pts.append((D, float(cy)))
            print(f"  已记录 D={D:.2f} m, v={cy:.1f}")

    cv2.destroyWindow(win)
    return pts


def selftest():
    """数学自检：造一组已知外参的投影，看能不能反解回来。"""
    cam = load_camera(CAMERA_YAML)
    print(f"内参: fx={cam.fx:.2f} fy={cam.fy:.2f} cx={cam.cx:.2f} cy={cam.cy:.2f}")
    ok_all = True
    for h_true, p_true in ((1.30, -20.0), (0.85, -25.0), (1.50, -15.0)):
        Ds = [2.0, 3.5, 5.0, 7.0, 9.0]
        uv, ok = project_ground(cam, h_true, p_true, Ds)
        if not np.all(ok):
            print(f"[跳过] h={h_true} pitch={p_true} 有点投不到画面内")
            continue
        samples = [(d, float(v)) for d, v in zip(Ds, uv[:, 1])]
        h_est, p_est, _ = solve(cam, samples, fix_height=h_true)
        e_p = abs(p_est - p_true)
        print(f"\n真值 h={h_true:.2f} pitch={p_true:.1f}  "
              f"->  固定 h 解出 pitch={p_est:.2f}  误差 {e_p:.3f}°")
        ok_all &= e_p < 0.05
        h_est2, p_est2, _ = solve(cam, samples, fix_height=None)
        print(f"                          两个参数同时解: h={h_est2:.3f} pitch={p_est2:.2f}"
              f"  (h误差 {abs(h_est2-h_true):.3f} m, pitch误差 {abs(p_est2-p_true):.2f}°)")
    print("\n自检", "通过 ✅" if ok_all else "失败 ❌")
    return 0 if ok_all else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=1)
    ap.add_argument("--from-image", default=None, help="用已有图片而不是相机")
    ap.add_argument("--height", type=float, default=None,
                    help="卷尺量到的相机光心离地高度（米）。给了就只解俯仰角，强烈推荐")
    ap.add_argument("--write", action="store_true", help="结果写入 config/camera.yaml")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    cam = load_camera(CAMERA_YAML)
    img = None
    if args.from_image:
        img = cv2.imread(args.from_image)
        if img is None:
            print(f"[错误] 打不开图片 {args.from_image}")
            return 2
        if (img.shape[1], img.shape[0]) != (cam.width, cam.height):
            print(f"[警告] 图片尺寸 {img.shape[1]}x{img.shape[0]} 与内参标定尺寸 "
                  f"{cam.width}x{cam.height} 不一致，先缩放")
            img = cv2.resize(img, (cam.width, cam.height))
    else:
        sys.path.insert(0, ROOT)
        from run import open_camera
        cap = open_camera(args.camera, "msmf", cam.width, cam.height, 30)
        if not cap.isOpened():
            print(f"[错误] 打不开相机 {args.camera}")
            return 3
        for _ in range(20):
            cap.read()
        ok, frame = cap.read()
        cap.release()
        if not ok:
            print("[错误] 取帧失败")
            return 3
        img = frame

    samples = pick_points(img, cam)
    if len(samples) < 2:
        print(f"\n只拿到 {len(samples)} 个点，至少需要 2 个")
        return 1

    h, p, _ = solve(cam, samples, fix_height=args.height, pitch_guess=-20.0)
    good = report(cam, h, p, samples)
    geo_ok = geometry_sanity(cam, h, p, samples)

    # 反算距离的平均绝对误差（作为残差写进 yaml，之前错误地固定写成 0.0）
    ex = Extrinsics.from_mount(height_m=h, pitch_deg=p)

    def _dist_of(v):
        uv = np.array([[cam.cx, v]], dtype=np.float64)
        dmm = cam.pixel_to_ray_camera(uv)
        ob, db = ex.ray_to_carrier(dmm)
        pts, okk = ex.ground_intersection(ob, db)
        return float(math.hypot(pts[0, 0], pts[0, 1])) if okk[0] else float("nan")

    errs = [abs(_dist_of(v) - D) for D, v in samples]
    mean_err = float(np.nanmean(errs)) if errs else float("nan")

    if args.write:
        if good and geo_ok:
            write_back(h, p, samples, mean_err)
        else:
            print("\n未写入。请按上面的提示重新标定。")
    return 0 if (good and geo_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
