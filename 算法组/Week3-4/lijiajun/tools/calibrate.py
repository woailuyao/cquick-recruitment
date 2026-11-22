import sys, os, cv2, numpy as np, yaml
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

REPO = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get("W34_DATA", REPO.parents[4] / "week34_data"))
CALIB = DATA / "calib"
OUT = REPO / "calibration" / "camera_intrinsics.yaml"
BOARD = (9, 9)
SQUARE = 0.0287
SIZE = (1280, 720)
MIN_IMAGES = 15
MIN_COVER = 0.22
DIST_FLAGS = cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)


def detect(gray):
    found, corners = cv2.findChessboardCornersSB(gray, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    return found, corners


def collect():
    w, h = BOARD
    objp = np.zeros((w * h, 3), np.float32)
    objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2) * SQUARE
    objs, imgs, names, dropped = [], [], [], []
    for p in sorted(CALIB.glob("*.png")):
        img = cv2.imread(str(p))
        if img is None:
            print("读不了", p.name)
            continue
        hh, ww = img.shape[:2]
        if (ww, hh) != SIZE:
            print("跳过非 720P:", p.name, (ww, hh))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = detect(gray)
        if not found:
            print("无角点:", p.name)
            continue
        c = np.asarray(corners).reshape(-1, 2).astype(np.float64)
        cover = max(float(np.ptp(c[:, 0])) / SIZE[0], float(np.ptp(c[:, 1])) / SIZE[1])
        if cover < MIN_COVER:
            print(f"剔除(板太小 cover={cover:.2f}):", p.name)
            dropped.append(p.name)
            continue
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRIT)
        objs.append(objp)
        imgs.append(corners)
        names.append(p.name)
    return objs, imgs, names, dropped


def coverage_report(imgs, need=3):
    buckets = {"左上": [], "右上": [], "左下": [], "右下": [], "中心": []}
    top_edge = 0
    for i, cor in enumerate(imgs):
        c = np.asarray(cor).reshape(-1, 2)
        u = float(c[:, 0].mean()) / SIZE[0]
        v = float(c[:, 1].mean()) / SIZE[1]
        if v < 0.28:
            top_edge += 1
        if abs(u - 0.5) < 0.18 and abs(v - 0.5) < 0.18:
            buckets["中心"].append(i)
        elif u < 0.4 and v < 0.4:
            buckets["左上"].append(i)
        elif u >= 0.4 and v < 0.4:
            buckets["右上"].append(i)
        elif u < 0.4:
            buckets["左下"].append(i)
        else:
            buckets["右下"].append(i)
    counts = {k: len(v) for k, v in buckets.items()}
    print("画面位置分布:", counts, f"| 贴到上边缘(v<0.28) {top_edge} 张")
    weak = [k for k, n in counts.items() if n < need]
    if top_edge < 3:
        weak.append("上边缘")
        print(f"上边缘覆盖不足（{top_edge} 张，需 >=3）：径向畸变和主点全靠画面边缘约束，先补拍。")
    if weak:
        print(f"缺口: {weak}")
    return counts, top_edge, weak


def per_view(objs, imgs, K, dist, rvecs, tvecs):
    out = []
    for i, (o, im) in enumerate(zip(objs, imgs)):
        proj, _ = cv2.projectPoints(o, rvecs[i], tvecs[i], K, dist)
        err = np.linalg.norm(im.reshape(-1, 2) - proj.reshape(-1, 2), axis=1)
        out.append((float(err.mean()), float(err.max()), i))
    return out


def check(K, dist, rms, hfov_ref):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    hfov = float(np.degrees(2 * np.arctan(SIZE[0] / 2 / fx)))
    k = np.ravel(dist)
    asym = abs(fx - fy) / fx
    tests = [
        ("cx", cx, 600 <= cx <= 680, "640 ± 40"),
        ("cy", cy, 330 <= cy <= 390, "360 ± 30"),
        ("abs(fx-fy)/fx", asym, asym < 0.05, "< 0.05"),
        ("k1", k[0], -0.5 <= k[0] <= 0.5, "-0.5 ~ 0.5"),
        ("rms", rms, rms < 0.6, "< 0.6"),
    ]
    bad = 0
    for name, val, ok, rule in tests:
        print(f"  {'PASS' if ok else 'FAIL'}  {name:<14} = {val:10.4f}   要求 {rule}")
        bad += 0 if ok else 1
    if hfov_ref is None:
        print(f"  TODO  HFOV            = {hfov:10.4f}   尚未用尺子实测验证（见 README 验证步骤）")
    else:
        dev = abs(hfov - hfov_ref) / hfov_ref
        print(f"  {'PASS' if dev < 0.10 else 'FAIL'}  {'HFOV_vs_实测':<14} = {dev:10.4f}   要求 与尺子实测差 <10%")
        bad += 0 if dev < 0.10 else 1
    return bad, hfov


def main():
    print(f"标定板: 内角点 {BOARD}（方格 {BOARD[0]+1}x{BOARD[1]+1}），单格 {SQUARE*1000:.1f} mm")
    print(f"畸变模型: 只估 k1/p1/p2，k2=k3=0（实测 5 参数模型会用极端 k2/k3 拟合板子弯曲）")
    print(f"板占比下限: {MIN_COVER}")
    objs, imgs, names, dropped = collect()
    print(f"有效标定图 {len(objs)} 张（要求 >= {MIN_IMAGES}），自动剔除 {len(dropped)} 张")
    counts, top_edge, weak = coverage_report(imgs)
    if len(objs) < MIN_IMAGES or weak:
        print("\n先别标定：补拍缺的区域后再跑。想先看趋势可加 --force。")
        if "--force" not in sys.argv:
            raise SystemExit(1)
        print("（--force 已指定，继续标定，但结果不可用于地面投影）")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objs, imgs, SIZE, None, None, flags=DIST_FLAGS)
    errs = sorted(per_view(objs, imgs, K, dist, rvecs, tvecs), reverse=True)
    avg_all = sum(e for e, _, _ in errs) / len(errs)

    print(f"\nfx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    print(f"dist(k1,k2,p1,p2,k3) = {np.ravel(dist).round(4).tolist()}")
    print(f"rms={rms:.4f}  平均重投影={avg_all:.3f} px")

    board_d = [float(t.ravel()[2]) for t in tvecs]
    print(f"标定板到相机光心距离: {min(board_d):.2f} ~ {max(board_d):.2f} m"
          f"  （必须落在你实测的清晰区间内，否则 fx 有问题）")

    print("\n最差 5 张:")
    for e, mx, i in errs[:5]:
        print(f"  {names[i]:<36} avg={e:.3f} px  max={mx:.3f} px")

    print("\n质量判据:")
    bad, hfov = check(K, dist, rms, None)

    data = {
        "image_size": [SIZE[0], SIZE[1]],
        "camera_matrix": [[float(v) for v in row] for row in K],
        "dist_coeffs": [float(v) for v in np.ravel(dist)],
        "dist_order": ["k1", "k2", "p1", "p2", "k3"],
        "dist_model": "k1 only (k2=k3=0 fixed)",
        "hfov_deg": round(hfov, 2),
        "rms_error": round(float(rms), 4),
        "reprojection_error_px": round(float(avg_all), 3),
        "board_inner_corners": list(BOARD),
        "board_square_m": SQUARE,
        "board_cover_min": MIN_COVER,
        "n_images": len(objs),
        "n_dropped_small": len(dropped),
        "coverage": counts,
        "n_top_edge": top_edge,
        "board_distance_range_m": [round(min(board_d), 2), round(max(board_d), 2)],
        "quality_fail_count": bad,
        "source_files": names,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print("\n写入", OUT)
    if bad:
        print(f"有 {bad} 项不合格：先别用这份内参做地面投影")


if __name__ == "__main__":
    main()
