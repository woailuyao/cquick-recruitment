import sys, re, os, cv2, numpy as np, yaml
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stderr.reconfigure(encoding="utf-8", errors="replace")

REPO = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get("W34_DATA", REPO.parents[4] / "week34_data"))
MARK_DIR = DATA / "extrinsics"
INTR = REPO / "calibration" / "camera_intrinsics.yaml"
OUT = INTR.parent / "camera_extrinsics.yaml"

H_SAMPLES = [0.725, 0.727, 0.729]
ROLL_EDGE_SAMPLES_M = [0.748, 0.751]
ROLL_BASE_WIDTH_M = 0.04
V_MIN, V_MAX, RB_MIN = 150, 255, 5
MIN_AREA = 40
GROUND_V_MIN = 300


def load_intrinsics():
    d = yaml.safe_load(INTR.read_text(encoding="utf-8"))
    K = np.array(d["camera_matrix"], dtype=np.float64)
    dist = np.array(d["dist_coeffs"], dtype=np.float64)
    return K, dist, d


def find_marks(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    B = img[:, :, 0].astype(int)
    G = img[:, :, 1].astype(int)
    R = img[:, :, 2].astype(int)
    V = hsv[:, :, 2].astype(int)
    mask = ((V >= V_MIN) & (V <= V_MAX) & (R >= B + RB_MIN) & (R >= 90)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((17, 17), np.uint8))
    num, lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    cands = []
    for i in range(1, num):
        a = stats[i, cv2.CC_STAT_AREA]
        if a < MIN_AREA:
            continue
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        fill = a / max(w * h, 1)
        cands.append([a, float(cent[i][0]), float(cent[i][1]), int(w), int(h), float(fill)])
    return sorted(cands, key=lambda c: -c[0]), mask


def parse_distances(stem):
    nums = [float(x) for x in re.findall(r"(\d+(?:\.\d+)?)", stem)]
    return sorted(nums)


def main():
    K, dist, intr = load_intrinsics()
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    h = float(np.mean(H_SAMPLES))
    roll = float(np.degrees(np.arctan2(ROLL_EDGE_SAMPLES_M[1] - ROLL_EDGE_SAMPLES_M[0], ROLL_BASE_WIDTH_M)))
    print(f"内参 fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
    print(f"h = {h:.4f} m（样本 {H_SAMPLES}，极差 {max(H_SAMPLES)-min(H_SAMPLES):.3f} m）")
    print(f"roll = {roll:.2f}°（左右边缘 {ROLL_EDGE_SAMPLES_M} m，基线宽 {ROLL_BASE_WIDTH_M} m）")
    print(f"地面过滤 v >= {GROUND_V_MIN}，黄色面积下限 {MIN_AREA}\n")

    files = sorted(MARK_DIR.glob("mark_*.png")) + sorted(MARK_DIR.glob("mark_*.jpg"))
    if not files:
        raise SystemExit(f"{MARK_DIR} 里没有 mark_*.png。\n"
                         f"命名规则：mark_2.0_2.5_3.0_3.7.png（把每个标记的实测距离写进文件名，顺序随意）")

    rows = []
    for p in files:
        ds = parse_distances(p.stem)
        img = cv2.imread(str(p))
        if img is None:
            print("读不了:", p.name); continue
        cands, mask = find_marks(img)
        ground = [c for c in cands if c[2] >= GROUND_V_MIN]
        ground.sort(key=lambda c: -c[2])
        print(f"{p.name}: 文件名里的距离 {ds}")
        print(f"  黄色块共 {len(cands)} 个，地面区域 {len(ground)} 个")
        for c in cands:
            tag = "地面" if c[2] >= GROUND_V_MIN else "已过滤(太高，非地面)"
            print(f"    面积={c[0]:5.0f} 中心=({c[1]:5.0f},{c[2]:5.0f}) 尺寸={c[3]}x{c[4]} "
                  f"填充率={c[5]:.2f}  {tag}")
        if len(ds) != len(ground):
            print(f"  !! 距离 {len(ds)} 个 vs 地面标记 {len(ground)} 个 不匹配，跳过这张")
            continue
        for D, c in zip(ds, ground):
            u, v = c[1], c[2]
            und = cv2.undistortPoints(np.array([[[u, v]]], np.float32), K, dist, P=K)
            u2, v2 = float(und[0, 0, 0]), float(und[0, 0, 1])
            pitch = np.degrees(np.arctan(h / D) - np.arctan((v2 - cy) / fy))
            rows.append((p.name, D, u2, v2, c[0], pitch))
            print(f"    D={D:.2f}m -> v={v2:.1f}  pitch={pitch:.3f}°")

    if len(rows) < 3:
        raise SystemExit(f"有效标记点只有 {len(rows)} 个，至少需要 3 个")

    ps = np.array([r[5] for r in rows])
    pitch = float(np.median(ps))
    print(f"\npitch 样本 = {np.round(ps,3).tolist()}")
    print(f"pitch 中位数 = {pitch:.3f}°  标准差 = {float(np.std(ps)):.3f}°  极差 = {float(np.ptp(ps)):.3f}°")

    order = sorted(rows, key=lambda r: r[1])
    vs = [r[3] for r in order]
    mono = all(vs[i] > vs[i + 1] for i in range(len(vs) - 1))
    print("单调性:", "OK" if mono else "!! 不单调，检查距离填反了没、或相机被碰过",
          [f"{r[1]:.1f}m->v{r[3]:.0f}" for r in order])

    print("\n用中位数 pitch 预测像素 v，与实测对比:")
    errs = []
    for name, D, u2, v2, a, p in rows:
        v_pred = cy + fy * np.tan(np.arctan(h / D) - np.radians(pitch))
        errs.append(v2 - v_pred)
        print(f"  D={D:.2f}m  实测 v={v2:6.1f}  预测 v={v_pred:6.1f}  差 {v2-v_pred:+6.1f} px")
    errs = np.array(errs)
    print(f"残差: 平均 {errs.mean():+.2f} px  RMS {np.sqrt((errs**2).mean()):.2f} px")

    print("\n距离预测误差（h/pitch 对了才准）:")
    derrs = []
    for name, D, u2, v2, a, p in rows:
        D_pred = h / np.tan(np.radians(pitch) + np.arctan((v2 - cy) / fy))
        derrs.append(D_pred - D)
        print(f"  实测 {D:.2f} m -> 预测 {D_pred:.3f} m，差 {(D_pred-D)*100:+.1f} cm")

    data = {
        "height_m": round(h, 4),
        "height_samples_m": H_SAMPLES,
        "height_method": "卷尺量镜头中心到地面 3 次取平均",
        "pitch_deg": round(pitch, 3),
        "pitch_samples_deg": [round(float(x), 3) for x in ps],
        "pitch_std_deg": round(float(np.std(ps)), 3),
        "pitch_method": "地面黄色胶带块 + 已知距离反解 pitch=atan(h/D)-atan((v-cy)/fy)，v 先去畸变",
        "roll_deg": round(roll, 2),
        "roll_edge_samples_m": ROLL_EDGE_SAMPLES_M,
        "roll_base_width_m": ROLL_BASE_WIDTH_M,
        "roll_method": "atan((右边缘离地-左边缘离地)/相机宽度)，未调平",
        "roll_why_not_fixed": "调平云台会整体转动相机从而改变 pitch，需重贴标记重新反解；"
                              "roll 造成的地面误差 ≈ D*tan(roll)*(u-cx)/fx，画面中心为 0，最角上约 12 cm，"
                              "远小于 50 cm 验收阈值，故保留实测值而不重标",
        "yaw_offset_deg": 0.0,
        "yaw_offset_method": "定义量：载体原点取相机光心，+X 轴取光轴水平投影方向，故 yaw_offset ≡ 0"
                             "（细则 2.9 允许以相机光心作为载体参考原点）",
        "n_marks": len(rows),
        "mark_residual_rms_px": round(float(np.sqrt((errs ** 2).mean())), 2),
        "distance_residual_cm": [round(float(x) * 100, 1) for x in derrs],
        "monotonic_ok": bool(mono),
        "rig": "三脚架，相机夹具固定，采集与标定期间未松动",
        "measured_on": "2026-11-07",
        "source_files": sorted({r[0] for r in rows}),
    }
    OUT.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print("\n写入", OUT)


if __name__ == "__main__":
    main()
