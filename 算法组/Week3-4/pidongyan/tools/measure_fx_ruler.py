"""平行板直测 fx：用已知尺寸的棋盘格 + 卷尺距离直接算焦距，10 分钟定性呼吸效应。

原理：
  棋盘格平面与像面平行、距离 D（卷尺量镜头到板面）时，像素尺寸与实际尺寸之比就是 f/D：
      fx ≈ (横向角点跨度_px / 实际宽度_m) * D
      fy ≈ (纵向角点跨度_px / 实际高度_m) * D
  同时 横向跨度/纵向跨度 应等于 实际宽/实际比，偏差大说明板子没摆正（倾斜），
  结果偏小 —— 会给出平行度警告。

用法（对焦呼吸实验：固定对焦不动，只改距离拍 3 组）：
  python tools/measure_fx_ruler.py --image xxx.jpg --distance 2.0
  python tools/measure_fx_ruler.py --image xxx.jpg --distance 0.5 --tag near
  python tools/measure_fx_ruler.py --dir calibration/fx_ruler/2m --distance 2.0

建议流程（每档 3 张）：
  1) 打印的棋盘格贴硬板（docs/checkerboard_A4_9x6_20mm.pdf）
  2) 相机对焦锁死（与拍场地时同一状态，全程别碰对焦环！）
  3) 卷尺量「镜头中心 → 棋盘格表面」距离 D，误差 ±5mm 内
  4) 板尽量与镜头垂直（板边缘与画面边缘平行），棋盘格居中，占画面 1/3~2/3
  5) 拍照 -> 本工具 -> 记录 fx/fy/平行度
  6) 换距离重复。fx 随距离明显变化 = 呼吸效应实锤，取场地工作距离（2~4m）那档的值
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_intrinsics(path):
    d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(d["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
    return K, dist


def measure_one(img_path, pattern, square, D, K, dist, out_dir):
    gray = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        print(f"  [跳过] 读不了 {img_path}")
        return None
    ok, corners = cv2.findChessboardCorners(gray, pattern, None)
    if not ok:
        print(f"  [跳过] 检不出棋盘格 {img_path.name}")
        return None
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
    # 去畸变后再量跨度（消除镜头畸变对跨度的膨胀）
    und = cv2.undistortPoints(corners, K, dist)          # 归一化坐标
    f_ref = K[0, 0]
    pts = und.reshape(-1, 2) * f_ref + K[:2, 2].reshape(1, 2)  # 折回像素（等比，跨度比例不变）
    cols, rows = pattern
    grid = pts.reshape(rows, cols, 2)
    # 用中间三行的横向跨度平均（抗单行噪声），和中间三列的纵向跨度
    row_spans = [(grid[r, -1, 0] - grid[r, 0, 0]) for r in range(rows // 3, rows * 2 // 3 + 1)]
    col_spans = [(grid[-1, c, 1] - grid[0, c, 1]) for c in range(cols // 3, cols * 2 // 3 + 1)]
    span_u = float(np.mean(row_spans))
    span_v = float(np.mean(col_spans))
    W = (cols - 1) * square
    H = (rows - 1) * square
    fx = span_u / W * D
    fy = span_v / H * D
    ratio_img = span_u / span_v
    ratio_real = W / H
    tilt_err = abs(ratio_img / ratio_real - 1) * 100
    print(f"  {img_path.name}: fx={fx:7.1f}  fy={fy:7.1f}  "
          f"宽高比偏差={tilt_err:4.1f}%  跨度=({span_u:.0f},{span_v:.0f})px")
    if tilt_err > 3:
        print("    ⚠️ 平行度差（>3%）：板子歪了，fx 偏小，重拍这张")
    if span_u < 200:
        print("    ⚠️ 棋盘格太小：跨度 <200px 时 fx 误差被放大，板子离近点或拍清楚些")
    # 标注图
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.drawChessboardCorners(vis, pattern, corners, ok)
    out_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_dir / f"det_{img_path.name}"), vis)
    return fx, fy, tilt_err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", nargs="*", help="棋盘格照片")
    ap.add_argument("--dir", help="或整个目录")
    ap.add_argument("--distance", type=float, required=True, help="镜头到板面距离(m)，卷尺实测")
    ap.add_argument("--pattern", default="9x6", help="内角点数，默认 9x6")
    ap.add_argument("--square", type=float, default=0.020, help="方格边长(m)，默认 20mm")
    ap.add_argument("--intrinsics", default=str(ROOT / "config" / "camera_intrinsics.yaml"))
    ap.add_argument("--tag", default=None)
    args = ap.parse_args()

    try:
        cols, rows = [int(v) for v in args.pattern.lower().split("x")]
    except ValueError:
        sys.exit("--pattern 格式应为 宽x高，如 9x6")
    K, dist = load_intrinsics(args.intrinsics)

    paths = []
    if args.image:
        paths += [Path(p) for p in args.image]
    if args.dir:
        paths += sorted(Path(args.dir).glob("*.jpg")) + sorted(Path(args.dir).glob("*.png"))
    if not paths:
        sys.exit("需要 --image 或 --dir")
    tag = args.tag or f"{args.distance}m"

    print(f"距离 D={args.distance} m，方格 {args.square*1000:.0f} mm，内角点 {cols}x{rows}"
          f"（实际测量宽度 {(cols-1)*args.square:.3f} m × 高 {(rows-1)*args.square:.3f} m）")
    res = []
    for p in paths:
        r = measure_one(p, (cols, rows), args.square, args.distance, K, dist,
                        ROOT / "check_out" / f"fx_ruler_{tag}")
        if r:
            res.append(r)
    if not res:
        sys.exit("没有一张图检出棋盘格")
    fxs = [r[0] for r in res]
    fys = [r[1] for r in res]
    bad = sum(1 for r in res if r[2] > 3)
    print(f"\n=== 距离 {args.distance} m 汇总（{len(res)} 张，其中 {bad} 张平行度差） ===")
    print(f"fx = {np.mean(fxs):.1f} ± {np.std(fxs):.1f}")
    print(f"fy = {np.mean(fys):.1f} ± {np.std(fys):.1f}")
    print("对比基准：近距标定 fx=2031 / 远距标定 fx=1888 / 位姿反推 1600~1750")
    print("把这一行的 fx 记下来，换个距离再跑一轮，两档差 >3% 即确认呼吸效应。")


if __name__ == "__main__":
    main()
