"""从实时录屏里筛「可能点出 >=4 个白线交点」的候选帧，供本人目视点选。

红线
----
* 只做**候选帧筛选**，不检测/不生成/不猜测任何观测点；
  点选必须本人在 pick_points.py 里目视完成。
* 抽帧仅用于**离线误差评估**（第 3 组 C0 测试），
  **绝不作为实时演示证据**，也不改变原 mp4（未剪切、未拼接）。
* 打分是启发式（清晰度 + 白线线段丰富度 + 下半屏绿色草地占比），
  不是几何解算结果，打印的 score 只是排序依据，**不等于可见交点数**。
  草地占比用来滤掉天花板灯管、白色衣物 / 围挡遮挡这类「假线」帧。

用法（PowerShell 单行，整行复制粘贴）
--------------------------------------
  python tools/find_demo_frames.py --video check_out/demo_20261120_151004.mp4
  python tools/find_demo_frames.py --video <新录屏.mp4> --top 8 --step 15
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def left_rgb(frame):
    """1920x540 等超宽分屏 -> 取左半 RGB；否则整帧。"""
    h, w = frame.shape[:2]
    if w / h > 2.5:
        return frame[:, : w // 2].copy()
    return frame.copy()


def score_frame(rgb):
    """返回 (分数, 清晰度, 长线段数, 角度簇数, 草地占比)，仅用于排序。"""
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())

    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    h, w = gray.shape

    # 绿色草地占比（只看下半屏）：天花板、白色衣物、白墙、围挡都会被判低
    grass = ((hsv[:, :, 0] > 30) & (hsv[:, :, 0] < 90) &
             (hsv[:, :, 1] > 55) & (hsv[:, :, 2] > 45)).astype(np.uint8)
    green_ratio = float(grass[int(h * 0.30):, :].mean())

    # 白线掩膜：高亮度、低饱和；只统计画面下半（压制天花板/围挡/白墙）
    mask = ((hsv[:, :, 2] > 165) & (hsv[:, :, 1] < 70)).astype(np.uint8) * 255
    mask[: int(h * 0.28), :] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    edges = cv2.Canny(mask, 60, 180)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=38,
                            minLineLength=70, maxLineGap=12)
    long_lines, angles = 0, []
    if lines is not None:
        for ln in lines[:, 0]:
            x1, y1, x2, y2 = map(int, ln)
            length = float(np.hypot(x2 - x1, y2 - y1))
            if length < 80:
                continue
            long_lines += 1
            ang = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
            angles.append(ang)
    # 角度按 30 度分桶，桶数越多说明横/竖/斜线越丰富
    bins = set(int(a // 30) for a in angles)
    n_bins = len(bins)
    score = ((long_lines * 1.0 + n_bins * 4.0 +
              min(sharp, 300) / 30.0) * (0.3 + green_ratio))
    return score, sharp, long_lines, n_bins, green_ratio


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", required=True, help="实时录屏 mp4（check_out/demo_*.mp4）")
    ap.add_argument("--step", type=int, default=15, help="每隔多少帧采样一次")
    ap.add_argument("--top", type=int, default=8, help="输出候选帧数量")
    ap.add_argument("--min-gap", type=int, default=120,
                    help="相邻候选帧最小帧号间隔（保证机位有变化）")
    ap.add_argument("--min-sharp", type=float, default=45.0,
                    help="清晰度下限，滤掉失焦帧")
    ap.add_argument("--min-green", type=float, default=0.22,
                    help="下半屏草地占比下限，滤掉天花板/白色衣物遮挡帧")
    args = ap.parse_args()

    vp = (ROOT / args.video).resolve()
    if not vp.exists():
        raise SystemExit(f"找不到视频：{vp}")
    cap = cv2.VideoCapture(str(vp))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    print(f"视频 {vp.name} | {n} 帧 | {fps:.1f} fps | {n/fps:.1f} s")
    if n <= 0:
        raise SystemExit("视频帧数为 0")

    scored = []
    for fid in range(0, n, args.step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fid)
        ok, fr = cap.read()
        if not ok:
            continue
        rgb = left_rgb(fr)
        sc, sharp, nl, nb, gr = score_frame(rgb)
        if sharp < args.min_sharp or gr < args.min_green:
            continue
        scored.append((sc, fid, sharp, nl, nb, gr))
    cap.release()
    scored.sort(reverse=True)
    print(f"采样 {len(range(0, n, args.step))} 帧，清晰度+草地占比达标 {len(scored)} 帧")

    picked = []
    for item in scored:
        fid = item[1]
        if len(picked) >= args.top:
            break
        if any(abs(fid - p[1]) < args.min_gap for p in picked):
            continue
        picked.append(item)
    if not picked:
        raise SystemExit("没有达标的候选帧；请降低 --min-sharp / --min-green 或改 --step 重跑")

    stem = vp.stem
    out_dir = ROOT / "check_out"
    out_dir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(vp))
    tiles = []
    print("\n候选帧（按分数降序，机位已错开；score 只是排序量，不是交点数）：")
    for sc, fid, sharp, nl, nb, gr in picked:
        cap.set(cv2.CAP_PROP_POS_FRAMES, fid)
        ok, fr = cap.read()
        if not ok:
            continue
        rgb = left_rgb(fr)
        out = out_dir / f"demo_pose_{stem}_f{fid:06d}.jpg"
        cv2.imwrite(str(out), rgb)
        tile = cv2.resize(rgb, (480, 270))
        cv2.putText(tile, f"f{fid}  t={fid/fps:.1f}s", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(tile, f"lines={nl} grass={gr:.2f} sharp={sharp:.0f}", (8, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 1)
        tiles.append(tile)
        print(f"  f{fid:6d}  t={fid/fps:6.1f}s  score={sc:6.1f}  "
              f"sharp={sharp:6.0f}  lines={nl:2d}  bins={nb}  grass={gr:.2f}  -> {out.name}")
    cap.release()

    cols = 3
    rows = (len(tiles) + cols - 1) // cols
    while len(tiles) < rows * cols:
        tiles.append(np.zeros((270, 480, 3), np.uint8))
    sheet = np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols])
                       for r in range(rows)])
    contact = out_dir / f"demo_pose_{stem}_contact.jpg"
    cv2.imwrite(str(contact), sheet)
    print(f"\n拼图：{contact}")

    print("\n下一步（本人目视点选，认不准就按 s 跳过，凑不到 4 点就记为无效状态）：")
    for item in picked:
        fid = item[1]
        tag = f"live_{stem}_f{fid:06d}"
        img = f"check_out/demo_pose_{stem}_f{fid:06d}.jpg"
        print(f"  python tools/pick_points.py --image {img} --tag {tag}")
    print("\n点选是离线误差评估用；实时演示证据仍是原 mp4，两者不要混用。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
