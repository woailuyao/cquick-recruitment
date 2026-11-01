"""拍照自检工具：判断一张图能不能做颜色分割（任务三）

用法:
    python src/check_photo.py <图片路径> [--show]

判定思路:
    1. 取画面外圈 8% 的区域当"背景样本"
    2. 背景几乎没颜色(S<=110) -> 目标 = 所有高饱和像素中最大的连通块
    3. 背景本身有颜色 -> 目标 = 色相与背景相差 35 以上(约 70 度)的连通块
    4. 目标面积 >= 2% 且饱和度 >= 110 才算合格

注意: OpenCV 5.0 在 Windows 上读不了中文路径，图片请放英文目录。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

from io_utils import imread, imwrite

BASE = Path(__file__).resolve().parents[1]
DEFAULT = BASE / "data" / "images" / "obj3.jpg"
MAX_SIDE = 800
SAT_MIN = 110
MIN_AREA_RATIO = 0.02

HUE_NAMES = {
    0: "红", 10: "橙", 20: "黄橙", 30: "黄", 40: "黄绿", 50: "绿",
    60: "绿", 70: "青绿", 80: "青", 90: "青蓝", 100: "蓝", 110: "蓝",
    120: "蓝紫", 130: "紫", 140: "紫红", 150: "品红", 160: "红", 170: "红",
}


def hue_name(h):
    return HUE_NAMES.get((int(h) // 10) * 10, "?")


def hue_dist(a, b):
    """色相在 0-179 上是环形的，0 和 179 相邻"""
    d = abs(a - b)
    return min(d, 180 - d)


def largest_blob(mask):
    """返回 (最大连通块面积, 该块包围框, 该块中心)"""
    m = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    if n <= 1:
        return 0, None, None, m
    idx = int(np.argmax(stats[1:, 4])) + 1
    x, y, w, h, area = stats[idx]
    return int(area), (int(x), int(y), int(w), int(h)), (int(cent[idx][0]), int(cent[idx][1])), m


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = Path(args[0]) if args else DEFAULT

    img = imread(str(path))
    if img is None:
        print(f"读取失败: {path}")
        print("若路径含中文，OpenCV 无法读取，请先复制到 data/images/ 下")
        sys.exit(1)

    h, w = img.shape[:2]
    scale = MAX_SIDE / max(h, w)
    if scale < 1:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    hh, ww = H.shape
    total = hh * ww

    print(f"文件: {path.name}")
    print(f"原始尺寸: {w}x{h}   处理尺寸: {ww}x{hh}")
    print(f"全图中位: H={int(np.median(H))} S={int(np.median(S))} V={int(np.median(V))}")

    # ---------- 背景采样：外圈 8% ----------
    m = max(8, int(min(hh, ww) * 0.08))
    border = np.zeros((hh, ww), bool)
    border[:m, :] = True
    border[-m:, :] = True
    border[:, :m] = True
    border[:, -m:] = True

    bg_sat = border & (S > SAT_MIN)
    bg_sat_ratio = bg_sat.sum() / max(1, border.sum())
    print(f"\n[背景样本] 外圈 {m} 像素，其中高饱和(S>{SAT_MIN})占 {bg_sat_ratio * 100:.1f}%")

    sat = S > SAT_MIN
    print(f"[全图] 高饱和像素占 {sat.mean() * 100:.1f}%")

    if bg_sat_ratio < 0.15:
        # 背景基本无彩色 -> 直接用高饱和连通块找目标
        bg_mode = "neutral"
        print("[判定依据] 背景接近中性色（白墙/灰桌/黑布），分割应该比较容易")
        print("           目标 = 所有高饱和像素中最大的连通块")
        cand = sat
    else:
        bg_mode = "colored"
        bh = H[border & sat]
        bg_bin = int(np.bincount((bh.astype(np.int32) // 10)).argmax())
        bg_hue = bg_bin * 10 + 5
        print(f"[判定依据] 背景本身有颜色，主色相 H≈{bg_hue} ({hue_name(bg_hue)})")
        print(f"           目标 = 与背景色相相差 35 以上（约 70 度）的连通块")
        cand = sat & (np.abs(H.astype(np.int32) - bg_hue) >= 35) & \
               ((180 - np.abs(H.astype(np.int32) - bg_hue)) >= 35)

    # ---------- 找候选目标 ----------
    area, box, center, mask = largest_blob((cand * 255).astype(np.uint8))
    print(f"\n[候选目标] 最大连通块面积 {area} 像素，占画面 {area / total * 100:.2f}%")
    if area == 0:
        print("\n结论: 不合格 —— 画面里没有任何鲜明颜色的物体")
        print("建议: 换一个饱和度高的物体（蓝色/绿色/红色塑料件），放在白纸或白墙前拍")
        return

    sel = mask > 0
    hm = int(np.median(H[sel]))
    sm = int(np.median(S[sel]))
    vm = int(np.median(V[sel]))
    print(f"           中位 H={hm} ({hue_name(hm)})  S={sm}  V={vm}")
    print(f"           外接框 {box}   中心 {center}")

    # 多目标检查
    m2 = cv2.morphologyEx((cand * 255).astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m2 = cv2.morphologyEx(m2, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(m2)
    similar = [int(a) for a in stats[1:, 4] if a > area * 0.4]
    print(f"           同量级连通块数量: {len(similar)}")

    # ---------- 判定 ----------
    print("\n[判定]")
    ok = True
    ratio = area / total
    if ratio < MIN_AREA_RATIO:
        print(f"  ✗ 目标太小: {ratio * 100:.2f}% < {MIN_AREA_RATIO * 100:.0f}%（拍近一点，或让物体占画面 1/5 以上）")
        ok = False
    else:
        print(f"  ✓ 目标面积 {ratio * 100:.2f}%，足够")
    if sm < SAT_MIN:
        print(f"  ✗ 目标饱和度 S={sm} 偏低（换更鲜艳的物体，或换白光）")
        ok = False
    else:
        print(f"  ✓ 目标饱和度 S={sm}，足够")
    if bg_sat_ratio > 0.5:
        print(f"  ✗ 背景 S>110 占 {bg_sat_ratio * 100:.1f}%，背景太花，容易误检")
        ok = False
    else:
        print(f"  ✓ 背景干扰较小")
    if len(similar) >= 3:
        print(f"  ! 画面里有 {len(similar)} 个同色物体，会同时检出多个目标")
        print("    若只想检一个，把其他同色物体移出画面")

    if ok:
        # 只用"离中位色相 45 以内"的像素估 H 范围，避免边缘杂色把范围拉歪
        hsel = H[sel].astype(np.int32)
        near = np.array([hue_dist(int(v), hm) < 45 for v in hsel])
        hcore = hsel[near] if near.any() else hsel
        h_lo = max(0, int(np.percentile(hcore, 2)) - 5)
        h_hi = min(179, int(np.percentile(hcore, 98)) + 5)
        # S / V 放宽一些，保证目标暗部不被切掉
        s_lo = max(45, int(np.percentile(S[sel], 5)) - 60)
        v_lo = max(30, int(np.percentile(V[sel], 2)) - 50)
        print(f"\n[建议参数] lower=({h_lo}, {s_lo}, {v_lo})  upper=({h_hi}, 255, 255)")
        print(f"            min_area={int(area * 0.3)}")
        print("\n结论: ✓ 可以用于任务三")
    else:
        print("\n结论: ✗ 不建议使用，换一张")

    if "--show" in sys.argv:
        vis = img.copy()
        if box:
            x, y, bw2, bh2 = box
            cv2.rectangle(vis, (x, y), (x + bw2, y + bh2), (0, 255, 0), 2)
        cv2.namedWindow("check", cv2.WINDOW_NORMAL)
        cv2.imshow("check", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


