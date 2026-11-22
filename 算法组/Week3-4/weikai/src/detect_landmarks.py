# -*- coding: utf-8 -*-
"""detect_landmarks.py —— 场地线交点（L/T/X 标志）经典 CV 检测

定位链路第一级：从图像中找出白色场地线的 L/T/X 型交点像素坐标。
用途：
  1. 离线验证定位几何链路（配合 M1-M3 定点视频）
  2. 批量扫描视频产出候选裁剪图，供 YOLO 标注（任务二检测模型训练数据）

方法（直线求交，比骨架化对压缩噪声鲁棒得多）：
  白线分割（近白 + 绿色草地上下文 + 连通域过滤）
  → HoughLinesP 提取长直线段
  → 共线段聚类合并成完整直线
  → 近垂直线对求交点，按"线段端点 vs 交点"关系分类：
      两线都穿过 = X，一线终止 = T，两线都终止 = L
  → 交点局部质心细化 + 去重

用法：
  python detect_landmarks.py --image D:/captures/_frames/M1_mid.png
  python detect_landmarks.py --video D:/captures/M1.mp4 --stride 30
"""
import argparse
import os
import sys

import cv2
import math

import numpy as np


# ---------- 白线分割 ----------
def white_line_mask(bgr, min_component=150, green_ctx=0.15):
    """场地白线提取（对曝光漂移鲁棒）：
    1. 灰度顶帽（原图 - 大核开运算背景）：细亮线相对局部背景突出，
       与整体曝光无关（M3 那种整片偏亮/偏灰的场景也能用）
    2. 绿色草地上下文（HSV 色相宽松判定 + 密度过滤）：排除白墙/球网/立柱
    3. 连通域面积过滤去小噪声"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    # 顶帽核必须大于画面中最粗的白线（近景线宽可达 50-90px，近大远小）：
    # 核 25 时粗线内部对顶帽零响应、只剩边缘碎斑，是近景漏检的根因；
    # 核 91 实测全面最优（暗光近景可检出、纯草地纹理帧被覆盖率升级抑制）
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (91, 91))
    tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, k)
    # 阈值自适应：Otsu 在顶帽图上自动分隔"线/背景"，
    # 暗光近景（全场中位灰度 ~110）与亮光远景都能各得其所；
    # 若 mask 覆盖率过高（草地纹理起反应），继续升阈值直到只剩线
    otsu_th, _ = cv2.threshold(tophat, 0, 255,
                               cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = max(22.0, float(otsu_th))
    _, mask = cv2.threshold(tophat, th, 255, cv2.THRESH_BINARY)
    while float(mask.mean()) / 255.0 > 0.12 and th < 120:
        th *= 1.35
        _, mask = cv2.threshold(tophat, th, 255, cv2.THRESH_BINARY)
    # 白线饱和度低（草地纹理边、阴影斑驳 S 偏高）
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    _, s_ch, _ = cv2.split(hsv)
    mask[s_ch > 60] = 0
    # 白线两侧都是草地；深色接缝带一侧很暗 → 邻近有暗像素的区域剔除。
    # "暗"用相对中位灰度定义：傍晚暗场（中位 ~110）不会全场误杀
    med_v = float(np.median(gray))
    dark = (gray < 0.75 * med_v).astype(np.float32)
    dark_density = cv2.blur(dark, (25, 25))
    mask[dark_density > 0.12] = 0
    h, s2, _ = cv2.split(hsv.astype(np.int16))
    green = ((h > 30) & (h < 95) & (s2 > 12)).astype(np.float32)
    density = cv2.blur(green, (81, 81))
    keep = (density > green_ctx).astype(np.uint8)
    mask = mask * keep

    # 暗光线常碎成小段：先闭运算桥接，再用较低面积门槛去噪
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    for k2 in range(1, n):
        if stats[k2, cv2.CC_STAT_AREA] < min_component:
            mask[labels == k2] = 0
    return mask


# ---------- 直线提取与合并 ----------
def extract_lines(mask, min_len=60, max_gap=14):
    """HoughLinesP + 共线聚类。返回 [(px,py,dx,dy,len,t0,t1)]，
    (px,py) 为线上参考点，(dx,dy) 单位方向，[t0,t1] 参数范围。"""
    segs = cv2.HoughLinesP(mask, 1, np.pi / 360, threshold=40,
                           minLineLength=min_len, maxLineGap=max_gap)
    if segs is None:
        return []
    segs = np.asarray(segs).reshape(-1, 4)
    lines = []
    for x1, y1, x2, y2 in segs:
        # 沿线段采样验证覆盖率（草地纹理拼出的线段会有大段空隙）
        xs = np.linspace(x1, x2, max(8, int(np.hypot(x2 - x1, y2 - y1) / 6)))
        ys2 = np.linspace(y1, y2, len(xs))
        xi = np.clip(xs.astype(int), 0, mask.shape[1] - 1)
        yi = np.clip(ys2.astype(int), 0, mask.shape[0] - 1)
        if mask[yi, xi].mean() < 165:       # 65% 以上命中白线
            continue
        d = np.array([x2 - x1, y2 - y1], float)
        L = float(np.hypot(*d))
        if L < 1:
            continue
        d /= L
        lines.append([np.array([x1, y1], float), d, L,
                      np.array([x1, y1], float), np.array([x2, y2], float)])

    # 共线聚类：角度相近 + 参考点到对方直线的垂距小 → 合并
    used = [False] * len(lines)
    merged = []
    for i in range(len(lines)):
        if used[i]:
            continue
        p_i, d_i, L_i, a_i, b_i = lines[i]
        group_pts = [a_i, b_i]
        for j in range(i + 1, len(lines)):
            if used[j]:
                continue
            p_j, d_j, L_j, _, _ = lines[j]
            cosang = abs(float(d_i @ d_j))
            if cosang < 0.995:            # 夹角 > ~5.7°
                continue
            perp = abs(float(np.cross(d_i, p_j - p_i)))
            if perp > 7.0:
                continue
            group_pts += [lines[j][3], lines[j][4]]
            used[j] = True
        pts = np.array(group_pts)
        # 以最长段的参考点和方向为基准，把所有端点投影到该直线取范围
        c = pts.mean(axis=0)
        d = d_i
        t = (pts - c) @ d
        merged.append((c[0], c[1], d[0], d[1], t.min(), t.max()))
    return merged


# ---------- 交点求取与分类 ----------
def line_intersections(lines, mask, img_size, cluster_r=35):
    """兼容旧接口：返回 [(x, y, type)]。"""
    return [(x, y, t) for x, y, t, _ in
            line_intersections_voted(lines, mask, img_size, cluster_r)]


def _ray_branch_count(mask_d, dt, x, y):
    """从 (x, y) 向四周发射线，数白像素形成的连续方向组数。

    射线长度随局部线宽自适应（粗线用长射线），覆盖率 >=0.7 视为该方向
    有线条延伸。返回方向组数：L 角 = 2，T 交点 = 3，十字 = 4，
    噪声团块 = 1（四周全连通）。越界部分不计入（画面边缘的角不误杀）。
    """
    H, W = mask_d.shape
    d0 = float(dt[min(max(y, 0), H - 1), min(max(x, 0), W - 1)])
    max_r = max(45.0, 3.5 * d0)
    min_r = max(6.0, 0.8 * d0)
    angs = np.deg2rad(np.arange(0.0, 360.0, 3.0))
    rs = np.arange(min_r, max_r, 4.0)
    supported = np.zeros(len(angs), bool)
    for k in range(len(angs)):
        xs = x + rs * math.cos(angs[k])
        ys = y + rs * math.sin(angs[k])
        inb = (xs >= 0) & (xs < W - 1) & (ys >= 0) & (ys < H - 1)
        if inb.sum() < 6:
            continue
        hit = mask_d[ys[inb].astype(int), xs[inb].astype(int)] > 0
        if hit.mean() >= 0.7:
            supported[k] = True
    # 圆周上连续 True 段计数（段至少 3 个采样 = 9°，滤单角度毛刺）
    n_ang = len(angs)
    visited = np.zeros(n_ang, bool)
    groups = 0
    for k in range(n_ang):
        if not supported[k] or visited[k]:
            continue
        j, cnt = k, 0
        while supported[j % n_ang] and not visited[j % n_ang]:
            visited[j % n_ang] = True
            j += 1
            cnt += 1
        if cnt >= 3:
            groups += 1
    return groups


def line_intersections_voted(lines, mask, img_size, cluster_r=35):
    """返回 [(x, y, type, votes)]，type 只有 "L" / "T"。

    本场地只有 L 型（两条线都终止于交点）和 T 型（一条线贯穿、
    另一条终止），不存在 X 型标志。分类不用线段端点参数（Hough 线段
    常越过拐角几十像素，会把 L 误判成 T），而是从交点向四周发射线，
    按掩膜上白像素的实际延伸方向数分类：
      - 2 个方向 → L
      - 3 个方向 → T
      - 其他（1=噪声团块、4=十字交叉）→ 丢弃
    votes = 支持该交点的线段对数，越大越可信（批量标注用它剔单票噪点）。
    """
    H, W = img_size
    mask_d = cv2.dilate(mask, np.ones((5, 5), np.uint8))
    dt = cv2.distanceTransform((mask > 0).astype(np.uint8),
                               cv2.DIST_L2, 3)
    raw = []
    for i in range(len(lines)):
        px, py, dx, dy, t0a, t1a = lines[i]
        for j in range(i + 1, len(lines)):
            qx, qy, ex, ey, t0b, t1b = lines[j]
            cos = abs(dx * ex + dy * ey)
            if cos > 0.95:                # 排除近平行
                continue
            cross = dx * ey - dy * ex
            if abs(cross) < 1e-6:
                continue
            t = ((qx - px) * ey - (qy - py) * ex) / cross
            s = ((qx - px) * dy - (qy - py) * dx) / cross
            X = px + t * dx
            Y = py + t * dy
            if not (2 <= X < W - 2 and 2 <= Y < H - 2):
                continue
            # 交点处要有白像素（容忍 5px 半径内）
            xi, yi = int(X), int(Y)
            win = mask[max(0, yi - 5):yi + 6, max(0, xi - 5):xi + 6]
            if win.size == 0 or win.max() == 0:
                continue
            raw.append((X, Y))

    # 聚类去重：35px 内的投票合并成一个交点，位置取质心
    out = []
    for X, Y in raw:
        hit = None
        for o in out:
            if (X - o[0]) ** 2 + (Y - o[1]) ** 2 < cluster_r ** 2:
                hit = o
                break
        if hit is None:
            out.append([X, Y, 1])
        else:
            hit[2] += 1
            hit[0] = (hit[0] * (hit[2] - 1) + X) / hit[2]
            hit[1] = (hit[1] * (hit[2] - 1) + Y) / hit[2]

    # 质心细化 + 射线分类
    final = []
    for X, Y, n in out:
        xi, yi = int(X), int(Y)
        x0, x1 = max(0, xi - 8), min(W, xi + 9)
        y0, y1 = max(0, yi - 8), min(H, yi + 9)
        sub = mask[y0:y1, x0:x1]
        yy, xx = np.nonzero(sub)
        if len(xx) >= 6:
            xi = int(x0 + xx.mean())
            yi = int(y0 + yy.mean())
        nb = _ray_branch_count(mask_d, dt, xi, yi)
        if nb == 2:
            typ = "L"
        elif nb == 3:
            typ = "T"
        else:
            continue
        final.append((xi, yi, typ, n))
    return final


def detect_voted(bgr, min_votes=2, l_min_votes=1, return_mask=False):
    """完整检测流程，返回 [(x, y, type, votes)]。

    min_votes: T 类最少投票数（一条线贯穿 + 多条线终止，常有多对线投票）
    l_min_votes: L 类最少投票数。L 角只有一对线相交（1 票），
                 用与 T 相同的门槛会把绝大多数真 L 滤掉
    return_mask: True 时返回 (点列表, mask)，供调用方估线宽/做后续处理
    """
    mask = white_line_mask(bgr)
    lines = extract_lines(mask)
    pts = line_intersections_voted(lines, mask,
                                   (bgr.shape[0], bgr.shape[1]))
    out = []
    for x, y, t, v in pts:
        need = l_min_votes if t == "L" else min_votes
        if v >= need:
            out.append((x, y, t, v))
    return (out, mask) if return_mask else out


def annotate(bgr, points):
    vis = bgr.copy()
    for x, y, typ in points:
        color = {"L": (0, 220, 0), "T": (0, 160, 255)}[typ]
        cv2.circle(vis, (x, y), 9, color, 2)
        cv2.putText(vis, typ, (x + 11, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return vis


def process_image(path, out_dir=None, save_crops=False):
    bgr = cv2.imread(path)
    if bgr is None:
        print("读不了:", path)
        return []
    mask = white_line_mask(bgr)
    lines = extract_lines(mask)
    pts = line_intersections(lines, mask, (bgr.shape[0], bgr.shape[1]))
    cnt = {}
    for _, _, t in pts:
        cnt[t] = cnt.get(t, 0) + 1
    print("%s: %d 条线, %d 个交点  %s" % (
        os.path.basename(path), len(lines), len(pts),
        "  ".join("%s x%d" % (k, v) for k, v in sorted(cnt.items()))))
    if out_dir:
        vis = annotate(bgr, pts)
        out = os.path.join(out_dir,
                           os.path.splitext(os.path.basename(path))[0]
                           + "_landmarks.png")
        cv2.imwrite(out, vis)
        if save_crops:
            cdir = os.path.join(out_dir, "crops")
            os.makedirs(cdir, exist_ok=True)
            base = os.path.splitext(os.path.basename(path))[0]
            for k, (x, y, typ) in enumerate(pts):
                r = 60
                x0, x1 = max(0, x - r), min(bgr.shape[1], x + r)
                y0, y1 = max(0, y - r), min(bgr.shape[0], y + r)
                crop = bgr[y0:y1, x0:x1]
                if min(crop.shape[:2]) >= 40:
                    fn = "%s_%s_%02d.png" % (base, typ, k)
                    cv2.imwrite(os.path.join(cdir, fn), crop)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="单张图片路径")
    ap.add_argument("--video", help="视频路径（批量抽帧）")
    ap.add_argument("--stride", type=int, default=30, help="视频抽帧间隔")
    ap.add_argument("--out-dir", default=r"D:\captures\_landmarks")
    ap.add_argument("--save-crops", action="store_true",
                    help="同时保存交点周边裁剪图（标注素材）")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    if args.image:
        process_image(args.image, args.out_dir, args.save_crops)
    elif args.video:
        cap = cv2.VideoCapture(args.video)
        k = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if k % args.stride == 0:
                tmp = os.path.join(args.out_dir, "_tmp.png")
                cv2.imwrite(tmp, frame)
                process_image(tmp, args.out_dir, args.save_crops)
            k += 1
        cap.release()
    else:
        print("需要 --image 或 --video")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
