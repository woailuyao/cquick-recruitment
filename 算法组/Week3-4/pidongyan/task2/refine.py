"""白线交点的亚像素细化：YOLO 框只给到几个像素的精度，
任务二几何解算需要亚像素交点。策略（由简到繁，逐级回退）：

  1. penalty_point（孤立圆点）-> 白色像素质心，最稳。
  2. L/T/X 交点 -> 对白色 mask 做 RANSAC 双直线拟合（第一条线收内点，
     剩余像素拟合第二条线），两线交点即交点。比 Hough 抗噪且亚像素化。
  3. 回退 -> 框内白色像素质心。
任何细化结果若偏离 YOLO 框中心超过 guard_px，则视为细化失败，退回框中心。
"""
import cv2
import numpy as np


def _white_mask(gray, box, margin=6):
    x1, y1, x2, y2 = [int(v) for v in box]
    h, w = gray.shape
    x1, y1 = max(0, x1 - margin), max(0, y1 - margin)
    x2, y2 = min(w, x2 + margin), min(h, y2 + margin)
    if x2 - x1 < 4 or y2 - y1 < 4:
        return None, (0, 0, 0, 0)
    crop = gray[y1:y2, x1:x2]
    # 白线亮、草地暗：Otsu 全局二值化足够（场地光照均匀）
    _, mask = cv2.threshold(crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return mask, (x1, y1, x2, y2)


def _centroid(mask, off):
    ys, xs = np.nonzero(mask)
    if len(xs) < 5:
        return None
    return float(xs.mean() + off[0]), float(ys.mean() + off[1])


def _fit_one_line(pts, inlier_dist=3.0, min_inliers=25, iters=200, rng=None):
    """RANSAC 拟合单条直线。返回 (p, d, inlier_mask) 或 None。"""
    n = len(pts)
    if n < min_inliers:
        return None
    best = None
    for _ in range(iters):
        i, j = rng.integers(0, n, 2)
        if i == j:
            continue
        p, q = pts[i], pts[j]
        d = q - p
        L = np.hypot(d[0], d[1])
        if L < 8:
            continue
        d = d / L
        r = pts - p
        dist = np.abs(r[:, 0] * d[1] - r[:, 1] * d[0])
        inl = dist < inlier_dist
        cnt = int(inl.sum())
        if cnt < min_inliers:
            continue
        if best is None or cnt > best[0]:
            # 用内点做最小二乘精化方向（total least squares）
            P = pts[inl]
            c = P.mean(axis=0)
            _, _, V = np.linalg.svd(P - c, full_matrices=False)
            best = (cnt, c, V[0], inl)
    if best is None:
        return None
    _, c, d, inl = best
    return c, d, inl


def _ransac_intersection(mask, off, center_uv, max_px=25.0):
    """白色 mask 双直线拟合，返回两线交点（绝对像素）。"""
    ys, xs = np.nonzero(mask)
    if len(xs) < 40:
        return None
    rng = np.random.default_rng(0)
    pts = np.column_stack([xs, ys]).astype(np.float64)

    line1 = _fit_one_line(pts, rng=rng)
    if line1 is None:
        return None
    _, _, inl1 = line1
    rest = pts[~inl1]
    line2 = _fit_one_line(rest, min_inliers=20, rng=rng)
    if line2 is None:
        # 两条线重叠（如短线段贴一起）：退回第一条线的最近点
        return None

    (c1, d1, _), (c2, d2, _) = line1, line2
    A = np.array([d1, -d2]).T                  # 2x2
    det = np.linalg.det(A)
    if abs(det) < 1e-9:                        # 平行
        return None
    t = np.linalg.solve(A, c2 - c1)
    inter = c1 + t[0] * d1
    px, py = inter + np.array(off[:2])

    # 合理性：交点须在 crop 内且贴近白色像素（排除延长线虚交）
    mx, my = int(round(inter[0])), int(round(inter[1]))
    if not (0 <= my < mask.shape[0] and 0 <= mx < mask.shape[1]):
        return None
    near = mask[max(0, my - 2):my + 3, max(0, mx - 2):mx + 3]
    if near.size and near.mean() < 128:
        return None
    if np.hypot(px - center_uv[0], py - center_uv[1]) > max_px:
        return None
    return float(px), float(py)


def refine_junction(gray, cls_name, box, guard_px=8.0):
    """细化一个交点。box=[x1,y1,x2,y2]，返回 (u, v, method)。"""
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    mask, off = _white_mask(gray, box)
    if mask is None:
        return cx, cy, "box_center"

    if cls_name == "penalty_point":
        c = _centroid(mask, off)
        if c and abs(c[0] - cx) <= guard_px and abs(c[1] - cy) <= guard_px:
            return c[0], c[1], "dot_centroid"
        return cx, cy, "box_center"

    # L/T/X：RANSAC 双线交点
    inter = _ransac_intersection(mask, off, (cx, cy))
    if inter is not None and (abs(inter[0] - cx) <= guard_px
                              and abs(inter[1] - cy) <= guard_px):
        return inter[0], inter[1], "line_intersection"

    c = _centroid(mask, off)
    if c and abs(c[0] - cx) <= guard_px and abs(c[1] - cy) <= guard_px:
        return c[0], c[1], "mask_centroid"
    return cx, cy, "box_center"
