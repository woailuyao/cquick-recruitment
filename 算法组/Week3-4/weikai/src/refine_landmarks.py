"""refine_landmarks.py —— YOLO 框内 L/T 标志几何交点细化（任务书 2.3）

任务书要求："检测框中心不一定等于精确几何交点。学生必须在 YOLO 检测后，
对标志框内图像做几何细化（例如线段拟合、角点/交点计算或局部模板几何），
输出真实的 landmark_pixel=(u,v)"。

本模块实现：在 YOLO bbox 内（含 15% padding）提取白色场地线 mask，
用距离变换的脊峰（白线最"粗"处 = 两条线交汇处）估计真实交点像素；
再用 Shi-Tomasi 角点作辅助修正。任何一步不可靠时回退 bbox 中心，
保证细化不会比基线更差。

取点规则：
- LCross：两条白线拐角的中心交点（拐角顶点）
- TCross：横线与竖线交汇的中心点
- 两者统一为"框内白线距离变换脊峰 + 最近角点"，无需为 L/T 分别调参
"""
import cv2
import numpy as np


def white_line_mask(roi):
    """框内白色场地线 mask：低饱和 + 亮度阈值，闭运算连接断口。

    注意：不能用 3x3 开运算——框内白线常只有 5~10 px 宽，会被腐蚀掉。
    """
    if roi.size == 0:
        return None
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # 白线：低饱和（排除绿草）+ 高于中位亮度。阈值取 Otsu 与中值偏低者，
    # 避免过曝帧 Otsu 偏高漏掉灰白线。
    _, m = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    m = ((m > 0) & (hsv[:, :, 1] < 110)).astype(np.uint8)
    # 小闭运算填断口即可，不做开运算
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    return m


def refine_point(bgr, box, pad=0.15, min_line_px=20, dmin=4.0,
                 mode="ridge", max_shift=0.30, debug=False):
    """细化单个标志的交点像素。

    参数 box: (x1,y1,x2,y2) 浮点 YOLO 框。
    mode: 'ridge'  距离变换脊峰（白线最粗处=交汇中心）
          'hough'  HoughLinesP 两条主方向线段求交（几何最直观）
    返回 (u, v, ok)：ok=True 表示用了几何细化，False 表示回退 bbox 中心。
    """
    x1, y1, x2, y2 = [float(v) for v in box]
    h, w = bgr.shape[:2]
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    bw, bh = x2 - x1, y2 - y1
    if bw < 4 or bh < 4:
        return (cx, cy, False)

    # 框外扩，保证拐角完整包住（bbox 常切掉线的一端）
    xa, xb = max(0, int(x1 - bw * pad)), min(w, int(x2 + bw * pad))
    ya, yb = max(0, int(y1 - bh * pad)), min(h, int(y2 + bh * pad))
    if xb <= xa or yb <= ya:
        return (cx, cy, False)
    roi = bgr[ya:yb, xa:xb]
    m = white_line_mask(roi)
    if m is None:
        return (cx, cy, False)
    npx = int((m > 0).sum())
    if npx < min_line_px:
        return (cx, cy, False)

    # 关键门控：若 bbox 中心已落在白线上，说明标注/检测已把中心对准交点，
    # 无需细化——此时强行找脊峰反而引入偏移、破坏帧间匹配一致性。
    lx, ly = int(round(cx - xa)), int(round(cy - ya))
    if 0 <= lx < m.shape[1] and 0 <= ly < m.shape[0]:
        patch = m[max(0, ly - 2):ly + 3, max(0, lx - 2):lx + 3]
        if patch.any():
            return (cx, cy, False)

    D = cv2.distanceTransform(m, cv2.DIST_L2, 5)
    dmax = float(D.max())
    if dmax < dmin:                     # 线太细/太糊，细化不可信
        return (cx, cy, False)

    rx = ry = None
    if mode == "ridge":
        # 距离变换脊峰：白线内部离边界最远处 = 两条线交汇中心
        peak = D >= max(dmin, 0.75 * dmax)
        ys, xs = np.where(peak)
        if len(xs) == 0:
            return (cx, cy, False)
        wts = D[ys, xs]
        rx = float((xs * wts).sum() / wts.sum())
        ry = float((ys * wts).sum() / wts.sum())
    elif mode == "hough":
        # Hough 线段拟合：取两条最长主方向线段，求其所在直线交点
        seg = cv2.HoughLinesP(m, 1, np.pi / 180, threshold=8,
                              minLineLength=max(8, int(min(bw, bh) * 0.3)),
                              maxLineGap=10)
        if seg is None:
            return (cx, cy, False)
        seg = np.asarray(seg).reshape(-1, 4)   # 兼容 OpenCV 4/5 返回维度
        if len(seg) < 2:
            return (cx, cy, False)
        lines = []
        for s in seg:
            x3, y3, x4, y4 = [float(v) for v in s]
            L = np.hypot(x4 - x3, y4 - y3)
            ang = np.arctan2(y4 - y3, x4 - x3)
            lines.append((L, ang, x3, y3, x4, y4))
        lines.sort(reverse=True)
        # 取最长的一条，再找与其方向夹角 >20° 的最长一条
        a = lines[0]
        b = None
        for cand in lines[1:]:
            da = abs(np.degrees(cand[1] - a[1])) % 180
            if 20 < da < 160:
                b = cand
                break
        if b is None:
            return (cx, cy, False)
        # 两条无限长直线求交
        def inter(l1, l2):
            x1_, y1_, x2_, y2_ = l1[2:]
            x3_, y3_, x4_, y4_ = l2[2:]
            A = np.array([[y2_ - y1_, -(x2_ - x1_)],
                          [y4_ - y3_, -(x4_ - x3_)]], float)
            B = np.array([(y2_ - y1_) * x1_ - (x2_ - x1_) * y1_,
                          (y4_ - y3_) * x3_ - (x4_ - x3_) * y3_], float)
            if abs(np.linalg.det(A)) < 1e-6:
                return None
            return np.linalg.solve(A, B)
        p = inter(a, b)
        if p is None:
            return (cx, cy, False)
        rx, ry = float(p[0]), float(p[1])

    if rx is None:
        return (cx, cy, False)
    u, v = xa + rx, ya + ry
    # 兜底：细化点离 bbox 中心过远说明跑偏，回退
    lim = max_shift * max(bw, bh)
    if np.hypot(u - cx, v - cy) > lim:
        return (cx, cy, False)
    return (u, v, True)


def detect_yolo_refined(bgr, weights, conf=0.25, imgsz=640,
                        refine=True, enhance=None, mode="ridge",
                        max_shift=0.30, blend=1.0):
    """YOLO 检测 + 框内细化，返回 [(x, y, type)]。

    与 locator.detect_yolo 输出格式一致，可直接替换主循环的检测后端。
    """
    from ultralytics import YOLO
    if weights not in _CACHE:
        _CACHE[weights] = YOLO(weights)
    model = _CACHE[weights]
    if enhance == "clahe":
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
        bgr = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    res = model.predict(bgr, imgsz=imgsz, conf=conf, verbose=False)[0]
    names = {0: "L", 1: "T"}
    cand = []
    for bb in res.boxes:
        cls = int(bb.cls[0])
        cf = float(bb.conf[0])
        box = [float(v) for v in bb.xyxy[0]]
        cx0, cy0 = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
        if refine:
            u, v, ok = refine_point(bgr, box, mode=mode, max_shift=max_shift)
            # blend: 1.0 全用细化点；<1 与 bbox 中心加权，抑制细化偏移
            if ok and blend < 1.0:
                u = blend * u + (1 - blend) * cx0
                v = blend * v + (1 - blend) * cy0
        else:
            u, v = cx0, cy0
        cand.append((cf, u, v, names.get(cls, "L")))
    cand.sort(reverse=True)
    out = []
    for _, x, y, t in cand:
        if not any(tt == t and (x - x2) ** 2 + (y - y2) ** 2 < 35 ** 2
                   for x2, y2, tt in out):
            out.append((x, y, t))
    return out


_CACHE = {}
