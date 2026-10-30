import os

import cv2
import numpy as np

# ================================================================
# 改参数改这里
# ================================================================
IMAGE = "data/test1_clear.png"    # 对比用哪张图

CASE_A = dict(                    # A 组（基线）
    lower=(35, 80, 60),           # HSV 下界
    upper=(85, 255, 255),         # HSV 上界
    min_area=500,                 # 最小面积
    morph="open",                 # 形态学
    ksize=5,
)

CASE_B = dict(                    # B 组（实验组）
    lower=(52, 145, 100),         # ← 例：收紧 HSV 下界（H 35→52、S 80→145、V 60→100）
    upper=(85, 255, 255),         #    作用：把"偏灰的绿草地"排除掉
    min_area=500,
    morph="open",
    ksize=5,
)

DESC = "只收紧 HSV 下界：H 35→52、S 80→145、V 60→100，看偏灰的草地会不会被排除"  # 说明文字
# ================================================================

# ---------------- 工具函数（与 run_detect.py 相同） ----------------
def imread_utf8(path):
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_utf8(path, img):
    ext = os.path.splitext(path)[1]
    ok, buf = cv2.imencode(ext, img)
    if ok:
        buf.tofile(path)


# ---------------- 核心算法（与 run_detect.py 完全相同，保证可比） ----------------
def build_mask(bgr, lower, upper, morph, ksize):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array(lower, dtype=np.uint8),
                       np.array(upper, dtype=np.uint8))
    if morph not in ("none", "", None) and ksize > 0:
        kernel = np.ones((ksize, ksize), np.uint8)
        if "open" in morph:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if "close" in morph:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect(bgr, lower, upper, min_area, morph, ksize):
    mask = build_mask(bgr, lower, upper, morph, ksize)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in contours:
        a = cv2.contourArea(c)
        if a >= min_area:
            candidates.append((a, c))
    candidates.sort(key=lambda item: item[0], reverse=True)

    if not candidates:
        return dict(detected=False, u=0.0, v=0.0, area=0.0,
                    contour=None, bbox=(0, 0, 0, 0), mask=mask, n_candidates=0)

    area, best = candidates[0]
    m = cv2.moments(best)
    if m["m00"] > 0:
        u = m["m10"] / m["m00"]
        v = m["m01"] / m["m00"]
    else:
        x, y, w, h = cv2.boundingRect(best)
        u, v = x + w / 2.0, y + h / 2.0

    x, y, w, h = cv2.boundingRect(best)
    return dict(detected=True, u=float(u), v=float(v), area=float(area),
                contour=best, bbox=(x, y, w, h), mask=mask,
                n_candidates=len(candidates))


def draw_result(bgr, det, title=""):
    img = bgr.copy()
    if det["detected"]:
        cv2.drawContours(img, [det["contour"]], -1, (0, 255, 0), 2)
        x, y, w, h = det["bbox"]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 1)
        cu, cv_ = int(round(det["u"])), int(round(det["v"]))
        cv2.drawMarker(img, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(img, (cu, cv_), 4, (255, 0, 0), -1)
        text = f"u={cu}, v={cv_}, area={int(det['area'])}"
        ty = max(24, y - 10)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        cv2.rectangle(img, (x, ty - th - 6), (x + tw + 6, ty + 6), (0, 0, 0), -1)
        cv2.putText(img, text, (x, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2, cv2.LINE_AA)
    else:
        cv2.putText(img, "NO TARGET DETECTED", (18, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
    if title:
        cv2.putText(img, title, (10, img.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)
    return img


# ---------------- 主程序 ----------------
if __name__ == "__main__":
    image = imread_utf8(IMAGE)
    if image is None:
        print(f"[ERROR] 图片读取失败: {IMAGE}")
        raise SystemExit(1)

    print(f"=== 参数对比实验 ===")
    print(f"说明: {DESC}")
    print(f"图片: {IMAGE}  ({image.shape[1]}x{image.shape[0]})")
    print()

    det_a = detect(image, CASE_A["lower"], CASE_A["upper"],
                   CASE_A["min_area"], CASE_A["morph"], CASE_A["ksize"])
    det_b = detect(image, CASE_B["lower"], CASE_B["upper"],
                   CASE_B["min_area"], CASE_B["morph"], CASE_B["ksize"])

    # ---- 打印 A/B 配置与结果 ----
    for tag, p, det in (("A", CASE_A, det_a), ("B", CASE_B, det_b)):
        print(f"[配置 {tag}] lower={p['lower']}  upper={p['upper']}  "
              f"min_area={int(p['min_area'])}  morph={p['morph']}")
        if det["detected"]:
            print(f"         → 检测到   u={det['u']:7.1f}  v={det['v']:7.1f}  "
                  f"area={det['area']:9.0f}  候选数={det['n_candidates']}")
        else:
            print("         → 未检测到 （通过面积阈值的轮廓数=0）")

    # ---- 差异分析（结论可直接抄进 README）----
    print()
    if det_a["detected"] and det_b["detected"]:
        du = det_b["u"] - det_a["u"]
        dv = det_b["v"] - det_a["v"]
        ratio = det_b["area"] / det_a["area"] if det_a["area"] > 0 else 0.0
        print(f"[差异] area {det_a['area']:.0f} → {det_b['area']:.0f}（×{ratio:.2f}）；"
              f"中心点偏移 du={du:+.1f}px dv={dv:+.1f}px")
        if ratio > 2.0 or ratio < 0.5:
            print("[结论] 面积变化超过 2 倍 —— 被检区域发生了本质变化：")
            print("       多半是某个配置把『背景/大片同色区域』也算成了目标（误检），")
            print("       或把目标的一部分切掉了（轮廓不完整）。")
        elif abs(du) > 5 or abs(dv) > 5:
            print("[结论] 面积接近但中心点漂移明显 —— 轮廓形状被改变，质心跟着移动。")
        else:
            print("[结论] 面积与中心点都基本稳定 —— 该参数对定位结果影响很小。")
        if det_a["n_candidates"] != det_b["n_candidates"]:
            print(f"[结论] 候选数量 {det_a['n_candidates']} → {det_b['n_candidates']}，"
                  f"说明该参数在影响『有几个东西被当成目标』。")
    elif det_a["detected"] and not det_b["detected"]:
        print("[结论] A 检测到、B 没检测到 → B 的参数过严，属于【漏检】。")
    elif not det_a["detected"] and det_b["detected"]:
        print("[结论] A 没检测到、B 检测到 → A 的参数过严，属于【漏检】。")
    else:
        print("[结论] 两种配置都没检测到 → 目标颜色/光照与 HSV 阈值不匹配，")
        print("       或 min_area 设得比目标本身还大。")

    # ---- 拼对比图（左 A 右 B）并保存 ----
    def label(img, text):
        out = img.copy()
        cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(out, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 0), 1, cv2.LINE_AA)
        return out

    img_a = label(draw_result(image, det_a),
                  f"A: {CASE_A['lower']}~{CASE_A['upper']} minArea={int(CASE_A['min_area'])} {CASE_A['morph']}")
    img_b = label(draw_result(image, det_b),
                  f"B: {CASE_B['lower']}~{CASE_B['upper']} minArea={int(CASE_B['min_area'])} {CASE_B['morph']}")
    gap = np.full((image.shape[0], 8, 3), 255, np.uint8)
    compare = np.hstack([img_a, gap, img_b])

    stem = os.path.splitext(os.path.basename(IMAGE))[0]
    os.makedirs("out", exist_ok=True)
    out_path = f"out/compare_my_{stem}.png"
    imwrite_utf8(out_path, compare)
    print(f"[INFO] 对比图已保存: {out_path}")
