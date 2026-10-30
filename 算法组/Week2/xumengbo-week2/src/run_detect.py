import os

import cv2
import numpy as np
IMAGE = "data/real_football_grass.png"      # 输入图片
LOWER = (35, 80, 60)                # HSV 下界 (H, S, V)
UPPER = (100, 255, 255)              # HSV 上界 (H, S, V)
MIN_AREA = 500                      # 最小轮廓面积（像素²），小于它的当噪点
MORPH = "open"                      # 形态学：none / open / close / open+close
KSIZE = 5                           # 形态学核大小（奇数：3、5、7…）
SAVE_MASK = True                    # True=同时保存二值掩膜，False=只存结果图
def imread_utf8(path):
    """读图：cv2.imread 对中文路径会失败，用这个兼容。"""
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_utf8(path, img):
    """写图：兼容中文路径。"""
    ext = os.path.splitext(path)[1]
    ok, buf = cv2.imencode(ext, img)
    if ok:
        buf.tofile(path)


def build_mask(bgr, lower, upper, morph, ksize):
    """BGR 图 → HSV → 阈值 → 形态学 → 二值掩膜（白色=目标色区域）。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array(lower, dtype=np.uint8),
                       np.array(upper, dtype=np.uint8))
    if morph not in ("none", "", None) and ksize > 0:
        kernel = np.ones((ksize, ksize), np.uint8)
        if "open" in morph:    # 开运算：先腐蚀后膨胀 → 去掉小噪点
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if "close" in morph:   # 闭运算：先膨胀后腐蚀 → 补目标内部的洞
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect(bgr, lower, upper, min_area, morph, ksize):
    """完整检测，返回字典。没有目标时 detected=False（绝不返回假坐标）。"""
    mask = build_mask(bgr, lower, upper, morph, ksize)

    # 找外轮廓（RETR_EXTERNAL 只取最外层，避免目标内部纹理被当成多个目标）
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    # 面积筛选：>= min_area 才留下，按面积从大到小排序（任务书 1.2）
    candidates = []
    for c in contours:
        a = cv2.contourArea(c)
        if a >= min_area:
            candidates.append((a, c))
    candidates.sort(key=lambda item: item[0], reverse=True)

    if not candidates:
        return dict(detected=False, u=0.0, v=0.0, area=0.0,
                    contour=None, bbox=(0, 0, 0, 0), mask=mask, n_candidates=0)

    # 取面积最大的那个作为目标（多候选时的选择规则）
    area, best = candidates[0]

    # 中心点：轮廓矩 m10/m00（x 均值）、m01/m00（y 均值）
    m = cv2.moments(best)
    if m["m00"] > 0:                 # m00 = 面积，为 0 时退化处理
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
    """把轮廓、外接矩形、中心点、坐标文字画回原图。"""
    img = bgr.copy()

    if det["detected"]:
        cv2.drawContours(img, [det["contour"]], -1, (0, 255, 0), 2)      # 绿色轮廓
        x, y, w, h = det["bbox"]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 1)     # 黄色矩形
        cu, cv_ = int(round(det["u"])), int(round(det["v"]))
        cv2.drawMarker(img, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)  # 红色十字
        cv2.circle(img, (cu, cv_), 4, (255, 0, 0), -1)                    # 蓝色圆点

        text = f"u={cu}, v={cv_}, area={int(det['area'])}"
        ty = max(24, y - 10)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        cv2.rectangle(img, (x, ty - th - 6), (x + tw + 6, ty + 6), (0, 0, 0), -1)  # 黑底条
        cv2.putText(img, text, (x, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2, cv2.LINE_AA)                # 白字
    else:
        cv2.putText(img, "NO TARGET DETECTED", (18, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)

    if title:
        cv2.putText(img, title, (10, img.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)
    return img


if __name__ == "__main__":
    image = imread_utf8(IMAGE)
    if image is None:                                     # 读图失败检查（任务书要求）
        print(f"[ERROR] 图片读取失败: {IMAGE}")
        print("        检查路径是否存在、文件是否为图片。")
        raise SystemExit(1)

    h, w = image.shape[:2]
    print(f"[INFO] 图像   : {IMAGE}")
    print(f"[INFO] 尺寸   : {w} x {h}（宽 x 高）")
    print(f"[INFO] HSV 阈值: lower={LOWER} upper={UPPER}")
    print(f"[INFO] 最小面积: {MIN_AREA}   形态学: {MORPH} (k={KSIZE})")

    det = detect(image, LOWER, UPPER, MIN_AREA, MORPH, KSIZE)

    if det["detected"]:
        print(f"[OK]   检测到目标: u={det['u']:.1f}, v={det['v']:.1f}, "
              f"area={det['area']:.0f}, 候选数={det['n_candidates']}")
        half_w, half_h = w / 2.0, h / 2.0
        if abs(det["u"] - half_w) < w * 0.05 and abs(det["v"] - half_h) < h * 0.05:
            where = "正中"
        else:
            where = ("左" if det["u"] < half_w else "右") + ("上" if det["v"] < half_h else "下")
        print(f"       中心点位于图像{where}"
              f"（横向 {det['u'] / w * 100:.1f}% ，纵向 {det['v'] / h * 100:.1f}%）")
    else:
        print("[WARN] 未检测到目标：没有轮廓的面积超过 min-area")   # 明确的失败提示

    # 保存标注结果
    stem = os.path.splitext(os.path.basename(IMAGE))[0]
    os.makedirs("out", exist_ok=True)

    out_path = f"out/{stem}_result.png"
    annotated = draw_result(
        image, det,
        title=f"lower={LOWER} upper={UPPER} minArea={MIN_AREA} {MORPH}")
    imwrite_utf8(out_path, annotated)
    print(f"[INFO] 结果图已保存: {out_path}")

    if SAVE_MASK and det["mask"] is not None:
        mask_path = f"out/{stem}_result_mask.png"
        imwrite_utf8(mask_path, det["mask"])
        print(f"[INFO] 掩膜已保存  : {mask_path}")
