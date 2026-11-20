import cv2 as cv
from pathlib import Path
import numpy as np
import sys
import os
from datetime import datetime
SHOW = os.environ.get("W2_SHOW", "1") != "0"
NAME = "sumaolin"

def stamp(img, tag, msg):
    bar_h = 104
    out = img.copy()
    cv.rectangle(out, (0, 0), (img.shape[1], bar_h), (0, 0, 0), -1)
    out = cv.addWeighted(out, 0.55, img, 0.45, 0)
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv.putText(out, f"name: {NAME}", (24, 44), cv.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 2)
    cv.putText(out, f"time: {now}", (24, 88), cv.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 2)
    cv.putText(out, tag, (780, 44), cv.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
    cv.putText(out, msg, (780, 88), cv.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
    print("姓名:", NAME)
    print(now)
    return out

def imwrite(path, img):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv.imencode(path.suffix, img)
    if not ok:
        raise ValueError(f"编码失败：{path}")
    buf.tofile(str(path))
    print("修改图已保存到:", path)
    test = cv.imdecode(np.fromfile(str(path), np.uint8), cv.IMREAD_COLOR)
    if test.size == 0:
        sys.exit("修改图读取异常")
    print("修改图可读取")

ROOT = Path(__file__).parent.parent
# 目标颜色：足球的白色球面。白色在 HSV 里是 S 低、V 高。
# 草地上的白线也是白色，所以后面两步是必须的：开运算先抹掉细线，面积和长宽比再筛掉残留。
HSV_LO = (0, 0, 120)  # V 下限放低才够得着球底背光的那一圈白，140 时 football_2 球心会偏上 40 px
HSV_HI = (180, 80, 255)
AREA_MIN = 5000      # 最小轮廓面积（像素平方）。football_1 的球 11243，football_3 最大线头 1215，5000 落在中间
OPEN_K = 21          # 开运算核要大于白线宽度（远处白线只有几个像素宽），球面是实心大块不会被吃
CLOSE_K = 9          # 闭运算把球面上黑色五边形造成的洞补回去，让外轮廓贴合球的外圆
AREA_RATIO_MIN = 0.6  # 球的宽高比接近 1，白线是细长条，用这个区间挡掉
AREA_RATIO_MAX = 1.7

def detect(path, tag):
    img = cv.imdecode(np.fromfile(str(path), np.uint8), cv.IMREAD_COLOR)
    if img.size == 0:
        sys.exit(f"{tag} not found")
    h, w = img.shape[:2]
    print(f"\n========== {tag} ==========")
    print("读取到", tag, img.shape)

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, HSV_LO, HSV_HI)
    print("白色 mask 像素:", int(mask.sum() // 255))
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_ELLIPSE, (OPEN_K, OPEN_K)))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K)))
    mask_vis = stamp(cv.cvtColor(mask, cv.COLOR_GRAY2BGR), tag, "white mask")
    imwrite(ROOT/"outputs"/"task1"/f"{tag}_mask.png", mask_vis)
    if SHOW:                                        # 先看 mask，白线有没有被清干净一眼就知道
        cv.imshow(f"{tag}_mask", mask_vis)
        cv.waitKey(0)

    cnts, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    area_min = AREA_MIN
    cand = []
    for c in cnts:
        area = cv.contourArea(c)
        if area < area_min:
            continue
        x0, y0, w0, h0 = cv.boundingRect(c)
        ratio = w0/h0
        if not (AREA_RATIO_MIN <= ratio <= AREA_RATIO_MAX):   # 球是圆的，外接矩形接近正方形
            print("  非球候选 长宽比 %.2f 面积: %d" % (ratio, int(area)))
            continue
        cand.append((area, c))
    cand.sort(key=lambda ac: ac[0], reverse=True)   # 多个候选时按面积从大到小，取最大
    print(f"轮廓总数: {len(cnts)}, 面积 >= {area_min:.0f} 且长宽比合适的候选: {len(cand)}")

    result = img.copy()
    if not cand:                                    # 没找到就明确报出来，不拿未初始化的坐标往下算
        print("未检测到目标")
        result = stamp(result, tag, "NO TARGET")
        imwrite(ROOT/"outputs"/"task1"/f"{tag}_result.png", result)
        if SHOW:
            cv.imshow(f"{tag}_result", result)
            cv.waitKey(0)
            cv.destroyAllWindows()
        return None

    for a2, _ in cand[1:]:
        print("  忽略较小候选 面积:", int(a2))
    area, cnt = cand[0]
    x, y, bw, bh = cv.boundingRect(cnt)
    m = cv.moments(cnt)
    if m["m00"] > 0:                                # m00 为 0 时退化成外接矩形中心
        u, v = m["m10"]/m["m00"], m["m01"]/m["m00"]
    else:
        u, v = x+bw/2, y+bh/2
    print("目标面积:", int(area), "中心点: u=%.1f, v=%.1f" % (u, v))

    cv.drawContours(result, [cnt], -1, (0, 255, 0), 3)
    cv.rectangle(result, (x, y), (x+bw, y+bh), (0, 255, 255), 2)
    cv.circle(result, (int(u), int(v)), 8, (0, 0, 255), -1)
    cv.putText(result, f"u={u:.0f}, v={v:.0f}", (x, max(210, y-15)), cv.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
    result = stamp(result, tag, "u=%.0f, v=%.0f, area=%d" % (u, v, int(area)))
    imwrite(ROOT/"outputs"/"task1"/f"{tag}_result.png", result)
    if SHOW:
        cv.imshow(f"{tag}_result", result)
        cv.waitKey(0)
        cv.destroyAllWindows()
    return u, v, area

print("姓名:", NAME)
print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
for name in ["football_1", "football_2", "football_3"]:
    detect(ROOT/"data"/f"{name}.jpg", name)
