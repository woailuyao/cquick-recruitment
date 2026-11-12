import os

import cv2
import numpy as np

# 白色足球：只卡亮度。白色在阴影里会带环境色，S 能从 35 涨到 200 左右，所以不限制饱和度
LOWER = np.array([0, 0, 160])
UPPER = np.array([179, 255, 255])

MIN_RATIO = 0.002                     # 最小面积 = 全图 × 0.2%
K_OPEN = np.ones((5, 5), np.uint8)
K_CLOSE = np.ones((15, 15), np.uint8)  # 要 15x15 才填得掉球面上的黑色花纹


def detect(path, name):
    """返回 (u, v, area)，未检测到返回 None"""
    img = cv2.imread(path)
    if img is None:
        print(f"[{name}] 读不到图片: {path}")
        return None

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)

    mask_pro = cv2.morphologyEx(mask, cv2.MORPH_OPEN, K_OPEN)
    mask_pro = cv2.morphologyEx(mask_pro, cv2.MORPH_CLOSE, K_CLOSE)

    contours, _ = cv2.findContours(mask_pro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    he, we = img.shape[:2]
    min_areas = MIN_RATIO * he * we
    print(f"[{name}] 白占比 {(mask == 255).mean() * 100:.1f}%  轮廓 {len(contours)} 个")

    kept = [c for c in contours if cv2.contourArea(c) > min_areas]
    if not kept:
        print(f"[{name}] 未检测到目标")
        return None

    # 取面积最大的那个
    cnt = max(kept, key=cv2.contourArea)
    area = cv2.contourArea(cnt)

    M = cv2.moments(cnt)
    if M["m00"] == 0:
        print(f"[{name}] 未检测到目标（轮廓面积为 0）")
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    x, y, bw, bh = cv2.boundingRect(cnt)

    cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
    cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
    cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)
    cv2.circle(img, (cx, cy), 2, (255, 255, 255), -1)

    # 白字黑边，浅色和深色背景都能看清
    text = f"({cx},{cy})"
    ty = y + 22 if y < 40 else y - 8
    cv2.putText(img, text, (x + 5, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
    cv2.putText(img, text, (x + 5, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    print(f"[{name}] u={cx}  v={cy}  面积={area:.0f}  框=({x},{y},{bw},{bh})  宽高比={bw / bh:.2f}")

    os.makedirs("outputs", exist_ok=True)
    cv2.imwrite(f"outputs/{name}_result.png", img)

    return cx, cy, area


if __name__ == "__main__":
    files = ["ball1_clear.jpg", "ball2_similar_bg.jpg", "ball3_small.jpg"]

    ok = 0
    for f in files:
        if detect("data/" + f, f[:-4]) is not None:
            ok += 1

    print(f"\n共 {len(files)} 张，检出 {ok} 张")
