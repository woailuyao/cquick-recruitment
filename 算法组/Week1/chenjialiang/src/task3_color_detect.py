import cv2
import numpy as np

# 三张图都已经缩到 1280 宽，这里不用再缩
img1 = cv2.imread("data/images/小金鱼.jpg")
img2 = cv2.imread("data/images/大肥鱼.jpg")
img3 = cv2.imread("data/images/bird.jpg")

# 转 HSV
hsv1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
hsv2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
hsv3 = cv2.cvtColor(img3, cv2.COLOR_BGR2HSV)

# 掩膜范围：橙色鱼身 / 深蓝头发 / 白色孔雀
lower1 = np.array([0, 100, 80])
upper1 = np.array([40, 255, 255])
lower2 = np.array([100, 50, 10])
upper2 = np.array([135, 255, 255])
lower3 = np.array([0, 0, 120])
upper3 = np.array([179, 60, 255])

mask1 = cv2.inRange(hsv1, lower1, upper1)
mask2 = cv2.inRange(hsv2, lower2, upper2)
mask3 = cv2.inRange(hsv3, lower3, upper3)

print(f"掩膜白占比  小金鱼={(mask1 == 255).mean() * 100:.1f}%  "
      f"大肥鱼={(mask2 == 255).mean() * 100:.1f}%  bird={(mask3 == 255).mean() * 100:.1f}%")

# 形态学：先开再闭
kernel = np.ones((3, 3), np.uint8)
mask1 = cv2.morphologyEx(cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
mask2 = cv2.morphologyEx(cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
mask3 = cv2.morphologyEx(cv2.morphologyEx(mask3, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)

# 找轮廓
contours1, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours3, _ = cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

print(f"轮廓数  小金鱼={len(contours1)}  大肥鱼={len(contours2)}  bird={len(contours3)}")


def remove_noise(contours, n):
    # 按面积从大到小排，保留前 n 个
    return sorted(contours, key=cv2.contourArea, reverse=True)[:n]


contours1 = remove_noise(contours1, n=1)
contours2 = remove_noise(contours2, n=2)   # 头发主体 + 头顶呆毛
contours3 = remove_noise(contours3, n=1)

# 用保留下来的轮廓重新画一张干净的掩膜
mask1_clean = np.zeros_like(mask1)
cv2.drawContours(mask1_clean, contours1, -1, 255, -1)
mask2_clean = np.zeros_like(mask2)
cv2.drawContours(mask2_clean, contours2, -1, 255, -1)
mask3_clean = np.zeros_like(mask3)
cv2.drawContours(mask3_clean, contours3, -1, 255, -1)

cv2.imwrite("outputs/task3/mask1.png", mask1_clean)
cv2.imwrite("outputs/task3/mask2.png", mask2_clean)
cv2.imwrite("outputs/task3/mask3.png", mask3_clean)


# 画外接矩形和中心点，顺便打印坐标和面积
def point(image, kept, name):
    canvas = image.copy()
    for i, c in enumerate(kept):
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)

        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(canvas, (cx, cy), 6, (0, 0, 255), -1)
        cv2.circle(canvas, (cx, cy), 2, (255, 255, 255), -1)

        text = f"({cx}, {cy}), Area={int(area)}"
        tx, ty = x + 5, (y + 22 if y < 40 else y - 10)
        cv2.putText(canvas, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(canvas, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        print(f"{name} 目标{i}: 中心=({cx}, {cy})  面积={area:.0f}  "
              f"外接矩形=({x}, {y}, {w}, {h})  宽高比={w / h:.2f}")

    return canvas


point1 = point(img1, contours1, "小金鱼")
point2 = point(img2, contours2, "大肥鱼")
point3 = point(img3, contours3, "bird")

cv2.imwrite("outputs/task3/result1.png", point1)
cv2.imwrite("outputs/task3/result2.png", point2)
cv2.imwrite("outputs/task3/result3.png", point3)
