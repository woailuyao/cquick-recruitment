import cv2
import numpy as np

# 三张测试图，每张检测其目标颜色（蓝/绿/橙）。
# 每个元素：(文件名, HSV 下界 (H,S,V), HSV 上界 (H,S,V))
targets = [
    ('test_blue_circle.jpg',     (100, 50, 50),   (130, 255, 255)),
    ('test_green_rect.jpg',      (35, 50, 50),    (85, 255, 255)),
    ('test_orange_triangle.jpg', (10, 100, 100),  (25, 255, 255)),
]
MIN_AREA = 500  # 轮廓面积小于此值的当作噪声丢弃

for fname, lower, upper in targets:
    img = cv2.imread('data/images/' + fname)
    assert img is not None, '图片读取失败: ' + fname

    # 步骤1：BGR -> HSV，inRange 生成二值掩膜（白=目标，黑=背景）
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))

    # 步骤2：开运算改善掩膜（先腐蚀后膨胀，去掉细小噪点）
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 步骤2：findContours 查找轮廓（OpenCV 4/5 返回 2 个值：轮廓、层级）
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 步骤3：在原图副本上画外接矩形和中心点，并输出坐标/面积
    result = img.copy()
    base = fname.split('.')[0]
    found = False
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue  # 面积太小，当噪声过滤掉
        found = True
        x, y, w, h = cv2.boundingRect(c)          # 左上角 (x,y) + 宽高 (w,h)
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cx = x + w // 2
        cy = y + h // 2
        cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)  # 红点标中心
        print(f'{fname}: 中心=({cx},{cy})  面积={area:.0f}')

    if not found:
        print(f'{fname}: 未检测到足够大的目标轮廓（可能 HSV 范围或光照不匹配）')

    # 步骤3：保存掩膜图和标注结果图
    cv2.imwrite('outputs/task3/' + base + '_mask.png', mask)
    cv2.imwrite('outputs/task3/' + base + '_result.png', result)
    print('---')
