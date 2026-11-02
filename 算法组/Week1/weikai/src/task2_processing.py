import cv2

# 读取图片（目标图：绿矩形）
img = cv2.imread('data/images/test_green_rect.jpg')
assert img is not None, "图片读取失败，检查路径"

# 1) 灰度图
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite('outputs/task2/01_gray.jpg', gray)

# 2) 高斯滤波（2 组参数对比：5x5 vs 15x15）
blur5 = cv2.GaussianBlur(gray, (5, 5), 0)
blur15 = cv2.GaussianBlur(gray, (15, 15), 0)
cv2.imwrite('outputs/task2/02_gaussian_5x5.jpg', blur5)
cv2.imwrite('outputs/task2/02_gaussian_15x15.jpg', blur15)

# 3) 阈值分割（2 组参数对比：80 vs 160）
_, bin80 = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
_, bin160 = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
cv2.imwrite('outputs/task2/03_threshold_80.jpg', bin80)
cv2.imwrite('outputs/task2/03_threshold_160.jpg', bin160)

# 4) Canny 边缘（2 组参数对比：低阈值 30/80 vs 高阈值 100/200）
edges1 = cv2.Canny(gray, 30, 80)
edges2 = cv2.Canny(gray, 100, 200)
cv2.imwrite('outputs/task2/04_canny_30_80.jpg', edges1)
cv2.imwrite('outputs/task2/04_canny_100_200.jpg', edges2)

# 5) 形态学处理（开运算，作用在边缘图上去除小噪点）
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
opened = cv2.morphologyEx(edges1, cv2.MORPH_OPEN, kernel)
cv2.imwrite('outputs/task2/05_morph_open.jpg', opened)

print("任务二完成：共保存 8 张结果图到 outputs/task2/")
