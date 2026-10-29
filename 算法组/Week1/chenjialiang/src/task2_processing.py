import cv2
import numpy as np

img = cv2.imread('data/images/大肥鱼.jpg')

# 灰度图
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite('outputs/task2/大肥鱼-灰度图.jpg', img_gray)

# 高斯滤波：两个核对比，都从灰度图出发
Gaussian_1 = cv2.GaussianBlur(img_gray, (5, 5), 0)
Gaussian_2 = cv2.GaussianBlur(img_gray, (15, 15), 0)
cv2.imwrite('outputs/task2/大肥鱼-高斯滤波-5x5.jpg', Gaussian_1)
cv2.imwrite('outputs/task2/大肥鱼-高斯滤波-15x15.jpg', Gaussian_2)

# 阈值分割：127 和 180 对比，都基于 (5,5) 那张
ret1, thresh11 = cv2.threshold(Gaussian_1, 127, 255, cv2.THRESH_BINARY)
ret2, thresh12 = cv2.threshold(Gaussian_1, 180, 255, cv2.THRESH_BINARY)
cv2.imwrite(f'outputs/task2/大肥鱼-二值图-阈值{int(ret1)}.jpg', thresh11)
cv2.imwrite(f'outputs/task2/大肥鱼-二值图-阈值{int(ret2)}.jpg', thresh12)

# Canny：两组双阈值
canny11 = cv2.Canny(Gaussian_1, 50, 100)
canny12 = cv2.Canny(Gaussian_1, 150, 250)
cv2.imwrite('outputs/task2/大肥鱼-canny-低.jpg', canny11)
cv2.imwrite('outputs/task2/大肥鱼-canny-高.jpg', canny12)

# 形态学：开运算和闭运算，都基于阈值 127 那张二值图
kernel = np.ones((5, 5), np.uint8)
opening = cv2.morphologyEx(thresh11, cv2.MORPH_OPEN, kernel)
closing = cv2.morphologyEx(thresh11, cv2.MORPH_CLOSE, kernel)
cv2.imwrite('outputs/task2/大肥鱼-开运算.jpg', opening)
cv2.imwrite('outputs/task2/大肥鱼-闭运算.jpg', closing)

print(f"灰度均值 {img_gray.mean():.1f}")
print(f"阈值 {int(ret1)} 白占比 {(thresh11 == 255).mean() * 100:.1f}%")
print(f"阈值 {int(ret2)} 白占比 {(thresh12 == 255).mean() * 100:.1f}%")
print(f"Canny(50,100) 边缘像素 {int((canny11 == 255).sum())}")
print(f"Canny(150,250) 边缘像素 {int((canny12 == 255).sum())}")
print(f"开运算白像素 {int((opening == 255).sum())}  闭运算白像素 {int((closing == 255).sum())}")
