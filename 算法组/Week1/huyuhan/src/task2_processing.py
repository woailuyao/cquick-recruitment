import cv2 as cv
import sys
gray = cv.imread("data/images/test2.jpg", cv.IMREAD_GRAYSCALE)
if gray is None:
    sys.exit("图片读取失败，请检查路径")
blur5 = cv.GaussianBlur(gray, (5, 5), 0)
blur9 = cv.GaussianBlur(gray, (9, 9), 0)
sigma1 = cv.GaussianBlur(gray, (9, 9), 1)
sigma2 = cv.GaussianBlur(gray, (9, 9), 2)
saved_gray = cv.imwrite("outputs/task2/01_gray.png", gray)
saved_blur5 = cv.imwrite("outputs/task2/02_gaussian5.png", blur5)
saved_blur9 = cv.imwrite("outputs/task2/03_gaussian9.png", blur9)
saved_sigma1 = cv.imwrite("outputs/task2/gaussian9_sigma1.png", sigma1)
saved_sigma2 = cv.imwrite("outputs/task2/gaussian9_sigma2.png", sigma2)

print("灰度图保存成功：", saved_gray)
print("5×5高斯滤波图保存成功：", saved_blur5)
print("9×9高斯滤波图保存成功：", saved_blur9)
print("9×9、标准差1保存成功：", saved_sigma1)
print("9×9、标准差2保存成功：", saved_sigma2)

ret, binary = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
ret, binary_inv = cv.threshold(gray, 127, 255, cv.THRESH_BINARY_INV)
ret, trunc = cv.threshold(gray, 127, 255, cv.THRESH_TRUNC)
ret, tozero = cv.threshold(gray, 127, 255, cv.THRESH_TOZERO)
ret, tozero_inv = cv.threshold(gray, 127, 255, cv.THRESH_TOZERO_INV)
adaptive_mean = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C,cv.THRESH_BINARY, 11, 2)
adaptive_gaussian = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)
otsu_threshold, otsu = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

print("普通二值化保存成功：", cv.imwrite("outputs/task2/threshold_binary.png", binary))
print("反向二值化保存成功：", cv.imwrite("outputs/task2/threshold_binary_inv.png", binary_inv))
print("截断阈值保存成功：", cv.imwrite("outputs/task2/threshold_trunc.png", trunc))
print("低值归零保存成功：", cv.imwrite("outputs/task2/threshold_tozero.png", tozero))
print("高值归零保存成功：", cv.imwrite("outputs/task2/threshold_tozero_inv.png", tozero_inv))
print("自适应均值阈值保存成功：", cv.imwrite("outputs/task2/threshold_adaptive_mean.png", adaptive_mean))
print("自适应高斯阈值保存成功：", cv.imwrite("outputs/task2/threshold_adaptive_gaussian.png", adaptive_gaussian))
print("Otsu阈值保存成功：", cv.imwrite("outputs/task2/threshold_otsu.png", otsu))
print("Otsu自动计算的阈值：", otsu_threshold)

ret80, binary80 = cv.threshold(gray, 80, 255, cv.THRESH_BINARY)
ret140, binary140 = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)

print("阈值80保存成功：", cv.imwrite("outputs/task2/threshold80.png", binary80))
print("阈值140保存成功：", cv.imwrite("outputs/task2/threshold140.png", binary140))

edges = cv.Canny(blur5, 50, 150)
saved_edges = cv.imwrite("outputs/task2/06_canny.png", edges)
print("Canny边缘图保存成功：", saved_edges)

opening = cv.morphologyEx(binary80, cv.MORPH_OPEN, None)
closing = cv.morphologyEx(binary80, cv.MORPH_CLOSE, None)

saved_opening = cv.imwrite("outputs/task2/07_open.png", opening)
saved_closing = cv.imwrite("outputs/task2/08_close.png", closing)

print("开运算保存成功：", saved_opening)
print("闭运算保存成功：", saved_closing)