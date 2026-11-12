import cv2
import os

# 使用杯子图片
img_path = "data/images/test1.jpg"  
os.makedirs("outputs/task2", exist_ok=True)

img = cv2.imread(img_path)
if img is None:
    print("❌ 读取失败！")
    exit()

cv2.imwrite("outputs/task2/01_original.jpg", img)

# 1. 转灰度图
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite("outputs/task2/02_gray.jpg", gray)

# 2. 高斯模糊（上滤镜）
# 注意这个 (25, 25)，数字越大越模糊！
gaussian = cv2.GaussianBlur(gray, (35, 35), 0)
cv2.imwrite("outputs/task2/03_gaussian_blur.jpg", gaussian)

# 3. 阈值分割（只留纯黑纯白）
# 127是个分界线，大于127的变白，小于的变黑
ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
cv2.imwrite("outputs/task2/04_threshold.jpg", thresh)

# 4. Canny 边缘检测（描边）
edges = cv2.Canny(gray, 50, 150)
cv2.imwrite("outputs/task2/05_canny_edges.jpg", edges)
# 6. 形态学处理：开运算（去除小噪点）
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
morph_open = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
cv2.imwrite("outputs/task2/06_morphology_open.jpg", morph_open)

# 7. 形态学处理：闭运算（填补小黑洞）
morph_close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
cv2.imwrite("outputs/task2/07_morphology_close.jpg", morph_close)

print("🎉 任务二执行完毕！请去 outputs/task2 看这 5 张图，区别巨大！")