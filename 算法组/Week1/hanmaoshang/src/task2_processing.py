import cv2
import numpy as np
import os

# 获取路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
output_dir = os.path.join(project_root, "outputs", "task2")
os.makedirs(output_dir, exist_ok=True)

# 读图函数（防中文路径报错）
def cv_imread(file_path):
    return cv2.imdecode(np.fromfile(file_path, dtype=np.uint8), -1)

# 读图函数（防中文路径保存报错）
def cv_imwrite(file_path, img):
    cv2.imencode('.jpg', img)[1].tofile(file_path)

# 选一张图来做实验（建议用 image1.jpg）
img_path = os.path.join(project_root, "data", "images", "image1.jpg")
img = cv_imread(img_path)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

cv_imwrite(os.path.join(output_dir, "00_original.jpg"), img)
cv_imwrite(os.path.join(output_dir, "01_gray.jpg"), gray)

# ================= 1. 高斯滤波（平滑去噪） =================
# 对比不同 kernel 大小（必须是奇数）
blur_3 = cv2.GaussianBlur(gray, (3, 3), 0)
blur_9 = cv2.GaussianBlur(gray, (9, 9), 0)
cv_imwrite(os.path.join(output_dir, "02_blur_3x3.jpg"), blur_3)
cv_imwrite(os.path.join(output_dir, "03_blur_9x9.jpg"), blur_9)

# ================= 2. 阈值分割（黑白分明） =================
# 在模糊后的图上做阈值，对比不同阈值大小
_, thresh_low = cv2.threshold(blur_3, 80, 255, cv2.THRESH_BINARY)
_, thresh_high = cv2.threshold(blur_3, 160, 255, cv2.THRESH_BINARY)
cv_imwrite(os.path.join(output_dir, "04_thresh_80.jpg"), thresh_low)
cv_imwrite(os.path.join(output_dir, "05_thresh_160.jpg"), thresh_high)

# ================= 3. Canny 边缘检测 =================
# 对比高低阈值
edges_low = cv2.Canny(blur_3, 50, 150)
edges_high = cv2.Canny(blur_3, 100, 200)
cv_imwrite(os.path.join(output_dir, "06_canny_50_150.jpg"), edges_low)
cv_imwrite(os.path.join(output_dir, "07_canny_100_200.jpg"), edges_high)

# ================= 4. 形态学处理（开/闭运算） =================
# 对低阈值图进行开运算（去噪点）和闭运算（填孔洞）
kernel = np.ones((5, 5), np.uint8)
opened = cv2.morphologyEx(thresh_low, cv2.MORPH_OPEN, kernel)
closed = cv2.morphologyEx(thresh_low, cv2.MORPH_CLOSE, kernel)
cv_imwrite(os.path.join(output_dir, "08_open_5x5.jpg"), opened)
cv_imwrite(os.path.join(output_dir, "09_close_5x5.jpg"), closed)

print("任务二运行结束！生成了10张对比图，快去 outputs/task2 看看吧！")