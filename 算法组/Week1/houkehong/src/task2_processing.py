import cv2
import os

# 路径设置
img_path = "../data/images/test1.jpg"  # 选一张目标清晰的图，如果叫别的名字请修改这里
output_dir = "../outputs/task2"
os.makedirs(output_dir, exist_ok=True)

# 读取图片
img = cv2.imread(img_path)
if img is None:
    print(f"❌ 找不到图片 {img_path}，请检查路径！")
    exit()

# 1. 转换为灰度图
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite(os.path.join(output_dir, "01_gray.jpg"), gray)
print("已保存：01_gray.jpg")

# 2. 高斯滤波对比 (参数影响：核大小 kernel_size)
# 设置两组不同的核大小
gaussian_5 = cv2.GaussianBlur(gray, (5, 5), 0)
gaussian_15 = cv2.GaussianBlur(gray, (15, 15), 0)
cv2.imwrite(os.path.join(output_dir, "02_gaussian_5x5.jpg"), gaussian_5)
cv2.imwrite(os.path.join(output_dir, "03_gaussian_15x15.jpg"), gaussian_15)
print("已保存：02 和 03 高斯滤波对比图")

# 3. 阈值分割对比 (参数影响：固定阈值大小)
# 使用滤波后的图做阈值，效果更好。这里用 127 (中等) 和 50 (偏低) 做对比
_, thresh_127 = cv2.threshold(gaussian_5, 127, 255, cv2.THRESH_BINARY)
_, thresh_50 = cv2.threshold(gaussian_5, 50, 255, cv2.THRESH_BINARY)
cv2.imwrite(os.path.join(output_dir, "04_thresh_127.jpg"), thresh_127)
cv2.imwrite(os.path.join(output_dir, "05_thresh_50.jpg"), thresh_50)
print("已保存：04 和 05 阈值分割对比图")

# 4. Canny 边缘检测
# 使用两组高低阈值进行对比
edges_1 = cv2.Canny(gaussian_5, 50, 150)
edges_2 = cv2.Canny(gaussian_5, 100, 200)
cv2.imwrite(os.path.join(output_dir, "06_canny_low.jpg"), edges_1)
cv2.imwrite(os.path.join(output_dir, "07_canny_high.jpg"), edges_2)
print("已保存：06 和 07 Canny边缘对比图")

# 5. 形态学处理 (解决断裂或噪点问题)
# 定义一个 5x5 的矩形结构元素
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
# 开运算 (先腐蚀后膨胀)：通常用于去除背景上的小噪点
opening = cv2.morphologyEx(thresh_127, cv2.MORPH_OPEN, kernel)
# 闭运算 (先膨胀后腐蚀)：通常用于连接断裂的区域或填补目标内部的小孔
closing = cv2.morphologyEx(thresh_127, cv2.MORPH_CLOSE, kernel)

cv2.imwrite(os.path.join(output_dir, "08_morph_open.jpg"), opening)
cv2.imwrite(os.path.join(output_dir, "09_morph_close.jpg"), closing)
print("已保存：08 和 09 形态学处理图")

print(f"\n🎉 任务二运行完毕！请去 {output_dir} 查看这 9 张图片。")
