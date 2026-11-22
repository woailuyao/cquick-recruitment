import cv2
import os

img_path = r"data/images/blueberry.jpg"
img = cv2.imread(img_path)

out_dir = "outputs/task2"
os.makedirs(out_dir, exist_ok=True)

# 1. 缩放
scale = 0.6
resized = cv2.resize(img, None, fx=scale, fy=scale)
# 2. 旋转（保证不被裁剪）
h, w = img.shape[:2]
center = (w//2, h//2)
M = cv2.getRotationMatrix2D(center, 30, 1.0)
rotated = cv2.warpAffine(img, M, (w, h))

# 3. 灰度 + 二值化
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, binary = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)

# 4. 形态学：腐蚀、膨胀
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
erode_img = cv2.erode(binary, kernel)
dilate_img = cv2.dilate(binary, kernel)

# 保存所有中间结果
cv2.imwrite(os.path.join(out_dir, "resized.jpg"), resized)
cv2.imwrite(os.path.join(out_dir, "rotated.jpg"), rotated)
cv2.imwrite(os.path.join(out_dir, "binary.jpg"), binary)
cv2.imwrite(os.path.join(out_dir, "erode.jpg"), erode_img)
cv2.imwrite(os.path.join(out_dir, "dilate.jpg"), dilate_img)

cv2.imshow("resized", resized)
cv2.imshow("rotated", rotated)
cv2.imshow("binary", binary)
cv2.imshow("erode", erode_img)
cv2.imshow("dilate", dilate_img)

print("✅ task2 全部处理完成，图片保存在outputs/task2")
cv2.waitKey(0)
cv2.destroyAllWindows()
