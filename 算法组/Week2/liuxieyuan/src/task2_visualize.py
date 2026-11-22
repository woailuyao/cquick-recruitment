import cv2
import numpy as np
from datetime import datetime

print("====== 任务二：可视化与参数对比 开始运行 ======")
now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
watermark = f"Name: liuxieyuan  Time: {now}"
print(f"当前时间戳: {now}")

# 读取真实水果图
img = cv2.imread('test3.jpg')
if img is None:
    print("❌ 找不到 test3.jpg，请确保图片在目录下！")
    exit()

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, np.array([35, 43, 46]), np.array([85, 255, 255]))
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# ==========================================
# 参数 A：面积阈值 500（正常画框）
# ==========================================
print("\n--- [参数 A] 最小面积阈值设为: 500 ---")
img_a = img.copy()
cv2.putText(img_a, watermark, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
count_a = 0

for c in contours:
    if cv2.contourArea(c) > 500:
        count_a += 1
        M = cv2.moments(c)
        u, v = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(img_a, (x, y), (x+w, y+h), (0, 0, 255), 2) 
        cv2.circle(img_a, (u, v), 5, (255, 0, 0), -1) 
        cv2.putText(img_a, f"u={u}, v={v}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2) 
        
cv2.imwrite('task2_paramA.png', img_a)
print(f"✅ 参数 A 运行完毕：共检测到 {count_a} 个目标，已生成标注图 task2_paramA.png")


# ==========================================
# 参数 B：面积阈值 1000000（强制漏检）
# ==========================================
print("\n--- [参数 B] 最小面积阈值设为: 1000000 ---")
img_b = img.copy()
cv2.putText(img_b, watermark, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
count_b = 0

for c in contours:
    if cv2.contourArea(c) > 1000000: 
        count_b += 1

if count_b == 0:
    print("❌ 参数 B 运行完毕：未检测到满足面积 > 1000000 的目标（发生漏检）。")
    cv2.putText(img_b, "Not Detected (Area > 1000000)", (50, int(img.shape[0]/2)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

cv2.imwrite('task2_paramB.png', img_b)
print("已生成漏检对比图 task2_paramB.png")
print("\n====== 任务二运行结束 ======")
