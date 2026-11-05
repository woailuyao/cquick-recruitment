import cv2
import numpy as np
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
output_dir = os.path.join(project_root, "outputs", "task3")
os.makedirs(output_dir, exist_ok=True)

def cv_imread(file_path):
    return cv2.imdecode(np.fromfile(file_path, dtype=np.uint8), -1)

def cv_imwrite(file_path, img):
    cv2.imencode('.jpg', img)[1].tofile(file_path)

image_paths = [
    os.path.join(project_root, "data", "images", "image1.jpg"),
    os.path.join(project_root, "data", "images", "image2.jpg"),
    os.path.join(project_root, "data", "images", "image3.jpg")
]

for i, path in enumerate(image_paths, 1):
    print(f"\n===== 处理第 {i} 张图片 =====")
    img = cv_imread(path)
    if img is None:
        continue

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # ================= 根据图片自动匹配颜色参数 =================
    if i == 1:
        # 第1张图：浅蓝色/灰青色眼镜盒 (照片发灰，S值低)
        print("目标：浅蓝色眼镜盒")
        lower = np.array([80, 30, 40])   
        upper = np.array([130, 255, 255])
        MIN_AREA = 10000                 # 眼镜盒面积很大，设置大一点防白墙干扰
        mask = cv2.inRange(hsv, lower, upper)

    elif i == 2:
        # 第2张图：黄色液体瓶 (反光严重，放宽S和V)
        print("目标：黄色液体瓶")
        lower = np.array([15, 40, 80])   
        upper = np.array([40, 255, 255])
        MIN_AREA = 2000                  # 整个瓶子面积
        mask = cv2.inRange(hsv, lower, upper)

    elif i == 3:
        # 第3张图：红色花朵 (红色跨了0和180)
        print("目标：红色花朵")
        lower1 = np.array([0, 80, 50])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 80, 50])
        upper2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2) 
        MIN_AREA = 1000
    # =========================================================

    # 保存原始生成的掩膜
    cv_imwrite(os.path.join(output_dir, f"{i}_raw_mask.jpg"), mask)

    # 【关键修改】：加大形态学处理的核大小，把反光断开的地方连起来
    kernel = np.ones((15, 15), np.uint8) # 从 5x5 加大到 15x15
    # 先用闭运算填补物体内部反光造成的黑色空洞
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # 再用开运算去掉周围的散碎噪点
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    
    cv_imwrite(os.path.join(output_dir, f"mask_{i}.png"), mask_clean)

    # 查找轮廓
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result_img = img.copy()
    found = False

    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        if area < MIN_AREA:
            continue
            
        found = True
        x, y, w, h = cv2.boundingRect(cnt)
        
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = x + w // 2, y + h // 2

        # 画框
        cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.circle(result_img, (cx, cy), 8, (0, 0, 255), -1)
        text = f"area:{int(area)}"
        cv2.putText(result_img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        print(f"✅ 找到目标！中心坐标: ({cx}, {cy}), 面积: {int(area)}")

    if not found:
        print("❌ 没找到符合大小的目标。")

    cv_imwrite(os.path.join(output_dir, f"result_{i}.png"), result_img)

print("\n任务三运行结束！快去 outputs/task3 看看吧！")