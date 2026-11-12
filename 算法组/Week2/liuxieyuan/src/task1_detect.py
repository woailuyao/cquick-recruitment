import cv2
import numpy as np

img_paths = ['green1.jpg', 'green2.jpg', 'test3.jpg', 'misdetect_cherry.jpg', 'test1.jpg']
canvas = np.zeros((800, 1200, 3), dtype=np.uint8)

for i, path in enumerate(img_paths):
    print(f"\n--- 正在处理第 {i+1} 张图: {path} ---")
    img = cv2.imread(path)
    
    if img is None:
        print(f"❌ 读取失败: {path}")
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        cv2.putText(img, "Read Failed", (80, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        result_img = img
    else:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 43, 46])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area > 500:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3)
                    cv2.circle(img, (u, v), 5, (255, 0, 0), -1)
                    text = f"u={u}, v={v}"
                    cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    print(f"✅ 成功！中心坐标: u={u}, v={v}")
            else:
                print("⚠️ 目标太小，漏检。")
        else:
            print("❌ 未检测到绿色目标！")
            
        result_img = cv2.resize(img, (400, 400))
        
    row = i // 3
    col = i % 3
    canvas[row*400:(row+1)*400, col*400:(col+1)*400] = result_img

cv2.imwrite('week2_all_results.jpg', canvas)
print("\n🎉 全部处理完毕！已生成终极拼图 week2_all_results.jpg")
