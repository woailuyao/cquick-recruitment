import numpy as np
import cv2 as cv

def detect_and_draw(img_name, lower_color, upper_color, min_area, out_prefix):
    img = cv.imread(f"data/images/{img_name}") 
    if img is None:
        print(f"{img_name} 读取失败")
        return
    
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_color, upper_color)
    
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (11, 11))
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    result = img.copy()
    
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > min_area: 
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(result, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cx, cy = x + w // 2, y + h // 2
            cv.circle(result, (cx, cy), 5, (255, 0, 0), -1)
            print(f"[{out_prefix}] 找到目标：面积={area:.1f}, 中心点=({cx}, {cy})")

    cv.imwrite(f"outputs/task3/{out_prefix}_mask.png", mask)
    cv.imwrite(f"outputs/task3/{out_prefix}_result.png", result)

if __name__ == "__main__":
    # 测试图1：提取棕黄色
    detect_and_draw("img1.jpg", np.array([10, 50, 50]), np.array([30, 255, 255]), 5000, "img1")
    
    # 测试图2：提取黄色
    detect_and_draw("img2.jpg", np.array([20, 100, 100]), np.array([35, 255, 255]), 1000, "img2")
    
    # 测试图3：提取绿色
    detect_and_draw("img3.jpg", np.array([35, 43, 46]), np.array([85, 255, 255]), 5000, "img3")