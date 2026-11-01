import cv2
import numpy as np

image_list = ["img1.jpg", "img2.jpg", "img3.jpg", "img4.jpg", "img5.jpg"]

HSV_LOWER = np.array([35, 43, 46])    
HSV_UPPER = np.array([85, 255, 255]) 

MIN_AREA = 500
MAX_AREA = 300000  

for IMG_PATH in image_list:
    print(f"\n--- 正在处理 {IMG_PATH} ---")
    img = cv2.imread(IMG_PATH)
    if img is None:
        print(f"图片读取失败！")
        continue

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        print("未检测到目标")
        cv2.imwrite(f"result_{IMG_PATH}", img)
        cv2.putText(img, "No target detected", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    else:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        target = None
        target_area = 0
        
        for c in contours:
            area = cv2.contourArea(c)
            if MIN_AREA <= area <= MAX_AREA:
                target = c
                target_area = area
                break

        if target is None:
            print("未检测到目标")
            cv2.imwrite(f"result_{IMG_PATH}", img)
        else:
            M = cv2.moments(target)
            if M["m00"] != 0:
                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])
                print(f"【检测成功】目标中心点: u={u}, v={v}, 面积={target_area}")
                
                cv2.drawContours(img, [target], -1, (0, 255, 0), 3) 
                cv2.circle(img, (u, v), 8, (0, 0, 255), -1)
                cv2.putText(img, f"u={u}, v={v}", (u + 15, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                cv2.imwrite(f"result_{IMG_PATH}", img)
                print(f"已保存标注图片: result_{IMG_PATH}")