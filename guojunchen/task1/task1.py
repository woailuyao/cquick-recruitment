import cv2
import numpy as np
image = cv2.imread("ceshi3.png")
if image is None:
    print("找不到图片")
    exit()
hsv_image =cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_green=np.array([35,43,46])
upper_green=np.array([77,255,255])
mask=cv2.inRange(hsv_image,lower_green,upper_green)
kernel = np.ones((5, 5), np.uint8)
mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
result = image.copy()
min_area = 500
valid=[]
#带着序号找
for i,cnt in enumerate(contours):
    area = cv2.contourArea(cnt)
    if area > min_area:
        valid.append(cnt)

if len(valid) ==0:
    print("未检测到目标")
    exit()

#面积从大到小排序
valid = sorted(valid, key=cv2.contourArea, reverse=True)

result = image.copy()
for i, cnt in enumerate(valid):
    # 计算面积
    current_area = cv2.contourArea(cnt)
    #算中心点
    M=cv2.moments(cnt)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        x,y,w,h = cv2.boundingRect(cnt)
        cx = x + w // 2
        cy = y + h // 2
    print(f"目标{i+1}: 面积={current_area}, 中心点=({cx},{cy})")
    cv2.drawContours(result, [cnt], -1, (0, 0, 255), 2)
    cv2.circle(result, (cx, cy), 5, (255, 0, 0), -1)  
    #排名 
    text= f"目标{i+1}: 面积={current_area}, 中心点=({cx},{cy})"
    cv2.putText(result, text, (cx-15, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
cv2.imshow("mask", mask)
cv2.imshow("result", result)
cv2.imencode(".png", result)[1].tofile("01检测输出(指定颜色).png")
cv2.imencode(".png", mask)[1].tofile("01检测输出(二值掩膜).png")
cv2.imwrite("01检测输出(指定颜色).png", result) 
cv2.imwrite("01检测输出(二值掩膜).png", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
