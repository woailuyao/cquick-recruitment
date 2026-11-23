import cv2
import numpy as np
import os
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

img_list=['bottle.jpg','vase.jpg','block.jpg']#至少用 3 张不同背景或光照条件的图片进行测试

for i in img_list:
    img_path = f"data/images/{i}"
    img = cv2.imread(img_path)
    #将图像从 BGR 转换到 HSV，并通过 inRange 生成二值掩膜。
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100, 80, 40])
    upper_blue = np.array([130, 255, 255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    #使用开运算或闭运算改善掩膜
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #通过 findContours 查找目标轮廓
    contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:     #按轮廓面积过滤明显的噪声区域
            continue
        x, y, w, h = cv2.boundingRect(cnt)#在原图上绘制目标外接矩形和中心点
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
            text = f"Area:{int(area)} C:({cx},{cy})"
            cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            #输出中心坐标 (x, y) 与轮廓面积

    cv2.drawContours(img, contours, -1, (0, 255, 0), 1)


    cv2.imwrite(f"outputs/task3/{i}_mask.png", mask)
    cv2.imwrite(f"outputs/task3/{i}_result.png", img)