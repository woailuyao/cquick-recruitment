import cv2
import os
import numpy as np

img_dict = {"lid.jpg":"red","paper.jpg":"yellow","pin.jpg":"blue"}
for name,color in img_dict.items():
    image_path = os.path.expanduser(f"~/ros2_ws/images/{name}")
    img = cv2.imread(image_path)

    if img is None:
        print(f"error：{image_path}")
    else:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if color == "red":
            lower1, upper1 = np.array([0, 150, 150]), np.array([10, 255, 255])
            lower2, upper2 = np.array([170, 150, 150]), np.array([180, 255, 255])
            mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)
        elif color == "blue":
            mask = cv2.inRange(hsv, np.array([100, 100, 100]), np.array([130, 255, 255]))
        elif color == "yellow":
            mask = cv2.inRange(hsv, np.array([20, 150, 100]), np.array([35, 255, 255]))

        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        b = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            a = w / h if h > 0 else 0
            if 0.5 < a < 2:
                b.append(cnt)
        if len(b) ==0:
            print('未检测到目标')
        else:
            c = max(b, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            area_c = cv2.contourArea(c)
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            M = cv2.moments(c)
            if M['m00'] != 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
                text = f"Area:{int(area_c)} C:(u:{u},v:{v})"
                cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        result_name = f"test_result_area_{name}"
        cv2.imwrite(result_name, img)






