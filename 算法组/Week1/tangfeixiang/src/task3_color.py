from email.mime import image

import cv2
import numpy as np
import os

current_script_path=os.path.abspath(__file__)
src_dir=os.path.dirname(current_script_path)
project_root=os.path.dirname(src_dir)
image_path=os.path.join(project_root, "../data", "images", "test3_orange_warm.jpg")
output_dir = os.path.join(project_root, "../outputs", "task3_orange_warm.jpg")



image_path= "../data/images/test3_orange_warm.jpg"
image=cv2.imread(image_path)

if image is None:
    print("读取失败")
else:
    print("读取成功")

    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    lower_green=np.array([0,50,50])
    upper_green=np.array([15,255,255])

    mask=cv2.inRange(hsv,lower_green,upper_green)

    ##kernel=np.ones((5,5),np.uint8)
    ##mask_cleaned=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    cv2.imwrite("../outputs/task3/mask3_warm_fixed.png", mask)

    contours,_=cv2.findContours(mask ,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if contours:
            largest_contour=max(contours,key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            M = cv2.moments(largest_contour)

    if M["m00"]!=0:
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cx=int(M["m10"]/M["m00"])
        cy=int(M["m01"]/M["m00"])
        cv2.circle(image,(cx,cy),5,(0,0,255),-1)
        text = f"u={cx}, v={cy}"
        cv2.putText(image, text, (cx - 50, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        print(f"中心点坐标({cx},{cy}),面积{area}")
        cv2.imwrite("../outputs/task3/result3_warm_fixed.png", image)
        print("任务完成")
    else:
        cx,cy=0,0
        print("未检测到目标")

