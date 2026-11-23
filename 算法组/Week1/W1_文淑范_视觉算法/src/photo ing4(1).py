import cv2
import os
import numpy as np
img=cv2.imread("photo2.jpg")
os.makedirs("outputs",exist_ok=True)
if img is None:
    print("输入失败")
else:
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",hsv)
    cv2.imwrite("outputs_hsv.jpg",hsv)
    cv2.waitKey(0)
    l=np.array([10,100,100])
    h=np.array([25,255,255])
    mask=cv2.inRange(hsv,l,h)
    cv2.imshow("mask",mask)
    cv2.imwrite("outputs_mask.jpg",mask)
    cv2.waitKey(0)
    kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    mask_clean=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    cv2.imshow("mask_clean",mask_clean)
    cv2.imwrite("output_mask_clean.jpg",mask_clean)
    cv2.waitKey(0)
    contours,_=cv2.findContours(mask_clean,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    valid_contours=[]
    m_area=50
    for cnt in contours:
        area=cv2.contourArea(cnt)
        if area>m_area:
                 valid_contours.append(cnt)
    l_img=img.copy()
    for cnt in valid_contours:
        x,y,m,n=cv2.boundingRect(cnt)
        cx=int(x+m/2)
        cy=int(y+n/2)
        print(f"中心坐标({cx,cy}),轮廓面积：{area:.1f}")
        cv2.rectangle(l_img,(x,y),(m+x,n+y),(0,255,1),2)
        cv2.circle(l_img,(cx,cy),4,(0,0,255),-1)
    cv2.imshow("l_img",l_img)
    cv2.imwrite("outputs_l_img.jpg",l_img)
    cv2.waitKey(0)
   
    
                            
    
    
                

