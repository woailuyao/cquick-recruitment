import cv2
import os
os.makedirs("outputs",exist_ok=True)
img=cv2.imread("photo3.jpg")
if img is not None:
    height,width,channels=img.shape
    print(f"图像的高为:{height}")
    print(f"图像的宽为:{width}")
    print(f"图像的通道数为:{channels}")
    scale1=float(input("请输入宽的缩放倍数:"))
    scale2=float(input("请输入高的缩放倍数:"))
    resized_img=cv2.resize(img,dsize=(0,0),fx=scale1,fy=scale2)
    cv2.imshow("resized",resized_img)
    cv2.imwrite("outputs_resized2.jpg",resized_img)
    cv2.waitKey(0)
    print("请输入裁切坐标")
    y1=int(input("y1:"))
    y2=int(input("y2:"))
    x1=int(input("x1:"))
    x2=int(input("x2:"))
    new_img=img[y1:y2,x1:x2]
    cv2.imshow("new",new_img)
    cv2.imwrite("outputs_new2_img.jpg",new_img)
    cv2.waitKey(0)
    gray_img=cv2.cvtColor(new_img,cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray",gray_img)
    cv2.imwrite("outputs_gray2.jpg",gray_img)
    cv2.waitKey(0)
else:
    print("输入失败")
    
