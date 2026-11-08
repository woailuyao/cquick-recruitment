import cv2
import os

out_dir = "output_task3"
if not os.path.exists(out_dir):
    os.mkdir(out_dir)

img_list = [
    {"name":"apple.jpg", "low_hsv":(0,120,80), "high_hsv":(10,255,255)},
    {"name":"basketball.jpg", "low_hsv":(0,100,100), "high_hsv":(15,255,255)},
    {"name":"football.jpg", "low_hsv":(0,0,0), "high_hsv":(180,90,90)}
]

for item in img_list:
    filename = item["name"]
    low = item["low_hsv"]
    high = item["high_hsv"]
    img = cv2.imread(filename)
    h,w = img.shape[:2]
    base = filename.split(".")[0]

    #转到HSV空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    cv2.imwrite(os.path.join(out_dir, f"{base}_mask.jpg"), mask)

    #形态学去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    clean_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_OPEN, kernel)
    cv2.imwrite(os.path.join(out_dir, f"{base}_clean_mask.jpg"), clean_mask)

    #轮廓检测，绘制包围框与中心点
    contours,_ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    res_img = img.copy()
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 300:
            x,y,ww,hh = cv2.boundingRect(cnt)
            cv2.rectangle(res_img, (x,y), (x+ww,y+hh), (0,255,0),2)
            cx = x + ww//2
            cy = y + hh//2
            cv2.circle(res_img, (cx,cy),4,(0,0,255),-1)
            print(f"{filename} 目标中心坐标 u={cx}, v={cy}")

    cv2.imwrite(os.path.join(out_dir, f"{base}_result.jpg"), res_img)

print("task3全部处理完成！结果保存在 output_task3")
`
