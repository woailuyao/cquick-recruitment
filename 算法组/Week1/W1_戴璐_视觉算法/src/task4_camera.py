import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头！")
    exit()

print("摄像头已开启！按按键'q' 退出程序")

while True:
    ret, frame = cap.read()

    if not ret:
        print("画面读取失败！")
        break

    #1.将BGR转为HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    #2.设定橙色范围，生成掩膜
    lower_orange = np.array([5,100,100])
    upper_orange = np.array([25,255,255])

    mask = cv.inRange(hsv, lower_orange, upper_orange)

    #3.形态学操作
    kernel = np.ones((2, 2), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    #4.寻找轮廓
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    #画框逻辑
    if len(contours) > 0:
        largest_contour = max(contours, key=cv.contourArea)
        area = cv.contourArea(largest_contour)

        if area > 1000:
            # 获取最大轮廓的外接矩形
            x, y, w, h = cv.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2
            
            #在原视频上画框和点
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            text = f"Area:{int(area)}"
            cv.putText(frame, text, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv.imshow("Real-time Detection", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destoryAllWindows()


        
            


