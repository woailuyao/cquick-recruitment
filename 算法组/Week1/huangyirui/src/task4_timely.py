import cv2
import numpy as np

SOURCES = "data/videos/test.mp4"  #读取路径


cap = cv2.VideoCapture(SOURCES)#读源
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)#宽
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)#高
if not cap.isOpened():#打不开就报错
    print("Fail to read the source:",SOURCES)
    raise SystemExit(1)

writer = cv2.VideoWriter("outputs/task4/task4_result.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 720))#输出一个视频

VideoCore = np.ones((5,5),np.uint8)#设置一个核

while True: #按帧读取数据
    NotLastOne,frame = cap.read()
    if not NotLastOne:#判断是否读取完毕
        break
    VideoHsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)#BRG 2 HSV
    VideoMask = cv2.inRange(VideoHsv,(0,0,0),(180,90,100))#找暗区
    VideoMask = cv2.morphologyEx(VideoMask,cv2.MORPH_CLOSE,VideoCore)#去核（补洞）
    Cnts,_ = cv2.findContours(VideoMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)#抓轮廓

    for c in Cnts:
        area = cv2.contourArea(c)#检查轮廓
        if area<2000:#排除噪点
            continue
        cnt1,cnt2,cnt3,cnt4 = cv2.boundingRect(c)#找外接矩阵
        if cnt4<cnt3:#跳过非竖条
            continue
        cv2.rectangle(frame,(cnt1,cnt2),(cnt1+cnt3,cnt2+cnt4),(0,255,0),2)#画矩形
        Mv = cv2.moments(c)#一阶矩求中心
        if Mv["m00"]!=0: #防止零做除数
            cx = int(Mv["m10"]/Mv["m00"])
            cy = int(Mv["m01"]/Mv["m00"])
            cv2.circle(frame,(cx,cy),4,(0,0,255),-1)#画红色中心点
            cv2.putText(frame,str(int(area)),(cnt1,cnt2-6),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)#在框上方标面积

    writer.write(frame) #写进视频里
    cv2.imshow("Demo",frame)#显示画面
    cv2.imshow("Mask_Demo",VideoMask)#显示掩膜
    key = cv2.waitKey(1) & 0xFF #每毫秒返回一次，保证流畅
    if key == ord("q") or key == 27:
        break

#cap.release()#这个只在用摄像头时调用
cv2.destroyAllWindows()#关闭窗口
writer.release() #保证末尾不掉帧