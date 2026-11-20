import cv2 as cv
from pathlib import Path
import numpy as np
import sys
def imwrite(path, img):
    path = Path(path)
    ok, buf = cv.imencode(path.suffix, img)
    if not ok:
        raise ValueError(f"编码失败：{path}")
    buf.tofile(str(path))
    print("修改图已保存到:", path)

video = Path(__file__).parent.parent/"data"/"video"/"blue_bird.mp4"
if video.exists():
    cap = cv.VideoCapture(str(video))
    print("读取到视频",video.name)
else:
    cap = cv.VideoCapture(0)
    print("读取到摄像头")
if not cap.isOpened():
    sys.exit("视频或摄像头打开失败")
fps = cap.get(cv.CAP_PROP_FPS)
if fps <= 0:
    fps = 24
delay = max(1,int(1000/fps))
frame_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
frame_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
print("fps:",fps,"宽:",frame_w,"高:",frame_h,"每帧等待:",delay,"毫秒")

video_out = Path(__file__).parent.parent/"outputs"/"task4"/"blue_bird_result.mp4"
fourcc = cv.VideoWriter_fourcc(*"mp4v")
writer = cv.VideoWriter(str(video_out),fourcc,fps,(frame_w,frame_h))
if not writer.isOpened():
    sys.exit("结果视频无法创建")
print("结果视频输出到:",video_out)

kernel = np.ones((21,21),np.uint8)
frame_id = 0
target_id = 0
while True:
    ret,frame = cap.read()
    if not ret:
        print("播放结束，共处理",frame_id,"帧，有目标",target_id,"帧")
        break
    frame_id = frame_id + 1

    frame_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    frame_mask = cv.inRange(frame_hsv,(95,60,70),(125,255,255))
    frame_mask = cv.morphologyEx(frame_mask,cv.MORPH_CLOSE,kernel)
    frame_cnts,hierarchy = cv.findContours(frame_mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    frame_area_min = frame.shape[0]*frame.shape[1]*0.002

    boxes = []
    for c in frame_cnts:
        area = cv.contourArea(c)
        if area < frame_area_min:
            continue
        x,y,w,h = cv.boundingRect(c)
        boxes.append((x,y,w,h,area))

    if len(boxes) > 0:
        target_id = target_id + 1
        x1 = min(b[0] for b in boxes)
        y1 = min(b[1] for b in boxes)
        x2 = max(b[0]+b[2] for b in boxes)
        y2 = max(b[1]+b[3] for b in boxes)
        area = sum(b[4] for b in boxes)
        cx = (x1+x2)//2
        cy = (y1+y2)//2
        cv.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
        cv.circle(frame,(cx,cy),6,(0,0,255),-1)
        cv.putText(frame,"(%d,%d) area=%d" % (cx,cy,int(area)),(x1,max(y1-10,20)),cv.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
        if frame_id % 30 == 1:
            print("第%d帧 中心=(%d, %d) 面积=%d 轮廓数=%d" % (frame_id,cx,cy,int(area),len(boxes)))
        if frame_id == 1:
            imwrite(str(Path(__file__).parent.parent/"outputs"/"task4"/"frame_001_result.png"),frame)
    elif frame_id % 30 == 1:
        print("第%d帧 未找到目标" % frame_id)

    writer.write(frame)
    cv.imshow("blue_bird",frame)
    key = cv.waitKey(delay) & 0xFF
    if key == 27 or key == ord('q'):
        print("手动退出")
        break

cap.release()
writer.release()
cv.destroyAllWindows()
print("完整视频已保存到:",video_out, video_out.stat().st_size, "字节")