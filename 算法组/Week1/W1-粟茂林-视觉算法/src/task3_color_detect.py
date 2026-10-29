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
# color_objects：抽蓝色圆柱
obj = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"data"/"images"/"color_objects.jpg"),np.uint8),cv.IMREAD_COLOR)
if obj.size == 0:
    sys.exit("color_objects not found")
print("读取到color_objects",obj.shape)
obj_hsv = cv.cvtColor(obj,cv.COLOR_BGR2HSV)
obj_mask = cv.inRange(obj_hsv,(95,120,60),(125,255,255))
kernel = np.ones((7,7),np.uint8)
obj_mask = cv.morphologyEx(obj_mask,cv.MORPH_OPEN,kernel)
cv.imshow("obj_mask",obj_mask)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task3"/"color_objects_mask.png"),obj_mask)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task3"/"color_objects_mask.png"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
obj_cnts,hierarchy = cv.findContours(obj_mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
print("轮廓总数:",len(obj_cnts))
obj_area_min = obj.shape[0]*obj.shape[1]*0.002
obj_result = obj.copy()
for c in obj_cnts:
    area = cv.contourArea(c)
    if area < obj_area_min:
        continue
    x,y,w,h = cv.boundingRect(c)
    cv.rectangle(obj_result,(x,y),(x+w,y+h),(0,255,0),3)
    cv.circle(obj_result,(x+w//2,y+h//2),8,(0,0,255),-1)
    print("轮廓面积:",int(area),"中心坐标:",(x+w//2,y+h//2))
cv.imshow("obj_result",obj_result)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task3"/"color_objects_result.png"),obj_result)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task3"/"color_objects_result.png"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
# apple：抽红苹果，红色跨色相 0 点，要分两段再取并集
apple = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"data"/"images"/"apple.jpg"),np.uint8),cv.IMREAD_COLOR)
if apple.size == 0:
    sys.exit("apple not found")
print("读取到apple",apple.shape)
apple_hsv = cv.cvtColor(apple,cv.COLOR_BGR2HSV)
apple_red1 = cv.inRange(apple_hsv,(0,100,50),(10,255,255))
apple_red2 = cv.inRange(apple_hsv,(170,100,50),(180,255,255))
apple_mask = cv.bitwise_or(apple_red1,apple_red2)
kernel = np.ones((9,9),np.uint8)
apple_mask = cv.morphologyEx(apple_mask,cv.MORPH_OPEN,kernel)
cv.imshow("apple_mask",apple_mask)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task3"/"apple_mask.png"),apple_mask)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task3"/"apple_mask.png"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
apple_cnts,hierarchy = cv.findContours(apple_mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
print("轮廓总数:",len(apple_cnts))
apple_area_min = apple.shape[0]*apple.shape[1]*0.002
apple_result = apple.copy()
for c in apple_cnts:
    area = cv.contourArea(c)
    if area < apple_area_min:
        continue
    x,y,w,h = cv.boundingRect(c)
    cv.rectangle(apple_result,(x,y),(x+w,y+h),(0,255,0),3)
    cv.circle(apple_result,(x+w//2,y+h//2),8,(0,0,255),-1)
    print("轮廓面积:",int(area),"中心坐标:",(x+w//2,y+h//2))
cv.imshow("apple_result",apple_result)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task3"/"apple_result.png"),apple_result)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task3"/"apple_result.png"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
# blue_bird：浅蓝饱和度低要放低下限，翅膀和尾羽被白肚子隔断，用大核闭运算连成一块
bird = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"data"/"images"/"blue_bird.jpg"),np.uint8),cv.IMREAD_COLOR)
if bird.size == 0:
    sys.exit("blue_bird not found")
print("读取到blue_bird",bird.shape)
bird_hsv = cv.cvtColor(bird,cv.COLOR_BGR2HSV)
bird_mask = cv.inRange(bird_hsv,(95,60,70),(125,255,255))
kernel = np.ones((21,21),np.uint8)
bird_mask = cv.morphologyEx(bird_mask,cv.MORPH_CLOSE,kernel)
cv.imshow("bird_mask",bird_mask)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task3"/"blue_bird_mask.png"),bird_mask)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task3"/"blue_bird_mask.png"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
bird_cnts,hierarchy = cv.findContours(bird_mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
print("轮廓总数:",len(bird_cnts))
bird_area_min = bird.shape[0]*bird.shape[1]*0.002
bird_result = bird.copy()
for c in bird_cnts:
    area = cv.contourArea(c)
    if area < bird_area_min:
        continue
    x,y,w,h = cv.boundingRect(c)
    cv.rectangle(bird_result,(x,y),(x+w,y+h),(0,255,0),3)
    cv.circle(bird_result,(x+w//2,y+h//2),8,(0,0,255),-1)
    print("轮廓面积:",int(area),"中心坐标:",(x+w//2,y+h//2))
cv.imshow("bird_result",bird_result)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task3"/"blue_bird_result.png"),bird_result)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task3"/"blue_bird_result.png"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
