import cv2
from pathlib import Path
import numpy as np

#中文路径读取
def imread_cn(path, flags=cv2.IMREAD_COLOR):
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, flags)
def imwrite_cn(path, img):
    path = str(path)
    ext = "." + path.rsplit(".", 1)[-1]
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        return False
    buf.tofile(path)
    return True


#定义相对路径
root_path = Path(__file__).resolve().parent.parent
(root_path / "outputs" / "task3").mkdir(parents=True, exist_ok=True)
img1_path = root_path / 'data' /'images'/ 'img1_mug_teal.jpg'
img2_path = root_path / 'data' / 'images'/'img2_apple_green.jpg'
img3_path = root_path / 'data' / 'images'/'img3_apple_red.jpg'
#读取图片
img1=imread_cn(str(img1_path))
if img1 is None:
    raise FileNotFoundError("File not found")
img2=imread_cn(str(img2_path))
if img2 is None:
    raise FileNotFoundError("File not found")
img3=imread_cn(str(img3_path))
if img3 is None:
    raise FileNotFoundError("File not found")
#处理图片1
hsv1=cv2.cvtColor(img1,cv2.COLOR_BGR2HSV)
lower1,upper1=(78,100,40),(108,255,255)
mask1=cv2.inRange(hsv1,lower1,upper1)
mask1= cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
imwrite_cn(str(root_path / 'outputs' / 'task3'/'mask1.png'), mask1)
cnt1,_=cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
H1, W1 = img1.shape[:2]
min_area = int(H1* W1* 0.001) 
for cnt in cnt1:
    area = cv2.contourArea(cnt)
    if area < min_area:
        continue                          
    x1, y1, w1, h1 = cv2.boundingRect(cnt)
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        continue
    cx1, cy1 = M["m10"] / M["m00"], M["m01"] / M["m00"]
    print(f"[img1] 面积={area:8.0f}px  中心=({cx1:6.1f}, {cy1:6.1f}) ")
vis1=img1.copy()
cv2.rectangle(vis1, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 0), 2)
cv2.circle(vis1, (int(cx1), int(cy1)), 6, (0, 0, 255), -1)
cv2.putText(vis1, f"{area:.0f}px {(int(cx1),int(cy1))}", (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
imwrite_cn(str(root_path / 'outputs' / 'task3'/'result1.png'), vis1)
#处理图片2
hsv2=cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
lower2,upper2=(25,30,8),(100,255,255)
mask2=cv2.inRange(hsv2,lower2,upper2)
mask2= cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
mask2= cv2.morphologyEx(mask2, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
imwrite_cn(str(root_path / 'outputs' / 'task3'/'mask2.png'), mask2)
cnt2,_=cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
H2, W2 = img2.shape[:2]
min_area = int(H2* W2* 0.001) 
for cnt in cnt2:
    area = cv2.contourArea(cnt)
    if area < min_area:
        continue                          
    x2, y2, w2, h2 = cv2.boundingRect(cnt)
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        continue
    cx2, cy2 = M["m10"] / M["m00"], M["m01"] / M["m00"]
    print(f"[img2] 面积={area:8.0f}px  中心=({cx2:6.1f}, {cy2:6.1f}) ")
vis2=img2.copy()
cv2.rectangle(vis2, (x2, y2), (x2 + w2, y2 + h2), (255, 0, 0), 2)
cv2.circle(vis2, (int(cx2), int(cy2)), 6, (0, 0, 255), -1)
cv2.putText(vis2, f"{area:.0f}px {(int(cx2),int(cy2))}", (x2, y2 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
imwrite_cn(str(root_path / 'outputs' / 'task3'/'result2.png'), vis2)
#处理图片3
hsv3=cv2.cvtColor(img3,cv2.COLOR_BGR2HSV)
lab = cv2.cvtColor(img3, cv2.COLOR_BGR2LAB)
a = lab[:, :, 1]
mask3 = cv2.inRange(a, 155, 255)             
mask3= cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
mask3= cv2.morphologyEx(mask3, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
imwrite_cn(str(root_path / 'outputs' / 'task3'/'mask3.png'), mask3)
cnt3,_=cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
H3, W3 = img3.shape[:2]
min_area = int(H3* W3* 0.001) 
for cnt in cnt3:
    area = cv2.contourArea(cnt)
    if area < min_area:
        continue                          
    x3, y3, w3, h3 = cv2.boundingRect(cnt)
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        continue
    cx3, cy3 = M["m10"] / M["m00"], M["m01"] / M["m00"]
    print(f"[img3] 面积={area:8.0f}px  中心=({cx3:6.1f}, {cy3:6.1f}) ")
vis3=img3.copy()
cv2.rectangle(vis3, (x3, y3), (x3 + w3, y3 + h3), (255, 0, 0), 2)
cv2.circle(vis3, (int(cx3), int(cy3)), 6, (0, 0, 255), -1)
cv2.putText(vis3, f"{area:.0f}px {(int(cx3),int(cy3))}", (x3, y3 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
imwrite_cn(str(root_path / 'outputs' / 'task3'/'result3.png'), vis3)


