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
root = Path(__file__).resolve().parent.parent
(root / "outputs" / "task2").mkdir(parents=True, exist_ok=True)
img_path = root / "data" / "images" / "img2_apple_green.jpg"

#读取图片
img=imread_cn(img_path)
if img is None:
    print("Image not found")
#生成灰度图并保存
grey=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
imwrite_cn(root/"outputs"/"task2"/"img_grey.png", grey)
#生成高斯滤波结果并保存
blur=cv2.GaussianBlur(grey,(5,5),0)
imwrite_cn(root/"outputs"/"task2"/"img_blur.png", blur)
#进行阈值分割比较不同参数影响并保存结果
ret,binary1=cv2.threshold(blur,32,255,cv2.THRESH_BINARY)
ret,binary2=cv2.threshold(blur,127,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
imwrite_cn(str(root/"outputs"/"task2"/"binary1.png"), binary1)
imwrite_cn(str(root/"outputs"/"task2"/"binary2_otsu.png"), binary2)
#进行边缘检测比较参数影响并保存结果
edges1=cv2.Canny(blur,100,200)
edges2=cv2.Canny(blur,30,150)
edges3=cv2.Canny(blur,20,100)
imwrite_cn(str(root/"outputs"/"task2"/"edges2.png"), edges2)
imwrite_cn(str(root/"outputs"/"task2"/"edges1_100_200.png"), edges1)
imwrite_cn(str(root/"outputs"/"task2"/"edges3_20_100.png"), edges3)
#对阈值分割结果进行形态学操作并保存结果
opened_binary1=cv2.morphologyEx(binary1, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
closed_binary1=cv2.morphologyEx(opened_binary1, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
imwrite_cn(str(root/"outputs"/"task2"/"opened_binary1.png"), closed_binary1)
closed_only_binary=cv2.morphologyEx(binary1, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
imwrite_cn(str(root/"outputs"/"task2"/"closed_only_binary.png"), closed_only_binary)