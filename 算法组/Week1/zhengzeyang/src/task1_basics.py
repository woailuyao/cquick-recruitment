import cv2
import numpy as np
from pathlib import Path

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
(root / "outputs" / "task1").mkdir(parents=True, exist_ok=True)
img1_path = root / "data" / "images" / "img1_mug_teal.jpg"
img2_path = root / "data" / "images" / "img2_apple_green.jpg"
#读取图片
img1=imread_cn(img1_path)
if img1 is None:
    raise FileNotFoundError("File1 not found")
else:
    print("img1 loaded")
img2=imread_cn(img2_path)
if img2 is None:
    raise FileNotFoundError("File2 not found")
else:
    print("img2 loaded")
    
#查看图像属性
h1,w1,num_channel1=img1.shape
h2,w2,num_channel2=img2.shape
print("h1:",h1,"w1:",w1,"num_channel1:",num_channel1)
print("h2:",h2,"w2:",w2,"num_channel2:",num_channel2)

#缩放图片
scale=0.5
new_h1,new_w1=int(h1*scale),int(w1*scale)
new_h2,new_w2=int(h2*scale),int(w2*scale)
resized_img1=cv2.resize(img1,(new_w1,new_h1),interpolation=cv2.INTER_AREA)
resized_img2=cv2.resize(img2,(new_w2,new_h2),interpolation=cv2.INTER_AREA)
imwrite_cn(root/"outputs"/"task1"/"img1_resized.png",resized_img1)
imwrite_cn(root/"outputs"/"task1"/"img2_resized.png",resized_img2)
resized_img1=imread_cn(str(root/"outputs"/"task1"/"img1_resized.png"))
if resized_img1 is None:
    raise FileNotFoundError("File1 not found")
else:
    print("resized_img1 loaded")
resized_img2=imread_cn(str(root/"outputs"/"task1"/"img2_resized.png"))
if resized_img2 is None:
    raise FileNotFoundError("File2 not found")
else:
    print("resized_img2 loaded")

#裁剪图片
crop1=img1[0:h1//2,0:w1//2]
crop2=img2[0:h2//2,0:w2//2]
imwrite_cn(root/"outputs"/"task1"/"img1_crop.png",crop1)
imwrite_cn(root/"outputs"/"task1"/"img2_crop.png",crop2)
crop1=imread_cn(str(root/"outputs"/"task1"/"img1_crop.png"))
if crop1 is None:
    raise FileNotFoundError("File1 not found")
else:
    print("crop1 loaded")
crop2=imread_cn(str(root/"outputs"/"task1"/"img2_crop.png"))
if crop2 is None:
    raise FileNotFoundError("File2 not found")
else:
    print("crop2 loaded")

#生成灰度图并保存
gray1=cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray2=cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
imwrite_cn(root/"outputs"/"task1"/"img1_gray.png",gray1)
imwrite_cn(root/"outputs"/"task1"/"img2_gray.png",gray2)
gray1=imread_cn(str(root/"outputs"/"task1"/"img1_gray.png"))
if gray1 is None:
    raise FileNotFoundError("File1 not found")
else:
    print("gray1 loaded")
gray2=imread_cn(str(root/"outputs"/"task1"/"img2_gray.png"))
if gray2 is None:
    raise FileNotFoundError("File2 not found")
else:
    print("gray2 loaded")
