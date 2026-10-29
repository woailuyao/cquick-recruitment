import cv2 as cv
from pathlib import Path
import numpy as np
import sys
def imwrite(path, img):
    path = Path(path)
    ok, buf = cv.imencode(".jpg", img)
    if not ok:
        raise ValueError(f"编码失败：{path}")
    buf.tofile(str(path))
    print("修改图已保存到:", path)

cat = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"data"/"images"/"cat.jpg"),np.uint8),cv.IMREAD_COLOR)
if cat.size == 0:
    sys.exit("cat not found")
print("读取到cat",cat.shape)
cat_small = cv.resize(cat,None,fx=0.2,fy=0.2)
cv.imshow("cat_small",cat_small)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task1"/"cat_small.jpg"),cat_small)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task1"/"cat_small.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test

cat_cut = cat[200:900,400:1800]
cv.imshow("cat_cut",cat_cut)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task1"/"cat_cut.jpg"),cat_cut)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task1"/"cat_cut.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test

cat_gray = cv.cvtColor(cat,cv.COLOR_BGR2GRAY)
cv.imshow("cat_gray",cat_gray)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task1"/"cat_gray.jpg"),cat_gray)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task1"/"cat_gray.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test


dog = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"data"/"images"/"dog.jpg"),np.uint8),cv.IMREAD_COLOR)
if dog.size == 0:
    sys.exit("dog not found")
print("读取到dog",dog.shape)
dog_small = cv.resize(dog,None,fx=0.2,fy=0.2)
cv.imshow("dog_small",dog_small)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task1"/"dog_small.jpg"),dog_small)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task1"/"dog_small.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test

dog_cut = dog[200:900,400:1800]
cv.imshow("dog_cut",dog_cut)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task1"/"dog_cut.jpg"),dog_cut)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task1"/"dog_cut.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test

dog_gray = cv.cvtColor(dog,cv.COLOR_BGR2GRAY)
cv.imshow("dog_gray",dog_gray)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task1"/"dog_gray.jpg"),dog_gray)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task1"/"dog_gray.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test