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

cat_gray = cv.cvtColor(cat,cv.COLOR_BGR2GRAY)
cv.imshow("cat_gray",cat_gray)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_gray.jpg"),cat_gray)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_gray.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test

cat_gaussian = cv.GaussianBlur(cat_gray,(5,5),0)
cv.imshow("cat_gaussian",cat_gaussian)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_gaussian.jpg"),cat_gaussian)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_gaussian.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
ret,cat_bin = cv.threshold(cat_gaussian,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
print("OTSU阈值:",ret)
cv.imshow("cat_bin",cat_bin)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_bin.jpg"),cat_bin)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_bin.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
cat_canny = cv.Canny(cat_gaussian,50,150)
cv.imshow("cat_canny",cat_canny)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_canny.jpg"),cat_canny)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_canny.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
kernel = np.ones((5,5),np.uint8)
cat_open = cv.morphologyEx(cat_bin,cv.MORPH_OPEN,kernel)
cv.imshow("cat_open",cat_open)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_open.jpg"),cat_open)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_open.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test
cat_close = cv.morphologyEx(cat_bin,cv.MORPH_CLOSE,kernel)
cv.imshow("cat_close",cat_close)
cv.waitKey(0)
imwrite(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_close.jpg"),cat_close)
test = cv.imdecode(np.fromfile(str(Path(__file__).parent.parent/"outputs"/"task2"/"cat_close.jpg"),np.uint8),cv.IMREAD_COLOR)
if test.size != 0:
    print("修改图可读取")
else:
    sys.exit("修改图读取异常")
del test