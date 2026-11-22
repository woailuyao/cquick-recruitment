from pathlib import Path
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

image_path = Path(__file__).resolve().parent.parent / "data/images"
output_path = Path(__file__).resolve().parent.parent / "outputs" / "task2"

def write(img,imgname):
    ok, tmp = cv.imencode(f"{name}_{imgname}.jpg", img)
    if ok:
        tmp.tofile(str(output_path / f"{name}_{imgname}.jpg"))

for p in image_path.glob("venti.jpg"):
    # print(p)
    name = p.stem
    tmp = np.fromfile(str(p), dtype=np.uint8)
    img_bgr = cv.imdecode(tmp, 1)
    if img_bgr is None:
        print("读取失败")
        continue
    img_rgb = cv.cvtColor(img_bgr, cv.COLOR_BGR2RGB)
    img_gray = cv.cvtColor(img_rgb, cv.COLOR_RGB2GRAY)
    img_blur = cv.GaussianBlur(img_bgr, (13, 13), 0)
    img_blur_modified = cv.GaussianBlur(img_bgr, (5, 5), 0)
    ret, img_thresh = cv.threshold(img_gray, 150, 255, cv.THRESH_BINARY)
    ret, img_thresh_modified = cv.threshold(img_gray, 100, 255, cv.THRESH_BINARY)
    img_canny = cv.Canny(img_gray, 145, 150, cv.THRESH_BINARY)
    img_canny_modified = cv.Canny(img_gray, 100, 200, cv.THRESH_BINARY)
    img_erode = cv.erode(img_bgr, (5,5), iterations=3)
    write(img_gray,"gray")
    write(img_blur,"blur")
    write(img_blur_modified,"blur_modified")
    write(img_canny,"canny")
    write(img_canny_modified,"canny_modified")
    write(img_thresh,"thresh")
    write(img_thresh_modified,"thresh_modified")
    write(img_erode,"erode")

