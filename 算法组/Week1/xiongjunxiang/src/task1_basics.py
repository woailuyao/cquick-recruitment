from pathlib import Path
import cv2 as cv
import numpy as np

image_path = Path(__file__).resolve().parent.parent / "data/images"
output_path = Path(__file__).resolve().parent.parent / "outputs" / "task1"

for p in image_path.glob("*.jpg"):
    # print(p)
    name = p.stem
    tmp = np.fromfile(str(p), dtype=np.uint8)
    img_bgr = cv.imdecode(tmp, 1)
    if img_bgr is None:
        print("读取失败")
        continue
    h, w, c = img_bgr.shape
    print(f"{name}: 高 = {h}, 宽 = {w}, 通道数 = {c}")
    # 缩放
    img_bgr_2x = cv.resize(img_bgr, None, fx=2, fy=2, interpolation=cv.INTER_NEAREST)
    ok, tmp = cv.imencode(f"{name}_2x.jpg", img_bgr_2x)
    if ok:
        tmp.tofile(str(output_path / f"{name}_2x.jpg"))
    # 裁剪
    img_cut = img_bgr[100:400,400:700]
    ok, tmp = cv.imencode(f"{name}_cut.jpg", img_cut)
    if ok:
        tmp.tofile(str(output_path / f"{name}_cut.jpg"))
    # 灰度
    img_grey = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)
    ok, tmp = cv.imencode(f"{name}_grey.jpg", img_grey)
    if ok:
        tmp.tofile(str(output_path / f"{name}_grey.jpg"))