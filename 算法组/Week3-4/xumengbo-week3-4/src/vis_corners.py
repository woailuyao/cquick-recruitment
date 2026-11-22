"""把成功标定图的内角点叠加画出来，人工核对角点位置。"""
import os, cv2, numpy as np

CALIB_DIR = r"D:\W34_dataset\calib"
OUT = r"D:\W34_dataset\check\corners_check"
os.makedirs(OUT, exist_ok=True)
PATTERN = (7, 5)

for fn in ["calib_01.jpg", "calib_12.jpg", "calib_16.jpg"]:
    p = os.path.join(CALIB_DIR, fn)
    img = cv2.imread(p)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, PATTERN,
                                           cv2.CALIB_CB_ADAPTIVE_THRESH +
                                           cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        cv2.drawChessboardCorners(img, PATTERN, corners, ok)
    cv2.imwrite(os.path.join(OUT, fn), img)
    print(fn, "ok=", ok, "n=", 0 if corners is None else len(corners))
