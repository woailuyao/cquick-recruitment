import os
import cv2 as cv
from detect import imread_zh, imwrite_zh, detect, draw_result, IMG_DIR, OUT_DIR

for s in [80, 120, 150, 170]:
    for name in ["test5", "test6"]:
        img = imread_zh(os.path.join(IMG_DIR, name + ".png"))
        d, u, v, area, c, mask = detect(img, lo=(8, s, 80), hi=(25, 255, 255))
        draw_result(img, d, u, v, c)
        cv.putText(img, "S>=%d" % s, (20, 470), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        imwrite_zh(os.path.join(OUT_DIR, "cmp_S%03d_%s.png" % (s, name)), img)
        print("S>=%d %-6s detected=%-5s area=%7.0f" % (s, name, d, area))

for k in [3, 5, 7, 9]:
    img = imread_zh(os.path.join(IMG_DIR, "test5.png"))
    d, u, v, area, c, mask = detect(img, ksize=k)
    imwrite_zh(os.path.join(OUT_DIR, "cmp_k%d_mask.png" % k), mask)
    print("kernel=%d detected=%-5s area=%7.0f" % (k, d, area))
