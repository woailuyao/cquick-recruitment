import sys

import cv2
import numpy as np

LO = np.array([5, 90, 70], np.uint8)
HI = np.array([25, 255, 255], np.uint8)

for p in sys.argv[1:]:
    img = cv2.imread(p)
    if img is None:
        print(p, 'READ_FAIL')
        continue
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, LO, HI)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cs = sorted(cs, key=cv2.contourArea, reverse=True)[:6]
    print('== %s  %dx%d  mask_px=%d' % (p.split('/')[-1], w, h, int(m.sum() / 255)))
    for c in cs:
        a = cv2.contourArea(c)
        x, y, bw, bh = cv2.boundingRect(c)
        print('   area=%7.0f  box=(%4d,%4d,%3d,%3d)  ratio=%.2f  fill=%.2f'
              % (a, x, y, bw, bh, bw / max(bh, 1), a / max(bw * bh, 1)))
