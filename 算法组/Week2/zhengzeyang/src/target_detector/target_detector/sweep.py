import os
import sys

import cv2
import numpy as np

SETS = [
    ('A base', (5, 90, 70), (25, 255, 255)),
    ('B s150', (5, 150, 100), (25, 255, 255)),
    ('C s190', (5, 190, 130), (25, 255, 255)),
    ('D h0', (0, 150, 100), (12, 255, 255)),
    ('E h0s190', (0, 190, 130), (14, 255, 255)),
]


def best(img, lo, hi):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    floor = 0.002 * img.shape[0] * img.shape[1]
    pick = None
    for c in cs:
        a = cv2.contourArea(c)
        if a < floor:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if not (0.7 <= w / max(h, 1) <= 1.4):
            continue
        fill = a / max(w * h, 1)
        if pick is None or fill > pick[0]:
            pick = (fill, a, x, y, w, h)
    return pick, int(m.sum() / 255)


for p in sys.argv[1:]:
    img = cv2.imread(p)
    if img is None:
        print(p, 'READ_FAIL')
        continue
    h, w = img.shape[:2]
    print('== %s  %dx%d' % (os.path.basename(p), w, h))
    for name, lo, hi in SETS:
        pick, maskpx = best(img, lo, hi)
        if pick is None:
            print('   %-8s mask=%6d  none' % (name, maskpx))
        else:
            fill, a, x, y, bw, bh = pick
            print('   %-8s mask=%6d  fill=%.2f area=%7.0f box=(%d,%d,%d,%d) c=(%d,%d)'
                  % (name, maskpx, fill, a, x, y, bw, bh, x + bw // 2, y + bh // 2))
