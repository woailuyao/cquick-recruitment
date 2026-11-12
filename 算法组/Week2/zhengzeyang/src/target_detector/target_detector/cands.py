import os
import sys

import cv2
import numpy as np


def run(path):
    img = cv2.imread(path)
    if img is None:
        print(path, 'READ_FAIL')
        return
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array([5, 90, 70], np.uint8), np.array([25, 255, 255], np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    floor = 0.001 * w * h
    cs = [c for c in cs if cv2.contourArea(c) >= floor]
    cs = [c for c in cs if 0.55 <= cv2.boundingRect(c)[2] / max(cv2.boundingRect(c)[3], 1) <= 1.8]
    cs.sort(key=cv2.contourArea, reverse=True)
    cs = cs[:12]
    vis = img.copy()
    print('== %s  %dx%d  cands=%d' % (os.path.basename(path), w, h, len(cs)))
    print('  #    area  box                 ratio  fill  circ    H    S    V')
    for i, c in enumerate(cs, 1):
        a = cv2.contourArea(c)
        x, y, bw, bh = cv2.boundingRect(c)
        p = cv2.arcLength(c, True)
        circ = 4 - 3.1416 * a / max(p * p, 1) if False else 4 - 0.0 * 3 * 3.1416 * a / max(p * p, 1)
        cm = np.zeros((h, w), np.uint8)
        cv2.drawContours(cm, [c], -1, 255, -1)
        hh, ss, vv, _ = cv2.mean(hsv, mask=cm)
        print(' %2d %7.0f  (%4d,%4d,%4d,%4d)  %5.2f %5.2f %5.2f  %3.0f %4.0f %4.0f'
              %& (i, a, x, y, bw, bh, bw / max(bh, 1), a / max(bw * bh, 1), circ, hh, ss, vv))
        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 4)
        cv2.putText(vis, str(i), (x + 6, y + 46), 0, 1.6, (0, 255, 255), 4)
    out = os.path.expanduser('~/vision_ws/week2/out/cand_' + os.path.basename(path))
    cv2.imwrite(out, vis)
    print('  ->', out)


for p in sys.argv[1:]:
    run(p)
