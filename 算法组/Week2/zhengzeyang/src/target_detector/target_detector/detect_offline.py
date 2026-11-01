# detect_offline.py —— 用普通 python3 跑，不进 ROS2，调阈值最快
import os
import sys

import cv2

from detector_core import detect_target, MIN_AREA


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'test.jpg'
    min_area = int(sys.argv[2]) if len(sys.argv) > 2 else MIN_AREA

    img = cv2.imread(path)
    if img is None:
        print('读图失败:', path)
        return

    r = detect_target(img, min_area=min_area)

    if r['found']:
        print('%s -> 检测到目标  u=%.1f  v=%.1f  area=%.0f'
              % (os.path.basename(path), r['u'], r['v'], r['area']))
        cv2.drawContours(img, [r['contour']], -1, (0, 255, 0), 2)
        cx, cy = int(r['u']), int(r['v'])
        cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)
        cv2.putText(img, 'u=%d v=%d' % (cx, cy), (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    else:
        print('%s -> 未检测到目标' % os.path.basename(path))
        cv2.putText(img, 'NO TARGET', (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

    out_dir = os.path.expanduser('~/vision_ws/week2/out')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, 'out_' + os.path.basename(path))
    cv2.imwrite(out, img)
    print('已保存:', out)


main()
