"""生成六张程序合成的预演图；它们不是实拍照片，也不是个人实操截图。

在个人项目目录运行：python3 src/prepare_demo_images.py
正式练习的照片另放 data/real_images，不会被这个脚本改动。
"""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def blue(value=210):
    """从固定 HSV 转为 OpenCV 使用的 BGR，便于单独改变亮度 V。"""
    pixel = np.array([[[110, 200, value]]], dtype=np.uint8)
    return tuple(int(x) for x in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


def canvas(label):
    image = np.full((480, 640, 3), 230, dtype=np.uint8)
    cv2.putText(image, 'SYNTHETIC / ' + label, (14, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (35, 35, 35), 1, cv2.LINE_AA)
    return image


def main():
    dest = ROOT / 'data/images'
    dest.mkdir(parents=True, exist_ok=True)
    scenes = []

    image = canvas('clear')
    cv2.circle(image, (220, 240), 60, blue(), -1)
    scenes.append(('01_clear.png', image, [220, 240], '清晰蓝色圆，半径60像素。'))

    image = canvas('small')
    cv2.circle(image, (330, 250), 9, blue(), -1)
    scenes.append(('02_small.png', image, [330, 250], '小蓝圆；默认面积门槛500会漏掉，改为100后应检出。'))

    image = canvas('clutter')
    cv2.circle(image, (420, 230), 42, blue(), -1)
    cv2.rectangle(image, (50, 120), (170, 260), (20, 140, 230), -1)
    cv2.rectangle(image, (200, 310), (340, 400), (50, 180, 40), -1)
    for x, y in ((70, 350), (360, 150), (530, 350), (200, 100)):
        cv2.circle(image, (x, y), 1, blue(), -1)
    scenes.append(('03_clutter.png', image, [420, 230], '其他颜色和蓝色小噪点；蓝圆为目标。'))

    image = canvas('dark / V=50')
    cv2.circle(image, (220, 240), 60, blue(50), -1)
    scenes.append(('04_dark.png', image, [220, 240], '只把清晰圆亮度降到V=50；默认V下限70会漏检。'))

    image = canvas('same color distractor')
    cv2.circle(image, (220, 240), 45, blue(), -1)
    cv2.rectangle(image, (430, 150), (600, 330), blue(), -1)
    scenes.append(('05_same_color.png', image, [220, 240], '左侧圆是预定目标；右侧同色矩形更大，最大面积规则会误选右侧。'))

    image = canvas('absent')
    cv2.circle(image, (230, 240), 60, (20, 140, 230), -1)
    cv2.rectangle(image, (420, 170), (570, 330), (50, 180, 40), -1)
    scenes.append(('06_absent.png', image, None, '没有蓝色目标；必须返回detected=false且三个数值为0。'))

    records = []
    for filename, image, center, note in scenes:
        ok, encoded = cv2.imencode('.png', image)
        if not ok:
            raise RuntimeError('图片编码失败：' + filename)
        path = dest / filename
        # 这些名称专用于合成图，真实照片请存放到另一个目录。
        encoded.tofile(path)
        reopened = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
        if reopened is None or not np.array_equal(image, reopened):
            raise RuntimeError('保存后校验失败：' + filename)
        records.append({'file': filename, 'width': 640, 'height': 480,
                        'intended_center_uv': center, 'note': note,
                        'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    manifest = {'synthetic': True, 'source': 'prepare_demo_images.py',
                'purpose': '算法预演；不代替本人照片、观察、ROS2运行或截图',
                'images': records}
    (ROOT / 'data/demo_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('已生成并回读验证 6 张合成图：data/images；来源：prepare_demo_images.py')


if __name__ == '__main__':
    main()
