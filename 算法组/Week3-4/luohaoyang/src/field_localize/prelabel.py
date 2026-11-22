"""用现有模型给新抽的帧生成"预标注"，人只需要改错，不用从零点。"""

import glob
import sys
from pathlib import Path

import cv2
import torchvision  # noqa: F401
from ultralytics import YOLO

ROOT = Path(r'F:\robot\luohaoyang\week3_4\task2_markers')
model = YOLO(str(ROOT / 'runs' / 'markers_v6' / 'weights' / 'best.pt'))

img_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else \
    Path(r'F:\robot\luohaoyang\week3_4\task2_markers\dataset\images\batch_field2')
files = sorted(img_dir.glob('*.jpg'))
print(f'{len(files)} images, pre-labeling at conf=0.18 ...')

n_box = 0
for f in files:
    im = cv2.imread(str(f))
    r = model.predict(im, conf=0.18, imgsz=1280, device='0', verbose=False)[0]
    lines = []
    if r.boxes is not None:
        h, w = im.shape[:2]
        for b in r.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
            bw, bh = (x2 - x1) / w, (y2 - y1) / h
            # 统一到 250px 的框宽（现有数据集的规范）
            target = 250.0 / 1280.0
            bw = min(max(bw, target), 0.35)
            bh = min(max(bh, target), 0.35)
            cx = min(max(cx, bw / 2 + 0.002), 1 - bw / 2 - 0.002)
            cy = min(max(cy, bh / 2 + 0.002), 1 - bh / 2 - 0.002)
            lines.append(f'{int(b.cls[0])} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}')
    (img_dir / (f.stem + '.txt')).write_text('\n'.join(lines) + ('\n' if lines else ''),
                                             encoding='utf-8')
    n_box += len(lines)
    print(f'  {f.name}: {len(lines)} 个候选')

print(f'total pre-labels: {n_box}')