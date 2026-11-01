"""Compose deliverable comparison figures with a real capture-time caption strip."""
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SHOTS = ROOT / 'outputs' / 'screenshots'
SHOTS.mkdir(parents=True, exist_ok=True)
STAMP = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
CAPTION = (f'liyutong   {STAMP}   HSV (5,200,80)-(25,255,255)   '
           'min_area=100   morphology=none')


def read(path):
    image = cv2.imdecode(np.frombuffer(Path(path).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'cannot decode {path}')
    return image


def caption(height, width, text):
    bar = np.full((38, width, 3), 255, np.uint8)
    cv2.putText(bar, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (20, 20, 20), 1, cv2.LINE_AA)
    return bar


def fit(image, max_w):
    h, w = image.shape[:2]
    if w <= max_w:
        return image
    scale = max_w / w
    return cv2.resize(image, (max_w, round(h * scale)), interpolation=cv2.INTER_AREA)


def add_caption(image, text, max_w=1500):
    image = fit(image, max_w)
    width = image.shape[1]
    return np.vstack([caption(image.shape[0], width, text), image])


def write(name, image):
    ok, buf = cv2.imencode('.png', image)
    if not ok:
        raise OSError('encode failed')
    target = SHOTS / name
    target.write_bytes(buf.tobytes())
    print(target, image.shape)


write('02_result_comparison.png',
      add_caption(read(ROOT / 'outputs' / 'view' / 'view_original_vs_annotated.png'), CAPTION))
write('02_parameter_comparison.png',
      add_caption(read(ROOT / 'outputs' / 'real_summary' / 'parameter_comparisons.png'), CAPTION))
write('01_five_photo_summary.png',
      add_caption(read(ROOT / 'outputs' / 'real_summary' / 'five_photo_results.png'), CAPTION))
print('caption stamp:', STAMP)
