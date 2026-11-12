"""Build a compact original-vs-annotated comparison that fits a normal viewer window."""
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs' / 'view'
OUT.mkdir(parents=True, exist_ok=True)


def read(path):
    image = cv2.imdecode(np.frombuffer(Path(path).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'cannot decode {path}')
    return image


def fit(image, max_h, max_w):
    h, w = image.shape[:2]
    scale = min(max_w / w, max_h / h)
    return cv2.resize(image, (max(1, round(w * scale)), max(1, round(h * scale))), interpolation=cv2.INTER_AREA)


def panel(image, title, max_h=220, max_w=560):
    thumb = fit(image, max_h, max_w)
    canvas = np.full((max_h + 46, max_w + 20, 3), 250, np.uint8)
    y = 40 + (max_h - thumb.shape[0]) // 2
    x = 10 + (max_w - thumb.shape[1]) // 2
    canvas[y:y + thumb.shape[0], x:x + thumb.shape[1]] = thumb
    cv2.putText(canvas, title, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (20, 20, 20), 2, cv2.LINE_AA)
    return canvas


pairs = [
    ('images/clear.jpg', 'outputs/real_selected/000001_clear_annotated.png', 'clear'),
    ('images/background.jpg', 'outputs/real_selected/000003_background_annotated.png', 'background'),
]
rows = []
for original, annotated, name in pairs:
    rows.append(np.hstack([panel(read(ROOT / original), f'{name}: original'),
                           panel(read(ROOT / annotated), f'{name}: annotated')]))
sheet = np.vstack(rows)
ok, encoded = cv2.imencode('.png', sheet)
if not ok:
    raise OSError('encode failed')
target = OUT / 'view_original_vs_annotated.png'
target.write_bytes(encoded.tobytes())
print(target, sheet.shape)
