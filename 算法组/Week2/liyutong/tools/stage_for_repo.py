"""把提交目录整理成算法组 Week2 规范结构，并压缩过大图片。

目标结构：
    算法组/Week2/liyutong/
        README.md  学习记录.md  SCREENSHOTS.md  STATUS.md  INSTALL_UBUNTU.md
        src/  tools/
        data/images/
        outputs/screenshots/  outputs/selected/  outputs/experiment/  outputs/ros_logs/
"""
from pathlib import Path
import re
import shutil
import sys

import cv2
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
REPO = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path.home() / 'cquick-recruitment'
TARGET = REPO / '算法组' / 'Week2' / 'liyutong'
MAX_DIM = 1400
LIMIT = 1024 * 1024


def read(path):
    data = Path(path).read_bytes()
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'cannot decode {path}')
    return image


def save_jpeg(path, image, quality=85):
    path.parent.mkdir(parents=True, exist_ok=True)
    h, w = image.shape[:2]
    scale = min(1.0, MAX_DIM / max(h, w))
    if scale < 1.0:
        image = cv2.resize(image, (round(w * scale), round(h * scale)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise OSError(f'encode failed: {path}')
    path.write_bytes(buf.tobytes())
    return path.stat().st_size


def copy(src, dest):
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)


if TARGET.exists():
    shutil.rmtree(TARGET)
TARGET.mkdir(parents=True)

# 文档：路径改为 data/images，其余保持原样
for name in ('REAL_PHOTO_REPORT.md', 'SCREENSHOTS.md', 'STATUS.md', 'INSTALL_UBUNTU.md', '学习记录.md'):
    text = (PROJECT / name).read_text(encoding='utf-8')
    for photo in ('clear', 'small', 'background', 'lighting', 'absent'):
        text = text.replace(f'images/{photo}.jpg', f'data/images/{photo}.jpg')
    text = text.replace('保存在 images/', '保存在 data/images/')
    text = text.replace('images/            5 张测试照片', 'data/images/      5 张测试照片')
    text = text.replace('tools/             参数对比、自检、截图脚本', 'tools/            参数对比、自检、截图脚本')
    out = TARGET / ('README.md' if name == 'REAL_PHOTO_REPORT.md' else name)
    out.write_text(text, encoding='utf-8')

# 源码与脚本
for folder in ('src', 'tools'):
    shutil.copytree(PROJECT / folder, TARGET / folder,
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.egg-info'))

# 原始照片
(TARGET / 'data' / 'images').mkdir(parents=True, exist_ok=True)
for photo in ('clear', 'small', 'background', 'lighting', 'absent'):
    copy(PROJECT / 'images' / f'{photo}.jpg', TARGET / 'data' / 'images' / f'{photo}.jpg')

# 选定参数下的标注结果：转 JPEG 并限尺寸
selected = TARGET / 'outputs' / 'selected'
selected.mkdir(parents=True, exist_ok=True)
for src in sorted((PROJECT / 'outputs' / 'real_selected').glob('*.png')):
    if src.stem.endswith('_mask'):
        copy(src, selected / src.name)
    else:
        save_jpeg(selected / (src.stem + '.jpg'), read(src))
copy(PROJECT / 'outputs' / 'real_selected' / 'results.json', selected / 'results.json')

# 实验：只保留数值记录与 mask，标注图与选定结果重复故不再存一份
experiment = TARGET / 'outputs' / 'experiment'
for variant in ('baseline', 'saturation_B', 'area_B'):
    src_dir = PROJECT / 'outputs' / 'real_experiment_01' / variant
    dest_dir = experiment / variant
    dest_dir.mkdir(parents=True, exist_ok=True)
    copy(src_dir / 'results.json', dest_dir / 'results.json')
    for mask in sorted(src_dir.glob('*_mask.png')):
        copy(mask, dest_dir / mask.name)
copy(PROJECT / 'outputs' / 'real_experiment_01' / 'comparison.csv', experiment / 'comparison.csv')

# 截图：超过 1 MB 的转 JPEG
shots = TARGET / 'outputs' / 'screenshots'
shots.mkdir(parents=True, exist_ok=True)
for src in sorted((PROJECT / 'outputs' / 'screenshots').glob('*.png')):
    if src.stat().st_size <= LIMIT:
        copy(src, shots / src.name)
    else:
        save_jpeg(shots / (src.stem + '.jpg'), read(src), quality=90)

# ROS2 实测日志
copy_tree = PROJECT / 'outputs' / 'real_ros_check_001'
dest = TARGET / 'outputs' / 'ros_logs'
dest.mkdir(parents=True, exist_ok=True)
for src in sorted(copy_tree.rglob('*')):
    if src.is_file():
        relative = src.relative_to(copy_tree)
        if relative.parts[0] == 'ros_logs':
            continue  # 运行产生的中间日志，不必提交
        if src.suffix.lower() in ('.png', '.jpg'):
            continue
        copy(src, dest / relative)

# 体积检查
oversized = []
for path in TARGET.rglob('*'):
    if path.is_file() and path.stat().st_size > LIMIT:
        oversized.append((path.stat().st_size, str(path.relative_to(TARGET))))
total = sum(p.stat().st_size for p in TARGET.rglob('*') if p.is_file())
count = sum(1 for p in TARGET.rglob('*') if p.is_file())
print(f'目标目录: {TARGET}')
print(f'文件数: {count}   总大小: {total / 1048576:.1f} MB')
if oversized:
    print('超过 1 MB 的文件:')
    for size, name in sorted(oversized, reverse=True):
        print(f'  {size / 1048576:.2f} MB  {name}')
else:
    print('所有文件均在 1 MB 以内')
