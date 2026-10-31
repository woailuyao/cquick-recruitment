"""生成 5 张带真值掩膜的合成练习图；这些图不是实拍照片。

直接运行：python src/make_demo_data.py
重新生成：python src/make_demo_data.py --force

合成图便于检查程序是否正常，并不能替代真实场景测试。
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

from common import PROJECT_ROOT, save_image, write_json


WIDTH, HEIGHT = 640, 480
DEFAULT_SEED = 20261020
TARGET_HSV = (110, 200, 210)


def hsv_color(h, s, v):
    """OpenCV 的 H 范围为 0～179；输出 BGR 颜色供绘图使用。"""
    pixel = np.array([[[h, s, v]]], dtype=np.uint8)
    return tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


def background(value=235):
    return np.full((HEIGHT, WIDTH, 3), value, dtype=np.uint8)


def blank_mask():
    return np.zeros((HEIGHT, WIDTH), dtype=np.uint8)


def rectangle_mask():
    mask = blank_mask()
    # numpy 区间不含终点，因此恰好是 160 × 120 个像素。
    mask[170:290, 170:330] = 255
    return mask


def paint_target(image, mask, color):
    image[mask != 0] = color


def label_image(image, title):
    # OpenCV 内置字体只支持 ASCII；标题区域避开目标区域。
    cv2.rectangle(image, (0, 0), (WIDTH - 1, 47), (248, 248, 248), cv2.FILLED)
    cv2.putText(image, f'SYNTHETIC / {title}', (14, 31), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (35, 35, 35), 1, cv2.LINE_AA)


def mask_center(mask):
    moments = cv2.moments(mask, binaryImage=True)
    return [round(moments['m10'] / moments['m00'], 3),
            round(moments['m01'] / moments['m00'], 3)]


def create_scenes(seed):
    rng = np.random.default_rng(seed)
    blue = hsv_color(*TARGET_HSV)
    scenes = []

    image, mask = background(), rectangle_mask()
    paint_target(image, mask, blue)
    scenes.append(('01_clean', 'Clean rectangle', image, mask, 1,
                   'A 160 x 120 pixel blue rectangle on a uniform light background.',
                   '均匀浅色背景上的蓝色矩形，宽 160、高 120 像素。', None))

    image, mask = background(225), blank_mask()
    cv2.circle(mask, (400, 250), 65, 255, cv2.FILLED)
    cv2.rectangle(image, (45, 115), (205, 240), hsv_color(18, 220, 220), cv2.FILLED)
    cv2.rectangle(image, (110, 325), (270, 410), hsv_color(60, 180, 185), cv2.FILLED)
    # 零碎蓝点也会通过颜色阈值；形态学和面积筛选应排除它们。
    # 留出圆目标周围 20 像素空隙，避免蓝点连接目标而改变目标边缘。
    for _ in range(110):
        x, y = int(rng.integers(15, WIDTH - 15)), int(rng.integers(65, HEIGHT - 15))
        if (x - 400) ** 2 + (y - 250) ** 2 > 85 ** 2:
            cv2.circle(image, (x, y), 1, blue, cv2.FILLED)
    paint_target(image, mask, blue)
    scenes.append(('02_clutter', 'Clutter and blue speckles', image, mask, 1,
                   'A blue circle with orange/green rectangles and tiny blue speckles.',
                   '蓝色圆形目标，旁边有橙色、绿色矩形及微小蓝色噪点。', None))

    # 在有符号整数中叠加噪声，避免 uint8 加减出现溢出。
    noise = rng.integers(-6, 7, size=(HEIGHT, WIDTH, 3), dtype=np.int16)
    image = np.clip(np.full((HEIGHT, WIDTH, 3), 238, np.int16) + noise, 0, 255).astype(np.uint8)
    mask = blank_mask()
    cv2.ellipse(mask, (320, 255), (32, 25), 0, 0, 360, 255, cv2.FILLED)
    paint_target(image, mask, blue)
    scenes.append(('03_small', 'Small target and background noise', image, mask, 1,
                   'A small blue ellipse (semi-axes 32 and 25 pixels) on a noisy light background.',
                   '浅色低幅噪声背景上的小蓝色椭圆，长短半轴为 32 和 25 像素。', None))

    image, mask = background(90), rectangle_mask()
    paint_target(image, mask, hsv_color(110, 200, 50))
    scenes.append(('04_dark_fail', 'Deliberate failure: dark target', image, mask, 0,
                   'A real target is present, but its HSV value V=50 is below the configured lower bound V=70.',
                   '目标确实存在，但亮度 V=50 低于默认阈值下限 70，预期漏检。',
                   'false_negative: target V=50 is below default V lower bound 70'))

    image, mask = background(), rectangle_mask()
    paint_target(image, mask, blue)
    cv2.circle(image, (500, 285), 90, blue, cv2.FILLED)
    scenes.append(('05_same_color_fail', 'Deliberate failure: same color', image, mask, 2,
                   'The rectangle is the true target; a separate larger blue circle is a same-color distractor.',
                   '左侧矩形是真实目标，右侧更大的同色圆形是干扰物；纯颜色算法会输出两个候选。',
                   'false_positive: same-color distractor cannot be rejected by color and area alone'))

    for stem, title, image, mask, count, description_en, description_zh, failure in scenes:
        label_image(image, title)
    return scenes


def describe_path(path):
    """默认路径保存为项目相对路径，便于移动和压缩整个项目。"""
    try:
        return path.resolve().relative_to(PROJECT_ROOT.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=PROJECT_ROOT / 'data/images',
                        help='Synthetic image directory; default: data/images')
    parser.add_argument('--mask-dir', type=Path, default=PROJECT_ROOT / 'data/ground_truth',
                        help='True-target masks; default: data/ground_truth')
    parser.add_argument('--manifest', type=Path, default=PROJECT_ROOT / 'data/demo_manifest.json')
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED)
    parser.add_argument('--force', action='store_true', help='Replace the five named demo files and manifest')
    args = parser.parse_args()
    if args.seed < 0:
        parser.error('--seed must be a non-negative integer.')
    if args.output_dir.resolve() == args.mask_dir.resolve():
        parser.error('--output-dir and --mask-dir must be different directories.')

    scenes = create_scenes(args.seed)
    destinations = [args.manifest]
    for scene in scenes:
        destinations.extend((args.output_dir / f'{scene[0]}.png', args.mask_dir / f'{scene[0]}.png'))
    if len({str(path.resolve()).casefold() for path in destinations}) != len(destinations):
        parser.error('Output paths overlap; choose distinct image, mask and manifest paths.')
    existing = [str(path) for path in destinations if path.exists()]
    if existing and not args.force:
        parser.error('Existing demo files will not be overwritten. Use --force to replace them:\n'
                     + '\n'.join(existing))
    if any(path.is_dir() for path in destinations):
        parser.error('An output file path points to a directory; choose another output path.')

    manifest = {
        'schema_version': 1,
        'synthetic': True,
        'seed': args.seed,
        'source_script': 'src/make_demo_data.py',
        'provenance': 'Generated locally with OpenCV geometry and NumPy; no downloaded or real photos.',
        'image_size_wh': [WIDTH, HEIGHT],
        'target_hsv_opencv': list(TARGET_HSV),
        'coordinate_system': 'image pixels; origin top-left; x right; y down',
        'mask_definition': 'uint8 PNG; 255=true target, 0=background or distractor; exactly one true target per image',
        'default_detector': {'hsv_lower': [95, 80, 70], 'hsv_upper': [130, 255, 255],
                             'open_kernel': 3, 'close_kernel': 5, 'kernel_shape': 'ellipse', 'min_area': 500},
        'expected_candidate_counts': [1, 1, 1, 0, 2],
        'interpretation': 'Candidate counts are expected color-detector outputs, not ground-truth object counts. The last two cases intentionally demonstrate failure.',
        'images': [],
    }
    for stem, title, image, mask, count, description_en, description_zh, failure in scenes:
        image_path, mask_path = args.output_dir / f'{stem}.png', args.mask_dir / f'{stem}.png'
        save_image(image_path, image)
        save_image(mask_path, mask)
        manifest['images'].append({
            'filename': image_path.name,
            'image_path': describe_path(image_path),
            'mask_path': describe_path(mask_path),
            'synthetic': True,
            'scene_en': description_en,
            'scene_zh': description_zh,
            'expected_candidate_count': count,
            'true_target_count': 1,
            'true_target_center_xy': mask_center(mask),
            'true_target_pixels': int(np.count_nonzero(mask)),
            'deliberate_failure': failure,
        })
        print(f'{image_path.name}: true target center={mask_center(mask)}, expected candidates={count}')
    write_json(args.manifest, manifest)
    print(f'Saved 5 synthetic images, 5 true-target masks and manifest: {args.manifest}')
    print('These synthetic images do not replace testing on real photos.')


if __name__ == '__main__':
    main()
