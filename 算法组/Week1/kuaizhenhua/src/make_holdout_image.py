"""新增一张未用于原五图演示的合成留出图，并用原参数核验。

在个人项目目录执行：python src/make_holdout_image.py
此图由代码绘制，不是实拍照片；单张合成图通过不代表真实场景准确率。
默认只新增文件或接受逐像素相同的已有文件，不覆盖不同内容。
"""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from common import read_image, save_image, write_json


# 项目整体移动后仍可定位，不依赖电脑用户名或终端当前目录。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
SEED = 20261027
WIDTH, HEIGHT = 640, 480
VERTICES = [[258, 116], [341, 149], [389, 223], [353, 303],
            [274, 343], [207, 271], [222, 191]]
TARGET_HSV = [112, 190, 195]
EXPECTED_CONFIG = {
    'hsv_lower': [95, 80, 70], 'hsv_upper': [130, 255, 255],
    'open_kernel': 3, 'close_kernel': 5, 'min_area': 500,
}


def hsv_to_bgr(values):
    """绘图颜色要按 BGR 顺序；把指定的一个 HSV 像素转换过来。"""
    pixel = np.array([[values]], dtype=np.uint8)
    return tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


def create_image():
    """先独立绘制真值，再把目标颜色放进去；真值不是算法预测结果。"""
    rng = np.random.default_rng(SEED)
    yy, xx = np.mgrid[:HEIGHT, :WIDTH]
    # 灰色渐变模拟背景亮度变化；小幅噪声使用固定种子，便于复现。
    gray = 205 + (25 * xx // WIDTH) + (15 * yy // HEIGHT)
    image = np.repeat(gray[:, :, None], 3, axis=2).astype(np.int16)
    image += rng.integers(-3, 4, size=image.shape, dtype=np.int16)
    image = np.clip(image, 0, 255).astype(np.uint8)

    # 干扰物使用非蓝颜色，形状和位置与原五图不同，且不遮挡目标。
    cv2.ellipse(image, (91, 151), (55, 32), 28, 0, 360,
                hsv_to_bgr([19, 210, 225]), cv2.FILLED)
    green_shape = np.array([[453, 316], [514, 287], [582, 338],
                            [552, 414], [476, 399]], dtype=np.int32)
    cv2.fillPoly(image, [green_shape], hsv_to_bgr([61, 185, 160]))
    cv2.line(image, (452, 110), (572, 184), hsv_to_bgr([2, 185, 200]), 13)

    # 七个顶点定义不规则多边形，避免继续只测试正矩形和圆形。
    truth = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    cv2.fillPoly(truth, [np.asarray(VERTICES, dtype=np.int32)], 255)
    image[truth != 0] = hsv_to_bgr(TARGET_HSV)
    cv2.rectangle(image, (0, 0), (WIDTH - 1, 47), (248, 248, 248), cv2.FILLED)
    cv2.putText(image, 'SYNTHETIC / HOLDOUT 06 / irregular polygon', (13, 30),
                cv2.FONT_HERSHEY_SIMPLEX, .58, (35, 35, 35), 1, cv2.LINE_AA)
    return image, truth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force', action='store_true',
                        help='明确允许替换本脚本同名输出；不会删除其他文件')
    parser.add_argument('--check-report', type=Path,
                        help='可选：另存核验 JSON；相对路径以当前终端目录为准')
    args = parser.parse_args()
    image_path = PROJECT_ROOT / 'data/images/holdout/06_holdout.png'
    mask_path = PROJECT_ROOT / 'data/ground_truth/06_holdout_mask.png'
    manifest_path = PROJECT_ROOT / 'data/holdout_manifest.json'
    image, truth = create_image()

    # 写入前同时检查两个文件，避免遇到用户同名图片时覆盖其内容。
    for path, expected in ((image_path, image), (mask_path, truth)):
        if path.is_dir():
            parser.error(f'输出文件位置是目录：{path}')
        if path.exists() and not args.force:
            try:
                matches = np.array_equal(read_image(path, cv2.IMREAD_UNCHANGED), expected)
            except (OSError, ValueError):
                matches = False
            if not matches:
                parser.error(f'已有文件与固定种子生成内容不同，未覆盖：{path}；确需替换时使用 --force。')
    for path, expected in ((image_path, image), (mask_path, truth)):
        if not path.exists() or args.force:
            save_image(path, expected)

    # 先生成并保存图，再直接调用原检测函数；不运行 task3 的输出保存入口。
    from task3_color_detect import detect, load_config
    config_path = PROJECT_ROOT / 'src/configs/blue.json'
    config = load_config(config_path)
    if any(config.get(key) != value for key, value in EXPECTED_CONFIG.items()):
        parser.error('blue.json 与原五图默认参数不同。留出测试不自动改动或重新调整参数。')
    restored = read_image(image_path)
    restored_truth = read_image(mask_path, cv2.IMREAD_UNCHANGED)
    result = detect(restored, config)
    predicted = result['mask_filtered'] != 0
    target = restored_truth != 0
    intersection = int(np.count_nonzero(predicted & target))
    union = int(np.count_nonzero(predicted | target))
    iou = intersection / union
    moments = cv2.moments(restored_truth, binaryImage=True)
    true_center = [moments['m10'] / moments['m00'], moments['m01'] / moments['m00']]
    contours, _ = cv2.findContours(restored_truth, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    true_contour_area = float(cv2.contourArea(contours[0]))
    detections = result['detections']
    if len(detections) == 1:
        center_error = float(np.linalg.norm(np.asarray(detections[0]['center_xy']) - true_center))
        area_relative_error = abs(detections[0]['contour_area_px2'] - true_contour_area) / true_contour_area
    else:
        center_error, area_relative_error = None, None
    file_bytes = image_path.stat().st_size
    passed = (len(detections) == 1 and iou >= .97 and center_error <= 2
              and area_relative_error <= .01 and file_bytes < 1_000_000)
    verification = {
        'passed': passed,
        'config': config,
        'parameters_adjusted_for_this_image': False,
        'candidate_count': len(detections),
        'detections': detections,
        'true_target_center_xy': [round(value, 6) for value in true_center],
        'true_target_contour_area_px2': true_contour_area,
        'true_target_pixel_count': int(np.count_nonzero(target)),
        'mask_iou': round(iou, 9),
        'centroid_error_px': round(center_error, 9) if center_error is not None else None,
        'contour_area_relative_error': round(area_relative_error, 9) if area_relative_error is not None else None,
        'image_file_bytes': file_bytes,
        'acceptance': {'candidate_count': 1, 'minimum_iou': .97,
                       'maximum_centroid_error_px': 2, 'maximum_area_relative_error': .01,
                       'image_smaller_than_bytes': 1_000_000},
        'scope': 'Single synthetic holdout image; not real-world accuracy or proof of personal practice.',
    }
    manifest = {
        'schema_version': 1,
        'synthetic': True,
        'source_script': 'src/make_holdout_image.py',
        'source': '本地 OpenCV 几何绘图与 NumPy 固定种子噪声；不是实拍，未使用外部图片。',
        'seed': SEED,
        'image_path': 'data/images/holdout/06_holdout.png',
        'mask_path': 'data/ground_truth/06_holdout_mask.png',
        'image_size_wh': [WIDTH, HEIGHT],
        'target_vertices_xy': VERTICES,
        'target_hsv_opencv': TARGET_HSV,
        'true_target_count': 1,
        'expected_candidate_count': 1,
        'mask_definition': '人工指定顶点生成的目标真值；255=目标，0=背景和干扰物；不是预测掩膜。',
        'coordinate_system': '原点左上角，x 向右、y 向下，单位像素。',
        'scene': '渐变带噪声背景上的蓝色不规则七边形，含橙色旋转椭圆、绿色多边形和红线干扰。',
        'holdout_status': '新增图未用于原五图演示；原 blue.json 参数保持不变。图案属于同类可控合成分布。',
        'image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
        'mask_sha256': hashlib.sha256(mask_path.read_bytes()).hexdigest(),
        'verification': verification,
    }
    if manifest_path.exists() and not args.force:
        existing = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
        if existing != manifest:
            parser.error('已有 holdout_manifest.json 与本次内容不同，未覆盖；确需替换时使用 --force。')
    if not manifest_path.exists() or args.force:
        write_json(manifest_path, manifest)
    if args.check_report:
        write_json(args.check_report, {
            'synthetic': True, 'seed': SEED,
            'image_path': manifest['image_path'], 'mask_path': manifest['mask_path'],
            'image_sha256': manifest['image_sha256'], 'mask_sha256': manifest['mask_sha256'],
            **verification,
        })
    print(f'Synthetic holdout: candidates={len(detections)}, IoU={iou:.6f}, passed={passed}')
    print(f'Image: {image_path}; bytes={file_bytes}')
    print(f'Detections: {detections}; centroid error={center_error}')
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
