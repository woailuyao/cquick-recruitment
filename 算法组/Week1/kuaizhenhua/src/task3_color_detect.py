"""任务三：HSV 颜色分割 → 开闭运算 → 轮廓过滤 → 矩形和质心。

用法：python src/task3_color_detect.py
真实图片：先将图片放入 data/real_images，再运行：
python src/task3_color_detect.py --input-dir data/real_images --output-dir outputs/real/task3
注意：输出的是颜色候选，无法仅凭颜色确认物体身份。
"""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from common import (
    PROJECT_ROOT,
    read_image,
    save_image,
    list_images,
    write_json,
    safe_output_dir,
    montage,
)

# 中间结果按步骤编号；mask.png 和 result.png 保留任务书指定的文件名。
# 第 3、6 步分别对应这两个固定接口，因此其余步骤的编号不连续。
OUTPUT_IMAGE_FILES = {
    'mask_raw': '01_mask_raw.png',
    'mask_opened': '02_mask_opened.png',
    'mask': 'mask.png',
    'mask_filtered': '04_mask_filtered.png',
    'extracted': '05_extracted.png',
    'result': 'result.png',
}


def load_config(path):
    """读取参数文件，并检查 HSV、形态学核和面积门槛的有效范围。"""
    cfg = json.loads(Path(path).read_text(encoding='utf-8-sig'))

    # 普通 uint8 HSV：H 为 0～179，S 和 V 为 0～255。
    for name in ('hsv_lower', 'hsv_upper'):
        value = cfg.get(name)
        if (
            not isinstance(value, list)
            or len(value) != 3
            or any(type(v) is not int for v in value)
        ):
            raise ValueError(f'{name} must contain three integers.')
        if any(
            not 0 <= v <= limit
            for v, limit in zip(value, (179, 255, 255))
        ):
            raise ValueError(f'{name}: H=0..179, S/V=0..255.')
    if any(
        a > b for a, b in zip(cfg['hsv_lower'], cfg['hsv_upper'])
    ):
        raise ValueError('Each HSV lower bound must be <= its upper bound.')

    # 使用奇数边长，保证核有中心像素；1×1 的核不会改变掩膜。
    for key in ('open_kernel', 'close_kernel'):
        v = cfg.get(key)
        if type(v) is not int or v < 1 or v % 2 == 0:
            raise ValueError(
                f'{key} must be a positive odd integer; 1 disables it.'
            )
    if (
        not isinstance(cfg.get('min_area'), (int, float))
        or not np.isfinite(cfg['min_area'])
        or cfg['min_area'] <= 0
    ):
        raise ValueError('min_area must be positive and finite.')
    return cfg


def detect(image, cfg):
    """处理一张 BGR 图，返回各阶段图像和全部合格候选的几何信息。"""
    # 1. 把颜色转换成 HSV，同时满足三个通道范围的像素才会变白。
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    raw = cv2.inRange(
        hsv,
        np.array(cfg['hsv_lower'], np.uint8),
        np.array(cfg['hsv_upper'], np.uint8),
    )

    # 2. 开运算尝试去小白点，闭运算尝试填小黑孔、连接小缺口。
    # 核过大可能删掉小目标或合并相邻区域，因此保留中间掩膜供对比。
    opened = cv2.morphologyEx(
        raw,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (cfg['open_kernel'],) * 2
        ),
    )
    mask = cv2.morphologyEx(
        opened,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (cfg['close_kernel'],) * 2
        ),
    )

    # 3. 只找最外层轮廓，按面积从大到小编号。
    # 保留全部合格候选；最大的候选也可能是同色背景，不能直接当作目标。
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    result = image.copy()
    filtered = np.zeros_like(mask)
    records = []

    for contour in contours:
        # 4. 按轮廓几何面积过滤小噪声，单位是像素平方。
        # 外轮廓面积不扣内部孔洞，也不等于掩膜白色像素的数量。
        area = float(cv2.contourArea(contour))
        if area < cfg['min_area']:
            continue

        # 5. 用矩计算质心，先排除零面积轮廓，避免除以零。
        # 坐标按 (x, y) 保存；它可能不同于外接矩形的中心。
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            continue
        cx, cy = (
            moments['m10'] / moments['m00'],
            moments['m01'] / moments['m00'],
        )
        x, y, w, h = cv2.boundingRect(contour)
        number = len(records) + 1
        records.append({
            'id': number,
            'center_xy': [round(cx, 3), round(cy, 3)],
            'contour_area_px2': area,
            'bbox_xywh': [x, y, w, h],
        })

        # 6. 在副本上依次画绿色轮廓、橙黄色框、红色质心和文字。
        # 框后画，重合处会盖住绿色轮廓；矩形右下角减 1 对应像素边界。
        cv2.drawContours(filtered, [contour], -1, 255, cv2.FILLED)
        cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
        cv2.rectangle(
            result, (x, y), (x + w - 1, y + h - 1), (0, 180, 255), 2
        )
        cv2.circle(result, (round(cx), round(cy)), 5, (0, 0, 255), -1)
        cv2.putText(
            result,
            f'#{number} ({cx:.1f},{cy:.1f}) A={area:.0f}',
            (max(0, x), max(20, y - 9)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (10, 10, 10),
            2,
            cv2.LINE_AA,
        )

    # 轮廓填充会填满内部孔洞，再与原掩膜相交，把这些孔保留下来。
    filtered = cv2.bitwise_and(filtered, mask)
    if not records:
        # 未检出时保留空列表，不用 (0, 0) 伪装成有效中心坐标。
        cv2.putText(
            result,
            'No candidate detected',
            (15, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    # 掩膜非零的位置保留原图颜色，其余位置变黑。
    return {
        'hsv': hsv,
        'mask_raw': raw,
        'mask_opened': opened,
        'mask': mask,
        'mask_filtered': filtered,
        'extracted': cv2.bitwise_and(image, image, mask=filtered),
        'result': result,
        'detections': records,
    }


def main():
    """逐图运行检测，把各阶段图片和数值分别存入对应图片的子目录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--input-dir', type=Path, default=PROJECT_ROOT / 'data/images'
    )
    parser.add_argument(
        '--output-dir', type=Path, default=PROJECT_ROOT / 'outputs/task3'
    )
    parser.add_argument(
        '--config', type=Path, default=PROJECT_ROOT / 'src/configs/blue.json'
    )
    args = parser.parse_args()

    # 输入输出目录分开，先检查路径与配置，避免覆盖原图。
    images = list_images(args.input_dir)
    output = safe_output_dir(args.output_dir, args.input_dir)
    cfg = load_config(args.config)
    all_records = []
    for path in images:
        img = read_image(path)
        data = detect(img, cfg)

        # 一张输入图对应一个同名子目录，mask.png 等文件不会互相覆盖。
        # save_image 会创建目录，并在保存后重新读取 PNG 检查结果。
        dest = output / path.stem
        for key, filename in OUTPUT_IMAGE_FILES.items():
            save_image(dest / filename, data[key])
        save_image(
            dest / '07_comparison.png',
            montage([
                ('Input', img),
                ('HSV raw mask', data['mask_raw']),
                ('After opening / closing', data['mask']),
                ('After area filtering', data['mask_filtered']),
                ('Extracted color', data['extracted']),
                ('Contours / centroid', data['result']),
            ]),
        )

        # 图像哈希、配置和几何信息一起保存，方便核对同一次实验。
        record = {
            'image': path.name,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'shape_hwc': list(img.shape),
            'config': cfg,
            'coordinate_system': (
                'image pixels; origin top-left; x right; y down'
            ),
            'area_definition': (
                'external contour geometric area in pixel^2, '
                'not count of white pixels; holes are not subtracted'
            ),
            'candidate_count': len(data['detections']),
            'raw_white_pixels': int(np.count_nonzero(data['mask_raw'])),
            'clean_white_pixels': int(np.count_nonzero(data['mask'])),
            'detections': data['detections'],
            'files': {**OUTPUT_IMAGE_FILES, 'comparison': '07_comparison.png'},
        }
        write_json(dest / 'detections.json', record)
        all_records.append(record)
        print(
            f"{path.name}: {record['candidate_count']} candidate(s); "
            f"{record['detections']}"
        )

    # 汇总文件放在本次任务输出目录内，不额外散落到项目根目录。
    write_json(output / 'detections.json', all_records)
    print(f'Saved task3 results: {output}')


if __name__ == '__main__':
    main()
